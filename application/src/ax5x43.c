/*
 * Copyright (c) 2020 Ievgenii Meshcheriakov.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT onnn_ax5x43

#include <errno.h>
#include <device.h>
#include <init.h>
#include <drivers/spi.h>
#include <sys/byteorder.h>

#include "ax5x43.h"
#include "ax5x43_regs.h"

#define LOG_LEVEL CONFIG_AX5X43_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(ax5x43);

static void irq_handler(const struct device *port, struct gpio_callback *cb,
                        gpio_port_pins_t pins)
{
	struct ax5x43_drv_data *drv_data =
	        CONTAINER_OF(cb, struct ax5x43_drv_data, irq_cb);

	if (drv_data->callback == NULL) {
		return;
	}

	const struct device *dev = drv_data->dev;
	drv_data->callback(dev);
}

int ax5x43_configure_interrupt(const struct device *dev, bool enable)
{
	const struct ax5x43_config *config = dev->config;
	struct ax5x43_drv_data *drv_data = dev->data;

	if (!drv_data->irq_dev) {
		return -ENOTSUP;
	}

	gpio_flags_t flags = enable ? (GPIO_INT_ENABLE | GPIO_INT_EDGE_RISING) :
	                              GPIO_INT_DISABLE;

	return gpio_pin_interrupt_configure(drv_data->irq_dev, config->irq.pin,
	                                    flags);
}

void ax5x43_set_callback(const struct device *dev, ax5x43_callback callback)
{
	struct ax5x43_drv_data *drv_data = dev->data;
	drv_data->callback = callback;
}

static int ax5x43_read_regs(const struct device *dev, bool force_long,
                            uint16_t addr, size_t count, uint8_t *data)
{
	__ASSERT_NO_MSG(dev != NULL);
	__ASSERT_NO_MSG(data != NULL);

	struct ax5x43_drv_data *drv_data = dev->data;
	const bool long_access = force_long || (addr > AX5X43_LAST_DYN_REG);

	uint8_t tx_data[2];
	uint8_t rx_data[2];

	if (long_access) {
		tx_data[0] = (addr >> 8) | 0x70;
		tx_data[1] = addr & 0xff;
	} else {
		tx_data[0] = addr;
		rx_data[1] = 0;
	}

	const struct spi_buf tx_buf[] = {
		{ .buf = &tx_data, .len = long_access ? 2 : 1 },
	};

	const struct spi_buf rx_buf[] = {
		{ .buf = rx_data, .len = long_access ? 2 : 1 },
		{ .buf = data, .len = count },
	};

	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};

	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = ARRAY_SIZE(rx_buf),
	};

	int ret = spi_transceive(drv_data->spi, &drv_data->spi_cfg, &tx, &rx);
	spi_release(drv_data->spi, &drv_data->spi_cfg);
	if (ret < 0) {
		return ret;
	}

	return sys_get_be16(rx_data);
}

static int ax5x43_read_u8(const struct device *dev, uint16_t addr,
                          uint8_t *data)
{
	return ax5x43_read_regs(dev, false, addr, 1, data);
}

static int ax5x43_read_u16(const struct device *dev, uint16_t addr,
                           uint16_t *data)
{
	uint8_t buf[2];
	int ret = ax5x43_read_regs(dev, false, addr, 2, buf);
	*data = sys_get_be16(buf);
	return ret;
}

static int ax5x43_write_regs(const struct device *dev, bool force_long,
                             uint16_t addr, size_t count, const uint8_t *data)
{
	__ASSERT_NO_MSG(dev != NULL);
	__ASSERT_NO_MSG(data != NULL);

	struct ax5x43_drv_data *drv_data = dev->data;
	const bool long_access = force_long || (addr > AX5X43_LAST_DYN_REG);

	uint8_t tx_data[2] = { addr | 0x80 };
	uint8_t rx_data[2];

	if (long_access) {
		tx_data[0] = (addr >> 8) | 0xF0;
		tx_data[1] = addr & 0xFF;
	} else {
		tx_data[0] = addr | 0x80;
		rx_data[1] = 0;
	}

	const struct spi_buf tx_buf[] = {
		{ .buf = &tx_data, .len = long_access ? 2 : 1 },
		{ .buf = (uint8_t *)data, .len = count },
	};

	const struct spi_buf rx_buf[] = {
		{ .buf = rx_data, .len = long_access ? 2 : 1 },
	};

	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};

	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = ARRAY_SIZE(rx_buf),
	};

	int ret = spi_transceive(drv_data->spi, &drv_data->spi_cfg, &tx, &rx);
	spi_release(drv_data->spi, &drv_data->spi_cfg);
	if (ret < 0) {
		return ret;
	}

	return sys_get_be16(rx_data);
}

static int ax5x43_write_u8(const struct device *dev, uint16_t addr,
                           uint8_t data)
{
	return ax5x43_write_regs(dev, false, addr, 1, &data);
}

static int ax5x43_write_u16(const struct device *dev, uint16_t addr,
                            uint16_t data)
{
	uint8_t buf[2];
	sys_put_be16(data, buf);
	return ax5x43_write_regs(dev, false, addr, 2, buf);
}

static int ax5x43_reset(const struct device *dev)
{
	int ret;
	uint8_t reg;

	/* Ensure that the chip is not in the deep sleep mode. */
	do {
		ret = ax5x43_read_u8(dev, AX5X43_REG_REVISION, &reg);
		if (ret < 0) {
			return ret;
		}
	} while ((ret & AX5X43_STATUS_POWERED) == 0);

	__ASSERT(reg == AX5X43_REVISION, "Invalid chip revision %02x", reg);

	/* Reset the chip */
	ret = ax5x43_write_u8(dev, AX5X43_REG_PWRMODE,
	                      AX5X43_PWRMODE_RST | AX5X43_PWRMODE_POWERDOWN);
	if (ret < 0) {
		LOG_ERR("Failed to assert RST");
		return ret;
	}
	ret = ax5x43_write_u8(dev, AX5X43_REG_PWRMODE,
	                      AX5X43_PWRMODE_POWERDOWN);
	if (ret < 0) {
		LOG_ERR("Failed to clear RST");
		return ret;
	}

	/* Try accessing scratch register to ensure that chip is working. */
	ret = ax5x43_read_u8(dev, AX5X43_REG_SCRATCH, &reg);
	if (ret < 0) {
		LOG_ERR("Failed to read SCRATCH register");
		return ret;
	}

	if (reg != AX5X43_SCRATCH_VALUE) {
		LOG_ERR("Invalid SCRATCH value: %02x", reg);
		return -EIO;
	}

	ret = ax5x43_write_u8(dev, AX5X43_REG_SCRATCH, 42);
	if (ret < 0) {
		LOG_ERR("Failed to update SCRATCH register");
		return ret;
	}

	ret = ax5x43_read_u8(dev, AX5X43_REG_SCRATCH, &reg);
	if (ret < 0) {
		LOG_ERR("Failed to read back SCRATCH register");
		return ret;
	}

	if (reg != 42) {
		LOG_ERR("Invalid SCRATCH read back: %02x", reg);
		return -EIO;
	}

	return 0;
}

static int ax5x43_config_oscillator(const struct device *dev)
{
	/* Configuration for a clipped sinewave oscillator. */
	int ret = ax5x43_write_u8(dev, AX5X43_REG_XTALOSC, 0x04);
	if (ret < 0) {
		return ret;
	}

	ret = ax5x43_write_u8(dev, AX5X43_REG_XTALAMP, 0x00);
	if (ret < 0) {
		return ret;
	}

	ret = ax5x43_write_u8(dev, AX5X43_REG_XTALCAP, 0x00);

	return ret;
}

#define CONFIGURE_PIN(name, extra_flags)                                         \
	({                                                                       \
		if (config->name.dev) {                                          \
			drv_data->name##_dev =                                   \
			        device_get_binding(config->name.dev);            \
			if (!drv_data->name##_dev) {                             \
				LOG_ERR("Unable to get GPIO device for " #name); \
				return -ENODEV;                                  \
			}                                                        \
			int ret = gpio_pin_configure(                            \
			        drv_data->name##_dev, config->name.pin,          \
			        config->name.flags | (extra_flags));             \
			if (ret < 0) {                                           \
				return ret;                                      \
			}                                                        \
			LOG_DBG(#name " configured on %s:%u",                    \
			        config->name.dev, config->name.pin);             \
		}                                                                \
	})

static int ax5x43_init(const struct device *dev)
{
	const struct ax5x43_config *config = dev->config;
	struct ax5x43_drv_data *drv_data = dev->data;

	drv_data->dev = dev;
	drv_data->callback = NULL;

	drv_data->spi = device_get_binding(config->spi_dev_name);
	if (!drv_data->spi) {
		LOG_ERR("Unable to get SPI device");
		return -ENODEV;
	}

	if (config->cs.dev) {
		/* handle SPI CS thru GPIO if it is the case */
		drv_data->cs_ctrl.gpio_dev = device_get_binding(config->cs.dev);
		if (!drv_data->cs_ctrl.gpio_dev) {
			LOG_ERR("Unable to get GPIO SPI CS device");
			return -ENODEV;
		}
		drv_data->cs_ctrl.gpio_pin = config->cs.pin;
		drv_data->cs_ctrl.gpio_dt_flags = config->cs.flags;
		drv_data->cs_ctrl.delay = 1;

		drv_data->spi_cfg.cs = &drv_data->cs_ctrl;

		gpio_pin_configure(drv_data->cs_ctrl.gpio_dev, config->cs.pin,
		                   config->cs.flags | GPIO_OUTPUT_HIGH);

		LOG_DBG("CS configured on %s:%u", config->cs.dev,
		        config->cs.flags);
	}

	drv_data->spi_cfg.frequency = config->freq;
	drv_data->spi_cfg.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |
	                              SPI_WORD_SET(8) | SPI_LINES_SINGLE |
	                              SPI_HOLD_ON_CS | SPI_LOCK_ON;
	drv_data->spi_cfg.slave = config->slave;

	CONFIGURE_PIN(irq, GPIO_INPUT);

	if (drv_data->irq_dev) {
		gpio_init_callback(&drv_data->irq_cb, &irq_handler,
		                   BIT(config->irq.pin));
		gpio_add_callback(drv_data->irq_dev, &drv_data->irq_cb);
	}

	int ret = ax5x43_reset(dev);
	if (ret < 0) {
		LOG_ERR("Reset failed");
		return ret;
	}

	ret = ax5x43_config_oscillator(dev);
	if (ret < 0) {
		LOG_ERR("Oscillator configuration failed");
		return ret;
	}

	LOG_DBG("Initialization successful");

	return 0;
}

#define PIN_CONFIG(inst, name)                          \
	{                                               \
		.dev = DT_INST_GPIO_LABEL(inst, name),  \
		.pin = DT_INST_GPIO_PIN(inst, name),    \
		.flags = DT_INST_GPIO_FLAGS(inst, name) \
	}

#define AX5X43_INIT(inst)                                                 \
	static const struct ax5x43_config ax5x43_##inst##_config = {    \
		.spi_dev_name = DT_INST_BUS_LABEL(inst),                \
		.slave = DT_INST_REG_ADDR(inst),                        \
		.freq = DT_INST_PROP(inst, spi_max_frequency),          \
		.clock_freq = DT_INST_PROP(inst, clock_frequency),      \
		IF_ENABLED(DT_INST_SPI_DEV_HAS_CS_GPIOS(inst),          \
			(.cs = {                                        \
				DT_INST_SPI_DEV_CS_GPIOS_LABEL(inst),   \
				DT_INST_SPI_DEV_CS_GPIOS_PIN(inst),     \
				DT_INST_SPI_DEV_CS_GPIOS_FLAGS(inst),   \
			},))                                            \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, irq_gpios),      \
			(.irq = PIN_CONFIG(inst, irq_gpios),))          \
	}; \
	static struct ax5x43_drv_data ax5x43_##inst##_drvdata = {};       \
	DEVICE_DT_INST_DEFINE(inst, ax5x43_init, NULL,                    \
	                      &ax5x43_##inst##_drvdata,                   \
	                      &ax5x43_##inst##_config, POST_KERNEL,       \
	                      CONFIG_AX5X43_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(AX5X43_INIT)

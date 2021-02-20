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

static int read_short(const struct device *dev, uint16_t addr, size_t count,
                      uint8_t *data)
{
	__ASSERT_NO_MSG(dev != NULL);
	__ASSERT_NO_MSG(data != NULL);

	struct ax5x43_drv_data *drv_data = dev->data;
	int ret;

	__ASSERT_NO_MSG(addr <= AX5X43_LAST_DYN_REG);
	__ASSERT_NO_MSG(count > 0);

	uint8_t tx_data[] = { addr };
	uint8_t rx_data[1];

	const struct spi_buf tx_buf[] = {
		{ .buf = &tx_data, .len = sizeof(tx_data) },
	};

	const struct spi_buf rx_buf[] = {
		{ .buf = rx_data, .len = sizeof(rx_data) },
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

	ret = spi_transceive(drv_data->spi, &drv_data->spi_cfg, &tx, &rx);
	spi_release(drv_data->spi, &drv_data->spi_cfg);
	if (ret < 0) {
		return ret;
	}

	return rx_data[0] << 8;
}

static int read_long(const struct device *dev, uint16_t addr, size_t count,
                     uint8_t *data)
{
	__ASSERT_NO_MSG(dev != NULL);
	__ASSERT_NO_MSG(data != NULL);

	struct ax5x43_drv_data *drv_data = dev->data;
	int ret;

	__ASSERT_NO_MSG(addr < 0x1000);
	__ASSERT_NO_MSG(count > 0);

	uint8_t tx_data[] = { (addr >> 8) | 0x70, addr & 0xff };
	uint8_t rx_data[2];

	const struct spi_buf tx_buf[] = {
		{ .buf = &tx_data, .len = sizeof(tx_data) },
	};

	const struct spi_buf rx_buf[] = {
		{ .buf = rx_data, .len = sizeof(rx_data) },
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

	ret = spi_transceive(drv_data->spi, &drv_data->spi_cfg, &tx, &rx);
	spi_release(drv_data->spi, &drv_data->spi_cfg);
	if (ret < 0) {
		return ret;
	}

	return sys_get_be16(rx_data);
}

static int read_data(const struct device *dev, uint16_t addr, size_t count,
                     uint8_t *data)
{
	if (addr <= AX5X43_LAST_DYN_REG) {
		return read_short(dev, addr, count, data);
	} else {
		return read_long(dev, addr, count, data);
	}
}

static int read_u8(const struct device *dev, uint16_t addr, uint8_t *data)
{
	return read_data(dev, addr, 1, data);
}

static int read_u16(const struct device *dev, uint16_t addr, uint16_t *data)
{
	uint8_t buf[2];
	int ret = read_data(dev, addr, 2, buf);
	*data = sys_get_be16(buf);
	return ret;
}

static int write_short(const struct device *dev, uint16_t addr, size_t count,
                       const uint8_t *data)
{
	__ASSERT_NO_MSG(dev != NULL);
	__ASSERT_NO_MSG(data != NULL);

	struct ax5x43_drv_data *drv_data = dev->data;
	int ret;

	__ASSERT_NO_MSG(addr <= AX5X43_LAST_DYN_REG);
	__ASSERT_NO_MSG(count > 0);

	uint8_t tx_data[] = { addr | 0x80 };
	uint8_t rx_data[1];

	const struct spi_buf tx_buf[] = {
		{ .buf = &tx_data, .len = sizeof(tx_data) },
		{ .buf = (uint8_t *)data, .len = count },
	};

	const struct spi_buf rx_buf[] = {
		{ .buf = rx_data, .len = sizeof(rx_data) },
	};

	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};

	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = ARRAY_SIZE(rx_buf),
	};

	ret = spi_transceive(drv_data->spi, &drv_data->spi_cfg, &tx, &rx);
	spi_release(drv_data->spi, &drv_data->spi_cfg);
	if (ret < 0) {
		return ret;
	}

	return rx_data[0] << 8;
}

static int write_long(const struct device *dev, uint16_t addr, size_t count,
                      const uint8_t *data)
{
	__ASSERT_NO_MSG(dev != NULL);
	__ASSERT_NO_MSG(data != NULL);

	struct ax5x43_drv_data *drv_data = dev->data;
	int ret;

	__ASSERT_NO_MSG(addr < 0x1000);
	__ASSERT_NO_MSG(count > 0);

	uint8_t tx_data[] = { (addr >> 8) | 0xF0, addr & 0xFF };
	uint8_t rx_data[2];

	const struct spi_buf tx_buf[] = {
		{ .buf = &tx_data, .len = sizeof(tx_data) },
		{ .buf = (uint8_t *)data, .len = count },
	};

	const struct spi_buf rx_buf[] = {
		{ .buf = rx_data, .len = sizeof(rx_data) },
	};

	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};

	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = ARRAY_SIZE(rx_buf),
	};

	ret = spi_transceive(drv_data->spi, &drv_data->spi_cfg, &tx, &rx);
	spi_release(drv_data->spi, &drv_data->spi_cfg);
	if (ret < 0) {
		return ret;
	}

	return sys_get_be16(rx_data);
}

static int write_data(const struct device *dev, uint16_t addr, size_t count,
                      const uint8_t *data)
{
	if (addr <= AX5X43_LAST_DYN_REG) {
		return write_short(dev, addr, count, data);
	} else {
		return write_long(dev, addr, count, data);
	}
}

static int write_u8(const struct device *dev, uint16_t addr, uint8_t data)
{
	return write_data(dev, addr, 1, &data);
}

static int write_u16(const struct device *dev, uint16_t addr, uint16_t data)
{
	uint8_t buf[2];
	sys_put_be16(data, buf);
	return read_data(dev, addr, 2, buf);
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

	uint8_t reg;
	int ret = read_u8(dev, AX5X43_REG_REVISION, &reg);

	if (ret < 0) {
		LOG_ERR("Failed to read REVISION: %d", ret);
	} else {
		LOG_DBG("REVISION: %02x, status: %04x", (unsigned)reg, ret);
	}

	ret = read_u8(dev, AX5X43_REG_SCRATCH, &reg);

	if (ret < 0) {
		LOG_ERR("Failed to read SCRATCH: %d", ret);
	} else {
		LOG_DBG("SCRATCH: %02x, status: %04x", (unsigned)reg, ret);
	}

	return 0;
}

#define PIN_CONFIG(inst, name)                          \
	{                                               \
		.dev = DT_INST_GPIO_LABEL(inst, name),  \
		.pin = DT_INST_GPIO_PIN(inst, name),    \
		.flags = DT_INST_GPIO_FLAGS(inst, name) \
	}

#define AX5X43_INIT(inst)                                               \
	static const struct ax5x43_config ax5x43_##inst##_config = {    \
		.spi_dev_name = DT_INST_BUS_LABEL(inst),                \
		.slave = DT_INST_REG_ADDR(inst),                        \
		.freq = DT_INST_PROP(inst, spi_max_frequency),          \
		IF_ENABLED(DT_INST_SPI_DEV_HAS_CS_GPIOS(inst),          \
			(.cs = {                                        \
				DT_INST_SPI_DEV_CS_GPIOS_LABEL(inst),   \
				DT_INST_SPI_DEV_CS_GPIOS_PIN(inst),     \
				DT_INST_SPI_DEV_CS_GPIOS_FLAGS(inst),   \
			},))                                            \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, irq_gpios),      \
			(.irq = PIN_CONFIG(inst, irq_gpios),))          \
	};                                                              \
	static struct ax5x43_drv_data ax5x43_##inst##_drvdata = {};     \
	DEVICE_DT_INST_DEFINE(inst, ax5x43_init, NULL,                  \
	                      &ax5x43_##inst##_drvdata,                 \
	                      &ax5x43_##inst##_config, POST_KERNEL,     \
	                      CONFIG_AX5X43_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(AX5X43_INIT)

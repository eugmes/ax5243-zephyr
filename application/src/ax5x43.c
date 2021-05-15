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
#include <logging/log.h>
#include <inttypes.h>

#include "ax5x43.h"
#include "ax5x43_regs.h"

LOG_MODULE_REGISTER(ax5x43, CONFIG_AX5X43_LOG_LEVEL);

NET_BUF_POOL_DEFINE(ax5x43_pool, CONFIG_AX5X43_NET_BUF_COUNT,
                    AX5X43_MAX_MSG_SIZE, sizeof(struct ax5x43_user_data), NULL);

#define CHECK_RET(call)             \
	({                          \
		ret = (call);       \
		if (ret < 0) {      \
			return ret; \
		}                   \
	})

void ax5x43_set_callback(const struct device *dev, ax5x43_recv_callback cb)
{
	struct ax5x43_drv_data *drv_data = dev->data;
	drv_data->recv_cb = cb;
}

static void ax5x43_gpio_callback_handler_sem(const struct device *port,
                                             struct gpio_callback *cb,
                                             gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	struct ax5x43_drv_data *drv_data =
	        CONTAINER_OF(cb, struct ax5x43_drv_data, irq_callback_sem);
	const struct ax5x43_config *config = drv_data->dev->config;

	gpio_pin_interrupt_configure_dt(&config->irq, GPIO_INT_DISABLE);
	k_sem_give(&drv_data->irq_sem);
}

static void ax5x43_gpio_callback_handler_wq(const struct device *port,
                                            struct gpio_callback *cb,
                                            gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	struct ax5x43_drv_data *drv_data =
	        CONTAINER_OF(cb, struct ax5x43_drv_data, irq_callback_wq);
	const struct ax5x43_config *config = drv_data->dev->config;

	gpio_pin_interrupt_configure_dt(&config->irq, GPIO_INT_DISABLE);
	k_work_submit(&drv_data->work);
}

static int ax5x43_read_regs(const struct device *dev, bool force_long,
                            uint16_t addr, size_t count, uint8_t *data)
{
	__ASSERT_NO_MSG(dev != NULL);
	__ASSERT_NO_MSG(data != NULL);

	const struct ax5x43_config *config = dev->config;
	const bool long_access = force_long || (addr > AX5X43_LAST_DYN_REG);
	int ret;

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

	CHECK_RET(spi_transceive(config->bus, &config->bus_cfg, &tx, &rx));
	return sys_get_be16(rx_data);
}

static int ax5x43_read_u8(const struct device *dev, uint16_t addr,
                          uint8_t *data)
{
	int ret = ax5x43_read_regs(dev, false, addr, 1, data);
	LOG_DBG("RD %03" PRIX16 " = %02" PRIX8, addr, *data);
	return ret;
}

// static int ax5x43_read_u16(const struct device *dev, uint16_t addr,
//                            uint16_t *data)
// {
// 	uint8_t buf[2];
// 	int ret = ax5x43_read_regs(dev, false, addr, 2, buf);
// 	*data = sys_get_be16(buf);
// 	LOG_DBG("RD %03" PRIX16 " = %04" PRIX16, addr, *data);
// 	return ret;
// }

static int ax5x43_read_u24(const struct device *dev, uint16_t addr,
                           uint32_t *data)
{
	uint8_t buf[3];
	int ret = ax5x43_read_regs(dev, false, addr, 3, buf);
	*data = sys_get_be24(buf);
	LOG_DBG("RD %03" PRIX32 " = %06" PRIX16, addr, *data);
	return ret;
}

static int ax5x43_read_i24(const struct device *dev, uint16_t addr,
                           int32_t *data)
{
	int32_t val;
	int ret;
	CHECK_RET(ax5x43_read_u24(dev, addr, &val));
	val <<= 8;
	val >>= 8;
	*data = val;
	return 0;
}

static int ax5x43_write_regs(const struct device *dev, bool force_long,
                             uint16_t addr, size_t count, const uint8_t *data)
{
	__ASSERT_NO_MSG(dev != NULL);
	__ASSERT_NO_MSG(data != NULL);

	const struct ax5x43_config *config = dev->config;
	const bool long_access = force_long || (addr > AX5X43_LAST_DYN_REG);
	int ret;

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

	CHECK_RET(spi_transceive(config->bus, &config->bus_cfg, &tx, &rx));
	return sys_get_be16(rx_data);
}

static int ax5x43_write_u8(const struct device *dev, uint16_t addr,
                           uint8_t data)
{
	LOG_DBG("WR %03" PRIX16 " = %02" PRIX8, addr, data);
	return ax5x43_write_regs(dev, false, addr, 1, &data);
}

static int ax5x43_write_u16(const struct device *dev, uint16_t addr,
                            uint16_t data)
{
	LOG_DBG("WR %03" PRIX16 " = %04" PRIX16, addr, data);
	uint8_t buf[2];
	sys_put_be16(data, buf);
	return ax5x43_write_regs(dev, false, addr, 2, buf);
}

static int ax5x43_write_u24(const struct device *dev, uint16_t addr,
                            uint32_t data)
{
	LOG_DBG("WR %03" PRIX16 " = %06" PRIX32, addr, data);
	uint8_t buf[3];
	sys_put_be24(data, buf);
	return ax5x43_write_regs(dev, false, addr, 3, buf);
}

static int ax5x43_write_u32(const struct device *dev, uint16_t addr,
                            uint32_t data)
{
	LOG_DBG("WR %03" PRIX16 " = %08" PRIX32, addr, data);
	uint8_t buf[4];
	sys_put_be32(data, buf);
	return ax5x43_write_regs(dev, false, addr, 4, buf);
}

/**
 * Set the power mode register content together with XOEN and REFEN.
 */
static int set_pwrmode(const struct device *dev, uint8_t mode)
{
	mode |= AX5X43_PWRMODE_XOEN | AX5X43_PWRMODE_REFEN;
	return ax5x43_write_u8(dev, AX5X43_REG_PWRMODE, mode);
}

static int ax5x43_enable_rx_interrupt(const struct device *dev)
{
	const struct ax5x43_config *config = dev->config;
	struct ax5x43_drv_data *drv_data = dev->data;

	gpio_pin_interrupt_configure_dt(&config->irq, GPIO_INT_EDGE_TO_ACTIVE);

	/* Submit the work manually if the interrupt line is already active. */
	if (gpio_pin_get(config->irq.port, config->irq.pin)) {
		return k_work_submit(&drv_data->work);
	}

	return 0;
}

int ax5x43_start_rx(const struct device *dev)
{
	struct ax5x43_drv_data *drv_data = dev->data;
	const struct ax5x43_config *config = dev->config;
	int ret;

	/* Configure FIFO interrupt. */
	CHECK_RET(ax5x43_write_u16(dev, AX5X43_REG_IRQMASK,
	                           AX5X43_IRQ_FIFONOTEMPTY));

	CHECK_RET(set_pwrmode(dev, AX5X43_PWRMODE_FULLRX));

	CHECK_RET(gpio_add_callback(config->irq.port,
	                            &drv_data->irq_callback_wq));
	ax5x43_enable_rx_interrupt(dev);

	return 0;
}

int ax5x43_start_tx(const struct device *dev)
{
	return set_pwrmode(dev, AX5X43_PWRMODE_FULLTX);
}

/*
 * STM32 does not support level interrupts, so emulate them here.
 *
 * TODO: Find an MCU without this nonsence.
 */
static void ax5x43_wait_for_interrupt(const struct device *dev)
{
	const struct ax5x43_config *config = dev->config;
	struct ax5x43_drv_data *drv_data = dev->data;

	k_sem_reset(&drv_data->irq_sem);
	gpio_pin_interrupt_configure_dt(&config->irq, GPIO_INT_EDGE_TO_ACTIVE);

	/* Read the pin status first. */
	if (gpio_pin_get(config->irq.port, config->irq.pin)) {
		gpio_pin_interrupt_configure_dt(&config->irq, GPIO_INT_DISABLE);
		return;
	}

	k_sem_take(&drv_data->irq_sem, K_FOREVER);
}

static int ax5x43_read_fifo(const struct device *dev, size_t size, uint8_t *buf)
{
	return ax5x43_read_regs(dev, false, AX5X43_REG_FIFODATA, size, buf);
}

static int ax5x43_read_fifo_u8(const struct device *dev, uint8_t *data)
{
	return ax5x43_read_u8(dev, AX5X43_REG_FIFODATA, data);
}

static int ax5x43_read_fifo_u24(const struct device *dev, uint32_t *data)
{
	return ax5x43_read_u24(dev, AX5X43_REG_FIFODATA, data);
}

static int ax5x43_read_fifo_i24(const struct device *dev, int32_t *data)
{
	return ax5x43_read_i24(dev, AX5X43_REG_FIFODATA, data);
}

/**
 * Process a single FIFO chunk.
 *
 * This function assumes that the FIFO is not empty.
 */
static int ax5x43_process_fifo(const struct device *dev)
{
	struct ax5x43_drv_data *drv_data = dev->data;
	int ret;

	uint8_t chunk_type;
	CHECK_RET(ax5x43_read_fifo_u8(dev, &chunk_type));

	uint8_t data_size;
	uint8_t payload_size_enc = chunk_type >> 5;
	if (payload_size_enc <= 3) {
		data_size = payload_size_enc;
	} else if (payload_size_enc == 7) {
		/* Read the size of the following data. */
		CHECK_RET(ax5x43_read_fifo_u8(dev, &data_size));
	} else {
		LOG_ERR("Unknown size encoding: %02" PRIX8, chunk_type);
		/* TODO: restart the device. */
		k_panic();
	}

	/* Try to allocate a buffer if it is not already allocated. */
	if (drv_data->buf == 0) {
		drv_data->buf = net_buf_alloc(&ax5x43_pool, K_NO_WAIT);
		if (drv_data->buf == NULL) {
			LOG_ERR("Failed to allocate a buffer");
			goto drain_fifo;
		}
	}

	struct ax5x43_user_data *user_data = net_buf_user_data(drv_data->buf);

	if (chunk_type == AX5X43_CHUNK_TIMER) {
		drv_data->rx_state = AX5X43_RX_STATE_INITIAL;
	}

	switch (drv_data->rx_state) {
	case AX5X43_RX_STATE_INITIAL:
		if (chunk_type == AX5X43_CHUNK_TIMER) {
			net_buf_reset(drv_data->buf);
			memset(user_data, 0, sizeof(*user_data));
			user_data->timestamp = k_uptime_get();

			uint32_t timer;
			CHECK_RET(ax5x43_read_fifo_u24(dev, &timer));
			user_data->timer = timer;

			drv_data->rx_state = AX5X43_RX_STATE_TIMER;
		} else {
			goto drain_fifo;
		}
		break;
	case AX5X43_RX_STATE_TIMER:
		if (chunk_type == AX5X43_CHUNK_DATA) {
			uint8_t chunk_flags;
			CHECK_RET(ax5x43_read_fifo_u8(dev, &chunk_flags));

			if ((chunk_flags & AX5X43_DATA_PKTSTART) == 0) {
				LOG_ERR("Bad flags: 0x%0" PRIx8
				        ", length: %" PRIu8,
				        chunk_flags, data_size);
				goto drain_fifo;
			}

			uint8_t *p = net_buf_add(drv_data->buf, data_size - 1);
			CHECK_RET(ax5x43_read_fifo(dev, data_size - 1, p));

			if ((chunk_flags & AX5X43_DATA_PKTEND) != 0) {
				drv_data->rx_state = AX5X43_RX_STATE_DATA_END;
			} else {
				drv_data->rx_state = AX5X43_RX_STATE_DATA;
			}
		} else {
			goto drain_fifo;
		}
		break;
	case AX5X43_RX_STATE_DATA:
		if (chunk_type == AX5X43_CHUNK_DATA) {
			uint8_t chunk_flags;
			CHECK_RET(ax5x43_read_fifo_u8(dev, &chunk_flags));

			if ((chunk_flags & AX5X43_DATA_PKTSTART) != 0) {
				goto drain_fifo;
			}

			uint8_t *p = net_buf_add(drv_data->buf, data_size - 1);
			CHECK_RET(ax5x43_read_fifo(dev, data_size - 1, p));

			if ((chunk_flags & AX5X43_DATA_PKTEND) != 0) {
				drv_data->rx_state = AX5X43_RX_STATE_DATA_END;
			}
		} else {
			goto drain_fifo;
		}
		break;
	case AX5X43_RX_STATE_DATA_END:
		if (chunk_type == AX5X43_CHUNK_RSSI) {
			int8_t rssi;
			CHECK_RET(ax5x43_read_fifo_u8(dev, &rssi));
			user_data->rssi = rssi;
			drv_data->rx_state = AX5X43_RX_STATE_RSSI;
		} else {
			goto drain_fifo;
		}
		break;
	case AX5X43_RX_STATE_RSSI:
		if (chunk_type == AX5X43_CHUNK_RFFREQOFFS) {
			int32_t rf_freq_offs;
			CHECK_RET(ax5x43_read_fifo_i24(dev, &rf_freq_offs));
			user_data->rf_freq_offs = rf_freq_offs;

			if (drv_data->recv_cb != NULL) {
				const struct ax5x43_config *config = dev->config;
				user_data->channel_id = config->channel_id;

				struct net_buf *buf = drv_data->buf;
				// Discard CRC
				net_buf_remove_mem(buf, 2);

				drv_data->buf = NULL;
				drv_data->recv_cb(dev, buf);
			}

			drv_data->rx_state = AX5X43_RX_STATE_INITIAL;
		} else {
			goto drain_fifo;
		}
		break;
	}

	return 0;

drain_fifo:
	// TODO
	LOG_ERR("Unexpected chunk type 0x%02" PRIx8 ", state: %d", chunk_type,
	        drv_data->rx_state);
	k_panic();
	return 0;
}

static void ax5x43_recv_work_handler(struct k_work *work)
{
	struct ax5x43_drv_data *data =
	        CONTAINER_OF(work, struct ax5x43_drv_data, work);
	const struct device *dev = data->dev;

	for (;;) {
		uint8_t fifo_stat;

		int ret = ax5x43_read_u8(dev, AX5X43_REG_FIFOSTAT, &fifo_stat);
		if (ret < 0) {
			LOG_ERR("Failed to read FIFOSTAT: %d", ret);
			k_panic();
		}

		if ((fifo_stat & AX5X43_FIFOSTAT_EMPTY) != 0) {
			break;
		}

		ret = ax5x43_process_fifo(dev);
		if (ret < 0) {
			LOG_ERR("Failed to process FIFO: %d", ret);
			k_panic();
		}
	}

	ax5x43_enable_rx_interrupt(dev);
}

int ax5x43_send_packet(const struct device *dev, const uint8_t *buf,
                       size_t size)
{
	int ret;
	// TODO add FIFO space checks

	const uint8_t headers[] = {
		/* Preamble */
		AX5X43_CHUNK_REPEATDATA,
		AX5X43_REPEATDATA_NOCRC | AX5X43_REPEATDATA_UNENC,
		3,
		0x55,
		/* Packet data header */
		AX5X43_CHUNK_DATA,
		size + 1,
		AX5X43_DATA_PKTSTART | AX5X43_DATA_PKTEND,
	};

	CHECK_RET(ax5x43_write_regs(dev, false, AX5X43_REG_FIFODATA,
	                            sizeof(headers), headers));
	CHECK_RET(
	        ax5x43_write_regs(dev, false, AX5X43_REG_FIFODATA, size, buf));

	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_FIFOSTAT,
	                          AX5X43_FIFOCMD_COMMIT));

	return 0;
}

static int ax5x43_reset(const struct device *dev)
{
	int ret;
	uint8_t reg;

	/* Ensure that the chip is not in the deep sleep mode. */
	do {
		CHECK_RET(ax5x43_read_u8(dev, AX5X43_REG_REVISION, &reg));
	} while ((ret & AX5X43_STATUS_POWERED) == 0);

	__ASSERT(reg == AX5X43_REVISION, "Invalid chip revision %02" PRIX8,
	         reg);

	/* Reset the chip */
	CHECK_RET(set_pwrmode(dev,
	                      AX5X43_PWRMODE_RST | AX5X43_PWRMODE_POWERDOWN));
	CHECK_RET(set_pwrmode(dev, AX5X43_PWRMODE_POWERDOWN));

	/* Try accessing scratch register to ensure that chip is working. */
	CHECK_RET(ax5x43_read_u8(dev, AX5X43_REG_SCRATCH, &reg));

	if (reg != AX5X43_SCRATCH_VALUE) {
		LOG_ERR("Invalid SCRATCH value: %02x", reg);
		return -EIO;
	}

	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_SCRATCH, 42));
	CHECK_RET(ax5x43_read_u8(dev, AX5X43_REG_SCRATCH, &reg));
	if (reg != 42) {
		LOG_ERR("Invalid SCRATCH read back: %02" PRIX8, reg);
		return -EIO;
	}

	return 0;
}

/**
 * Initialize the performance registers.
 */
static int init_perf_regs(const struct device *dev)
{
	int ret;

	CHECK_RET(ax5x43_write_u8(dev, 0xF00, 0x0F));
	//CHECK_RET(ax5x43_write_u8(dev, 0xF0C, 0x00));
	CHECK_RET(ax5x43_write_u8(dev, 0xF0D, 0x03));

	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_XTALOSC, 0x04));
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_XTALAMP, 0x00));
	// TODO configure capacitors for XTAL
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_XTALCAP, 0x00));

	CHECK_RET(ax5x43_write_u8(dev, 0xF18, 0x06));
	CHECK_RET(ax5x43_write_u8(dev, 0xF1C, 0x07));
	CHECK_RET(ax5x43_write_u8(dev, 0xF21, 0x68));
	CHECK_RET(ax5x43_write_u8(dev, 0xF22, 0xFF));
	CHECK_RET(ax5x43_write_u8(dev, 0xF23, 0x84));
	CHECK_RET(ax5x43_write_u8(dev, 0xF26, 0x98));
	//CHECK_RET(ax5x43_write_u8(dev, 0xF30, 0x3F));
	//CHECK_RET(ax5x43_write_u8(dev, 0xF31, 0xF0));
	//CHECK_RET(ax5x43_write_u8(dev, 0xF32, 0x3F));
	//CHECK_RET(ax5x43_write_u8(dev, 0xF33, 0xF0));
	// Set to 0x28 if RFDIV is set, or to 0x08 otherwise
	CHECK_RET(ax5x43_write_u8(dev, 0xF34, 0x08));
	CHECK_RET(ax5x43_write_u8(dev, 0xF35, 0x11));
	CHECK_RET(ax5x43_write_u8(dev, 0xF44, 0x25));
	// Set to 0x06 for "Raw, Soft Bits" framing
	// CHECK_RET(ax5x43_write_u8(dev, 0xF0D, 0x00));

	return 0;
}

/**
 * Initialize registers controlling GPIO pins.
 */
static int init_pin_func_regs(const struct device *dev)
{
	int ret;

	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_PINFUNCDCLK, 0));
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_PINFUNCDATA, 0));
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_PINFUNCTCXO_EN, 0));
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_PINFUNCSYSCLK, 0));

	return 0;
}

static int ax5x43_wait_for_osc_startup(const struct device *dev)
{
	int ret;

	CHECK_RET(ax5x43_write_u16(dev, AX5X43_REG_IRQMASK,
	                           AX5X43_IRQ_XTALREADY));
	ax5x43_wait_for_interrupt(dev);
	CHECK_RET(ax5x43_write_u16(dev, AX5X43_REG_IRQMASK, 0));

	return 0;
}

static int ax5x43_wait_for_ppl_ranging(const struct device *dev)
{
	int ret;

	CHECK_RET(ax5x43_write_u16(dev, AX5X43_REG_IRQMASK,
	                           AX5X43_IRQ_PLLRNGDONE));
	ax5x43_wait_for_interrupt(dev);
	CHECK_RET(ax5x43_write_u16(dev, AX5X43_REG_IRQMASK, 0));

	return 0;
}

static uint32_t div24(uint32_t a, uint32_t b)
{
	uint64_t tmp = ((uint64_t)a) << 24;
	return tmp / b;
}

static int init_common_regs(const struct device *dev)
{
	const struct ax5x43_config *config = dev->config;
	int ret;
	uint8_t reg;

	uint32_t freqa = div24(config->carrier_freq, config->clock_freq);

	/* Always set the lower bit to avoid harmonics. */
	freqa |= 1;
	CHECK_RET(ax5x43_write_u32(dev, AX5X43_REG_FREQA, freqa));

	// TODO: enable external PLL filter cap

	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_PLLLOOP, 0x0A));
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_PLLCPI, 0x10));

	/* Configure the external inductor for the VCO */
	reg = BIT(4) | BIT(5);
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_PLLVCODIV, reg));

	/* Perform VCO autoraning. */
	CHECK_RET(set_pwrmode(dev, AX5X43_PWRMODE_STANDBY));
	ax5x43_wait_for_osc_startup(dev);
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_PLLRANGINGA,
	                          AX5X43_PLLRANGING_RNGSTART | 8));

	ax5x43_wait_for_ppl_ranging(dev);
	CHECK_RET(ax5x43_read_u8(dev, AX5X43_REG_PLLRANGINGA, &reg));

	CHECK_RET(set_pwrmode(dev, AX5X43_PWRMODE_POWERDOWN));

	if (reg & AX5X43_PLLRANGING_RNGERR) {
		LOG_ERR("Ranging failed");
		return -EIO;
	}

	/* Transmitter parameters. */
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_MODULATION, 0x08));
	uint32_t txrate = div24(config->bitrate, config->clock_freq);
	CHECK_RET(ax5x43_write_u24(dev, AX5X43_REG_TXRATE, txrate));
	uint32_t fskdev = div24(config->bitrate / 4, config->clock_freq);
	CHECK_RET(ax5x43_write_u24(dev, AX5X43_REG_FSKDEV, fskdev));

	// Lower the TX power for testing.
	CHECK_RET(ax5x43_write_u16(dev, AX5X43_REG_TXPWRCOEFFB, 0x0));

	/* Receiver parameters. */
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_DECIMATION, 0x0B));
	CHECK_RET(ax5x43_write_u24(dev, AX5X43_REG_RXDATARATE, 0x003D8D));
	CHECK_RET(ax5x43_write_u16(dev, AX5X43_REG_IFFREQ, 0x3C8));

	CHECK_RET(ax5x43_write_u24(dev, AX5X43_REG_MAXRFOFFSET, 0x800285));
	CHECK_RET(ax5x43_write_u24(dev, AX5X43_REG_MAXDROFFSET, 0));

	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_RXPARAMSETS, 0xF4));

	uint16_t rx = AX5X43_RX_PARAM_SET0;
	CHECK_RET(ax5x43_write_u8(dev, rx + AX5X43_RX_AGCGAIN, 0xB4));
	CHECK_RET(ax5x43_write_u8(dev, rx + AX5X43_RX_AGCTARGET, 0x84));
	CHECK_RET(ax5x43_write_u8(dev, rx + AX5X43_RX_TIMEGAIN, 0xF8));
	CHECK_RET(ax5x43_write_u8(dev, rx + AX5X43_RX_DRGAIN, 0xF2));
	CHECK_RET(ax5x43_write_u8(dev, rx + AX5X43_RX_PHASEGAIN, 0xC3));
	CHECK_RET(ax5x43_write_u32(dev, rx + AX5X43_RX_FREQGAIN, 0x0F1F0707));
	CHECK_RET(ax5x43_write_u8(dev, rx + AX5X43_RX_AMPLGAIN, 0x06));
	CHECK_RET(ax5x43_write_u16(dev, rx + AX5X43_RX_FREQDEV, 0));
	CHECK_RET(ax5x43_write_u8(dev, rx + AX5X43_RX_BBOFFSRES, 0x00));

	rx = AX5X43_RX_PARAM_SET1;
	CHECK_RET(ax5x43_write_u8(dev, rx + AX5X43_RX_AGCGAIN, 0xB4));
	CHECK_RET(ax5x43_write_u8(dev, rx + AX5X43_RX_AGCTARGET, 0x84));
	CHECK_RET(ax5x43_write_u8(dev, rx + AX5X43_RX_AGCAHYST, 0x00));
	CHECK_RET(ax5x43_write_u8(dev, rx + AX5X43_RX_AGCMINMAX, 0x00));
	CHECK_RET(ax5x43_write_u8(dev, rx + AX5X43_RX_TIMEGAIN, 0xF6));
	CHECK_RET(ax5x43_write_u8(dev, rx + AX5X43_RX_DRGAIN, 0xF1));
	CHECK_RET(ax5x43_write_u8(dev, rx + AX5X43_RX_PHASEGAIN, 0xC3));
	CHECK_RET(ax5x43_write_u32(dev, rx + AX5X43_RX_FREQGAIN, 0x0F1F0707));
	CHECK_RET(ax5x43_write_u8(dev, rx + AX5X43_RX_AMPLGAIN, 0x06));
	CHECK_RET(ax5x43_write_u16(dev, rx + AX5X43_RX_FREQDEV, 0x0032));
	CHECK_RET(ax5x43_write_u8(dev, rx + AX5X43_RX_FOURFSK, 0x16));
	CHECK_RET(ax5x43_write_u8(dev, rx + AX5X43_RX_BBOFFSRES, 0x00));

	rx = AX5X43_RX_PARAM_SET3;
	CHECK_RET(ax5x43_write_u8(dev, rx + AX5X43_RX_AGCGAIN, 0xFF));
	CHECK_RET(ax5x43_write_u8(dev, rx + AX5X43_RX_AGCTARGET, 0x84));
	CHECK_RET(ax5x43_write_u8(dev, rx + AX5X43_RX_AGCAHYST, 0x00));
	CHECK_RET(ax5x43_write_u8(dev, rx + AX5X43_RX_AGCMINMAX, 0x00));
	CHECK_RET(ax5x43_write_u8(dev, rx + AX5X43_RX_TIMEGAIN, 0xF5));
	CHECK_RET(ax5x43_write_u8(dev, rx + AX5X43_RX_DRGAIN, 0xF0));
	CHECK_RET(ax5x43_write_u8(dev, rx + AX5X43_RX_PHASEGAIN, 0xC3));
	CHECK_RET(ax5x43_write_u32(dev, rx + AX5X43_RX_FREQGAIN, 0x0F1F0B0B));
	CHECK_RET(ax5x43_write_u8(dev, rx + AX5X43_RX_AMPLGAIN, 0x06));
	CHECK_RET(ax5x43_write_u16(dev, rx + AX5X43_RX_FREQDEV, 0x0032));
	CHECK_RET(ax5x43_write_u8(dev, rx + AX5X43_RX_FOURFSK, 0x16));
	CHECK_RET(ax5x43_write_u8(dev, rx + AX5X43_RX_BBOFFSRES, 0x00));

	CHECK_RET(ax5x43_write_u32(dev, AX5X43_REG_MATCH0PAT, 0xAACCAACC));
	CHECK_RET(ax5x43_write_u16(dev, AX5X43_REG_MATCH1PAT, 0xAAAA));
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_MATCH1LEN, 0x0A));
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_MATCH1MAX, 0x0A));

	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_TMGTXBOOST, 0x3E));
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_TMGTXSETTLE, 0x31));
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_TMGRXBOOST, 0x3E));
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_TMGRXSETTLE, 0x31));
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_TMGRXOFFSACQ, 0x00));
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_TMGRXCOARSEAGC, 0x7F));
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_TMGRXRSSI, 0x03));
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_TMGRXPREAMBLE2, 0x17));
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_RSSIABSTHR, 0xE3));
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_BGNDRSSITHR, 0));

	/* Encoding and framing */
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_ENCODING,
	                          AX5X43_ENCODING_INV | AX4X43_ENCODING_DIFF));
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_FRAMING,
	                          AX5X43_FRMMODE_HDLC | AX5X43_CRCMODE_CCITT));
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_PKTLENCFG, 0xF0));
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_PKTMAXLEN,
	                          AX5X43_MAX_MSG_SIZE));
	// TODO: accept residue too
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_PKTACCEPTFLAGS,
	                          AX5X43_ACCPT_LRGP));
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_PKTSTOREFLAGS,
	                          AX5X43_ST_TIMER | AX5X43_ST_RSSI |
	                                  AX5X43_ST_RFOFFS));

	// TODO make chunk size configurable
	CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_PKTCHUNKSIZE,
	                          AX5X43_PKTCHUNKSIZE_64));

	// FIXME: a better way to test the transmitter
	// CHECK_RET(ax5x43_write_u8(dev, AX5X43_REG_PINFUNCDATA, 4));
	// CHECK_RET(set_pwrmode(dev, AX5X43_PWRMODE_FULLTX));
	return 0;
}

static int ax5x43_init(const struct device *dev)
{
	const struct ax5x43_config *config = dev->config;
	struct ax5x43_drv_data *drv_data = dev->data;
	int ret;

	drv_data->dev = dev;

	k_work_init(&drv_data->work, ax5x43_recv_work_handler);

	ret = gpio_pin_configure_dt(&config->irq, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure IRQ GPIO");
		return ret;
	}

	gpio_init_callback(&drv_data->irq_callback_sem,
	                   ax5x43_gpio_callback_handler_sem,
	                   BIT(config->irq.pin));
	gpio_init_callback(&drv_data->irq_callback_wq,
	                   ax5x43_gpio_callback_handler_wq,
	                   BIT(config->irq.pin));

	ret = gpio_add_callback(config->irq.port, &drv_data->irq_callback_sem);
	if (ret < 0) {
		LOG_ERR("Failed to set GPIO callback");
		return ret;
	}

	k_sem_init(&drv_data->irq_sem, 0, 1);

	ret = ax5x43_reset(dev);
	if (ret < 0) {
		LOG_ERR("Reset failed");
		return ret;
	}

	ret = init_perf_regs(dev);
	if (ret < 0) {
		LOG_ERR("Performance registers configuration failed");
		return ret;
	}

	ret = init_pin_func_regs(dev);
	if (ret < 0) {
		LOG_ERR("Pin function registers configuration failed");
		return ret;
	}

	ret = init_common_regs(dev);
	if (ret < 0) {
		LOG_ERR("Failed to initialize common registers");
		return ret;
	}

	gpio_remove_callback(config->irq.port, &drv_data->irq_callback_sem);

	LOG_DBG("Initialization successful");

	return 0;
}

#define AX5X43_INIT(inst)                                                     \
	static struct ax5x43_drv_data ax5x43_##inst##_drvdata = {};           \
	static const struct ax5x43_config ax5x43_##inst##_config = {          \
		.bus = DEVICE_DT_GET(DT_INST_BUS(inst)),                      \
		.bus_cfg = SPI_CONFIG_DT_INST(inst, AX5X43_SPI_OPERATION, 0), \
		.irq = GPIO_DT_SPEC_GET(DT_DRV_INST(inst), irq_gpios),        \
		.clock_freq = DT_INST_PROP(inst, clock_frequency),            \
		.xtaldiv = (DT_INST_PROP(inst, clock_frequency) > 24800000) ? \
		                   2 :                                        \
		                   1,                                         \
		.carrier_freq = DT_INST_PROP(inst, carrier_frequency),        \
		.clock_source = DT_ENUM_IDX(DT_DRV_INST(inst), clock_source), \
		.bitrate = DT_INST_PROP(inst, bitrate),                       \
		.channel_id = DT_INST_PROP(inst, channel_id),                 \
	};                                                                    \
	DEVICE_DT_INST_DEFINE(inst, ax5x43_init, NULL,                        \
	                      &ax5x43_##inst##_drvdata,                       \
	                      &ax5x43_##inst##_config, POST_KERNEL,           \
	                      CONFIG_AX5X43_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(AX5X43_INIT)

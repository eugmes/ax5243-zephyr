/*
 * Copyright (c) 2021 Ievgenii Meshcheriakov.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APPLICATION_SRC_AX5X43_H_
#define APPLICATION_SRC_AX5X43_H_

#include <kernel.h>
#include <drivers/spi.h>
#include <drivers/gpio.h>
#include <net/buf.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AX5X43_SPI_OPERATION                                       \
	(SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | \
	 SPI_LINES_SINGLE)

enum ax5x43_clock_source {
	AX5X43_CRYSTAL,
	AX5X43_OSCILLATOR,
};

struct ax5x43_config {
	const struct device *bus;
	struct spi_config bus_cfg;
	struct gpio_dt_spec irq;
	uint32_t clock_freq;
	uint32_t carrier_freq;
	uint16_t bitrate;
	uint8_t clock_source;
	uint8_t xtaldiv;
	uint8_t channel_id;
};

struct ax5x43_user_data {
	uint8_t channel_id : 5;
	uint8_t pad_bits : 3;
	int8_t rssi;
	int32_t rf_freq_offs : 24;
	uint32_t timer : 24;
	int64_t timestamp;
} __packed;

enum ax5x43_rx_state {
	/** Initial state. */
	AX5X43_RX_STATE_INITIAL = 0,
	/** Timer packet received. */
	AX5X43_RX_STATE_TIMER,
	/** Data packet received. */
	AX5X43_RX_STATE_DATA,
	/** Final data packet received. */
	AX5X43_RX_STATE_DATA_END,
	/** RSSI packet received. */
	AX5X43_RX_STATE_RSSI,
};

typedef void (*ax5x43_recv_callback)(const struct device *dev,
                                     struct net_buf *buf);

struct ax5x43_drv_data {
	/* Backlink to ease handling of GPIO interrupts. */
	const struct device *dev;
	struct gpio_callback irq_callback_sem;
	struct gpio_callback irq_callback_wq;
	struct k_sem irq_sem;
	struct k_work work;

	enum ax5x43_rx_state rx_state;
	struct net_buf *buf;
	ax5x43_recv_callback recv_cb;
};

/* Maximum message size in bytes, this includes CRC.
 * This should be enough for any AIS message. */
#define AX5X43_MAX_MSG_SIZE 160

void ax5x43_set_callback(const struct device *dev, ax5x43_recv_callback cb);

int ax5x43_start_rx(const struct device *dev);
int ax5x43_start_tx(const struct device *dev);
int ax5x43_send_packet(const struct device *dev, const uint8_t *buf,
                       size_t size);

#ifdef __cplusplus
}
#endif

#endif

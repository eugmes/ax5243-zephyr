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
};

struct ax5x43_drv_data {
	/* Backling to ease handling of GPIO interrupts. */
	const struct device *dev;
	struct gpio_callback irq_callback;
	struct k_sem irq_sem;
};

int ax5x43_start_rx(const struct device *dev);
int ax5x43_start_tx(const struct device *dev);
int ax5x43_read_fifo(const struct device *dev, uint8_t *buf);
int ax5x43_send_packet(const struct device *dev, const uint8_t *buf, size_t size);

#ifdef __cplusplus
}
#endif

#endif

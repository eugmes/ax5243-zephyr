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

typedef void (*ax5x43_callback)(const struct device *dev);

#define AX5X43_SPI_OPERATION                                       \
	(SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | \
	 SPI_LINES_SINGLE | SPI_HOLD_ON_CS | SPI_LOCK_ON)

struct ax5x43_config {
	const struct device *bus;
	struct spi_config bus_cfg;
	uint32_t clock_freq;
	struct gpio_dt_spec irq;
};

struct ax5x43_drv_data {
	/* Backling to ease handling of GPIO interrupts. */
	const struct device *dev;

	struct gpio_callback irq_cb;
	ax5x43_callback callback;
};

int ax5234_configure_interrupt(const struct device *dev, bool enable);
void ax5234_set_callback(const struct device *dev, ax5x43_callback callback);

#ifdef __cplusplus
}
#endif

#endif

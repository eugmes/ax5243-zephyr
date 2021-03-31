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

struct ax5x43_config {
	/* Master SPI device. */
	const struct device *spi;
	struct spi_config spi_cfg;
	uint32_t clock_freq;
	struct gpio_dt_spec cs;
	struct gpio_dt_spec irq;
};

struct ax5x43_drv_data {
	struct spi_cs_control spi_cs;

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

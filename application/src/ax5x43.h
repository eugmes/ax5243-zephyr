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

/* FIXME why is this not a standard type??? */
struct ax5x43_gpio_pin_config {
	const char *dev;
	gpio_pin_t pin;
	gpio_dt_flags_t flags;
};

struct ax5x43_config {
	const char *spi_dev_name;
	uint16_t slave;
	uint32_t freq;
	uint32_t clock_freq;
	struct ax5x43_gpio_pin_config cs;
	struct ax5x43_gpio_pin_config irq;
};

struct ax5x43_drv_data {
	/** Master SPI device */
	const struct device *spi;
	struct spi_config spi_cfg;
	struct spi_cs_control cs_ctrl;

	/* Backling to ease handling of GPIO interrupts. */
	const struct device *dev;

	const struct device *irq_dev;

	struct gpio_callback irq_cb;
	ax5x43_callback callback;
};

int ax5234_configure_interrupt(const struct device *dev, bool enable);
void ax5234_set_callback(const struct device *dev, ax5x43_callback callback);

#ifdef __cplusplus
}
#endif

#endif

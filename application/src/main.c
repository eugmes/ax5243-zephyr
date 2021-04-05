#include <zephyr.h>
#include <devicetree.h>
#include <inttypes.h>

#include "ax5x43.h"

static const struct device *get_ax5x43_device(void)
{
	const struct device *dev = DEVICE_DT_GET_ANY(onnn_ax5x43);

	if (dev == NULL) {
		printk("Error: No device found.\n");
		return NULL;
	}

	if (!device_is_ready(dev)) {
		printk("Error: Device \"%s\" is not ready.\n", dev->name);
		return NULL;
	}

	printk("Found device \"%s\".\n", dev->name);

	return dev;
}

int main(void)
{
	const struct device *dev = get_ax5x43_device();
	if (dev == NULL) {
		return 1;
	}

	int ret = ax5x43_start_rx(dev);
	if (ret < 0) {
		printk("Failed to start RX mode: %d\n", ret);
		return 2;
	}


	while (1) {
		uint8_t buf[256];

		ret = ax5x43_read_fifo(dev, buf);
		if (ret < 0) {
			printk("FIFO readout failed: %d\n", ret);
			return 3;
		}

		if (ret > 0) {
			printk("CHUNK %02"PRIX8", size = %d\n", buf[0], ret);
		}

		k_sleep(K_MSEC(1));
	}
}

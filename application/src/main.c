#include <zephyr.h>
#include <devicetree.h>
#include <inttypes.h>
#include <string.h>
#include <stdio.h>
#include <sys/byteorder.h>

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

#define NMEA_MAX_LENGTH 82
#define AIVDM_OVERHEAD (strlen("!AIVDM,0,0,0,A,,0,*00\r\n"))
// Note: this is the limit for multipart messages, it is one greater for single
// part.
#define MAX_SENTENCE_CHARS (NMEA_MAX_LENGTH - AIVDM_OVERHEAD)

static char nmea_buffer[NMEA_MAX_LENGTH + 1];
static uint8_t multipart_counter;

static char get_ascii6(const uint8_t *buf, uint16_t bit_offset)
{
	uint16_t byte_idx = bit_offset / 8;
	uint16_t bit_idx = bit_offset % 8;
	uint8_t n;

	if (bit_idx <= 2) {
		n = (buf[byte_idx] >> (2 - bit_idx)) & 0x3f;
	} else {
		n = ((buf[byte_idx] << (bit_idx - 2)) & 0x3f) |
		    (buf[byte_idx + 1] >> (10 - bit_idx));
	}

	__ASSERT_NO_MSG(n < 64);

	return (n >= 40) ? n + 56 : n + 48;
}

static void format_metadata(const struct ax5x43_user_data *data)
{
	char *p = nmea_buffer;
	*p++ = '\\';
	p += sprintf(p, "r:%" PRIu64 ",", data->timestamp);
	p += sprintf(p, "S:%" PRId8 ",", data->rssi);
	p += sprintf(p, "F:%" PRId32 ",", data->rf_freq_offs);
	p += sprintf(p, "T:%" PRIu32, data->timer);

	/* Calculate the checksum. */
	uint8_t sum = 0;

	for (const char *q = nmea_buffer + 1; q < p; q++) {
		sum ^= *q;
	}

	/* Add the checksum and the line end. */
	p += sprintf(p, "*%02X\\", sum);

	// TODO ensure that this is also true for the longest string.
	__ASSERT(PART_OF_ARRAY(nmea_buffer, p), "nmea_buffer overlow");

	// tty_write(&tty, nmea_buffer, p - nmea_buffer);
	printk("%s", nmea_buffer);
}

static void format_aivdm(uint8_t channel_id, const uint8_t *buf, size_t len)
{
	uint16_t num_chars = ceiling_fraction(len * 8, 6);

	bool multipart = num_chars > MAX_SENTENCE_CHARS + 1;
	uint8_t num_parts =
	        multipart ? ceiling_fraction(num_chars, MAX_SENTENCE_CHARS) : 1;
	//LOG_DBG("len: %zu, chars: %u, parts: %u", len, num_chars, num_parts);

	char channel = 'A' + channel_id;
	uint16_t bit_offset = 0;
	uint16_t remaining_chars = num_chars;

	for (uint8_t part = 1; part <= num_parts; part++) {
		char *p = nmea_buffer;

		/* Put header. */
		if (multipart) {
			p += sprintf(p, "!AIVDM,%u,%u,%u,%c,", num_parts, part,
			             multipart_counter, channel);
		} else {
			p += sprintf(p, "!AIVDM,1,1,,%c,", channel);
		}

		/* Put the data. */
		uint16_t part_chars = remaining_chars;
		if (multipart && remaining_chars > MAX_SENTENCE_CHARS) {
			part_chars = MAX_SENTENCE_CHARS;
		}

		//LOG_DBG("part: %u, chars: %u", part, part_chars);

		for (int i = 0; i < part_chars; i++) {
			*p++ = get_ascii6(buf, bit_offset);
			bit_offset += 6;
		}

		remaining_chars -= part_chars;

		/* Add pad info */
		uint8_t pad = 0;

		if (part == num_parts) {
			__ASSERT_NO_MSG(num_chars * 6 >= len * 8);
			pad = num_chars * 6 - len * 8;
		}

		p += sprintf(p, ",%u", pad);

		/* Calculate the checksum. */
		uint8_t sum = 0;

		for (const char *q = nmea_buffer + 1; q < p; q++) {
			sum ^= *q;
		}

		/* Add the checksum and the line end. */
		p += sprintf(p, "*%02X\r\n", sum);

		__ASSERT(PART_OF_ARRAY(nmea_buffer, p), "nmea_buffer overlow");

		// tty_write(&tty, nmea_buffer, p - nmea_buffer);
		printk("%s", nmea_buffer);
	}

	if (multipart) {
		multipart_counter++;
		if (multipart_counter > 9) {
			multipart_counter = 0;
		}
	}
}

K_FIFO_DEFINE(rx_fifo);

static void recv_cb(const struct device *dev, struct net_buf *buf)
{
	ARG_UNUSED(dev);
	net_buf_put(&rx_fifo, buf);
}

static int main_rx(const struct device *dev)
{
	ax5x43_set_callback(dev, recv_cb);

	int ret = ax5x43_start_rx(dev);
	if (ret < 0) {
		printk("Failed to start RX mode: %d\n", ret);
		return 2;
	}

	while (1) {
		struct net_buf *buf = net_buf_get(&rx_fifo, K_FOREVER);
		const struct ax5x43_user_data *meta = net_buf_user_data(buf);
		format_metadata(meta);
		format_aivdm(meta->channel_id, buf->data, buf->len);
		net_buf_unref(buf);
	}
}

static int main_tx(const struct device *dev)
{
	int ret = ax5x43_start_tx(dev);
	if (ret < 0) {
		printk("Failed to start TX mode: %d\n", ret);
		return 2;
	}

	k_sleep(K_MSEC(1000));

	uint8_t seq_no = 0;

	while (1) {
		// Single Slot Binary Message
		uint8_t data[] = { 25 << 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

		uint16_t timestamp = k_uptime_get();
		data[13] = seq_no++;
		sys_put_be64(timestamp, data + 5);

		ret = ax5x43_send_packet(dev, data, sizeof(data));
		if (ret < 0) {
			printk("Failed to send packet: %d\n", ret);
			return 3;
		}

		printk(".");

		k_sleep(K_MSEC(1000));
	}
}

int main(void)
{
	const struct device *dev = get_ax5x43_device();
	if (dev == NULL) {
		return 1;
	}

#if 1
	return main_rx(dev);
#else
	return main_tx(dev);
#endif
}

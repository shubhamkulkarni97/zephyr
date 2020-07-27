/* Stellaris Ethernet Controller
 *
 * Copyright (c) 2018 Zilogic Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT espressif_esp32_ethernet

#define LOG_MODULE_NAME eth_esp32
#define LOG_LEVEL CONFIG_ETHERNET_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <net/ethernet.h>
#include <net/net_pkt.h>
#include <net/net_if.h>
#include <device.h>
#include <soc.h>
#include <ethernet/eth_stats.h>
#include "eth_stellaris_priv.h"
#include "esp_wifi_internal.h"

static int eth_esp32_send(struct device *dev, struct net_pkt *pkt)
{
	struct eth_esp32_runtime *dev_data = DEV_DATA(dev);
	struct net_buf *frag;
	uint16_t offset = 0;


	/* Frame transmission
	 *
	 * First two bytes is the length of the frame, exclusive of
	 * the header length.
	 */
	uint8_t *tx_buffer = (uint8_t *) k_malloc(net_pkt_get_len(pkt));
	if (tx_buffer == NULL) {
		return -EIO;
	}

	/* Send the payload */
	for (frag = pkt->frags; frag; frag = frag->frags) {
		memcpy((tx_buffer + offset), frag->data, frag->len);
		offset += frag->len;
	}

	esp_wifi_internal_tx(ESP_IF_WIFI_STA, (void *) tx_buffer, net_pkt_get_len(pkt));

	LOG_DBG("pkt sent %p len %d", pkt, net_pkt_get_len(pkt));
	k_free(tx_buffer);

	return 0;
}

static void eth_esp32_rx_error(struct net_if *iface)
{
	struct device *dev = net_if_get_device(iface);
	uint32_t val;

	eth_stats_update_errors_rx(iface);

#if 0
	/* Clear the rx_frame buffer,
	 * otherwise it could lead to underflow errors
	 */
	sys_write32(0x0, REG_MACRCTL);
	sys_write32(BIT_MACRCTL_RSTFIFO, REG_MACRCTL);
	val = BIT_MACRCTL_BADCRC | BIT_MACRCTL_RXEN;
	sys_write32(val, REG_MACRCTL);
#endif
}

static struct net_pkt *eth_esp32_rx_pkt(struct device *dev,
					    struct net_if *iface)
{
#if 0
	int frame_len, bytes_left;
	struct net_pkt *pkt;
	uint32_t reg_val;
	uint16_t count;
	uint8_t *data;

	/*
	 * The Ethernet frame received from the hardware has the
	 * following format. The first two bytes contains the ethernet
	 * frame length, followed by the actual ethernet frame.
	 *
	 * +---------+---- ... -------+
	 * | Length  | Ethernet Frame |
	 * +---------+---- ... -------+
	 */

	/*
	 * The first word contains the frame length and a portion of
	 * the ethernet frame. Extract the frame length.
	 */
	reg_val = sys_read32(REG_MACDATA);
	frame_len = reg_val & 0x0000ffff;

	pkt = net_pkt_rx_alloc_with_buffer(iface, frame_len,
					   AF_UNSPEC, 0, K_NO_WAIT);
	if (!pkt) {
		return NULL;
	}

	/*
	 * The remaining 2 bytes, in the first word is appended to the
	 * ethernet frame.
	 */
	count = 2U;
	data = (uint8_t *)&reg_val + 2;
	if (net_pkt_write(pkt, data, count)) {
		goto error;
	}

	/* A word has been read already, thus minus 4 bytes to be read. */
	bytes_left = frame_len - 4;

	/* Read the rest of words, minus the partial word and FCS byte. */
	for (; bytes_left > 7; bytes_left -= 4) {
		reg_val = sys_read32(REG_MACDATA);
		count = 4U;
		data = (uint8_t *)&reg_val;
		if (net_pkt_write(pkt, data, count)) {
			goto error;
		}
	}

	/* Handle the last partial word and discard the 4 Byte FCS. */
	while (bytes_left > 0) {
		/* Read the partial word. */
		reg_val = sys_read32(REG_MACDATA);

		/* Discard the last FCS word. */
		if (bytes_left <= 4) {
			bytes_left = 0;
			break;
		}

		count = bytes_left - 4;
		data = (uint8_t *)&reg_val;
		if (net_pkt_write(pkt, data, count)) {
			goto error;
		}

		bytes_left -= 4;
	}

	return pkt;

error:
	net_pkt_unref(pkt);

#endif
	return NULL;
}

static void eth_esp32_rx(struct device *dev)
{
	struct eth_esp32_runtime *dev_data = DEV_DATA(dev);
	struct net_if *iface = dev_data->iface;
	struct net_pkt *pkt;

	pkt = eth_esp32_rx_pkt(dev, iface);
	if (!pkt) {
		LOG_ERR("Failed to read data");
		goto err_mem;
	}

	if (net_recv_data(iface, pkt) < 0) {
		LOG_ERR("Failed to place frame in RX Queue");
		goto pkt_unref;
	}

	return;

pkt_unref:
	net_pkt_unref(pkt);

err_mem:
	eth_esp32_rx_error(iface);
}

static void eth_esp32_init(struct net_if *iface)
{
	struct device *dev = net_if_get_device(iface);
	const struct eth_esp32_config *dev_conf = DEV_CFG(dev);
	struct eth_esp32_runtime *dev_data = DEV_DATA(dev);

	dev_data->iface = iface;

	/* Assign link local address. */
	net_if_set_link_addr(iface,
			     dev_data->mac_addr, 6, NET_LINK_ETHERNET);

	ethernet_init(iface);

	/* Initialize Interrupts. */
	dev_conf->config_func(dev);
}

#if defined(CONFIG_NET_STATISTICS_ETHERNET)
static struct net_stats_eth *eth_esp32_stats(struct device *dev)
{
	return &(DEV_DATA(dev)->stats);
}
#endif

static int eth_esp32_dev_init(struct device *dev)
{
#if 0
	uint32_t value;

	/* Assign MAC address to Hardware */
	eth_esp32_assign_mac(dev);

	/* Program MCRCTL to clear RXFIFO */
	value = BIT_MACRCTL_RSTFIFO;
	sys_write32(value, REG_MACRCTL);

	/* Enable transmitter */
	value = BIT_MACTCTL_DUPLEX | BIT_MACTCTL_CRC |
		BIT_MACTCTL_PADEN | BIT_MACTCTL_TXEN;
	sys_write32(value, REG_MACTCTL);

	/* Enable Receiver */
	value = BIT_MACRCTL_BADCRC | BIT_MACRCTL_RXEN;
	sys_write32(value, REG_MACRCTL);
#endif

	return 0;
}

DEVICE_DECLARE(eth_esp32);

static void eth_esp32_irq_config(struct device *dev)
{
#if 0
	/* Enable Interrupt. */
	IRQ_CONNECT(DT_INST_IRQN(0),
		    DT_INST_IRQ(0, priority),
		    eth_esp32_isr, DEVICE_GET(eth_esp32), 0);
	irq_enable(DT_INST_IRQN(0));
#endif
}

struct eth_esp32_config eth_cfg = {
	.mac_base = 0 /*DT_INST_REG_ADDR(0)*/,
	.config_func = eth_esp32_irq_config,
};

struct eth_esp32_runtime eth_data = {
	.mac_addr = {0x30, 0xae, 0xa4, 0xef, 0x48, 0x58} /*DT_INST_PROP(0, local_mac_address)*/,
	.tx_err = false,
	.tx_word = 0,
	.tx_pos = 0,
};

static const struct ethernet_api eth_esp32_apis = {
	.iface_api.init	= eth_esp32_init,
	.send =  eth_esp32_send,
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	.get_stats = eth_esp32_stats,
#endif
};

NET_DEVICE_INIT(eth_esp32, "esp_eth" /*DT_INST_LABEL(0)*/,
		eth_esp32_dev_init, device_pm_control_nop,
		&eth_data, &eth_cfg, CONFIG_ETH_INIT_PRIORITY,
		&eth_esp32_apis, ETHERNET_L2,
		NET_L2_GET_CTX_TYPE(ETHERNET_L2), NET_ETH_MTU);

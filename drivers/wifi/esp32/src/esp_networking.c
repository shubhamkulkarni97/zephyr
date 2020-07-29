/* Espressif WiFi Controller
 *
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
#include "esp_networking_priv.h"
#include "esp_wifi_internal.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "wifi_system.h"

#define DEV_DATA(dev) \
	((struct eth_esp32_runtime *)(dev)->driver_data)
#define DEV_CFG(dev) \
	((const struct eth_esp32_config *const)(dev)->config_info)

struct eth_esp32_runtime {
	struct net_if *iface;
	uint8_t mac_addr[6];
	struct k_sem tx_sem;
	bool tx_err;
	uint32_t tx_word;
	int tx_pos;
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	struct net_stats_eth stats;
#endif
};

typedef void (*eth_esp32_config_irq_t)(struct device *dev);

struct eth_esp32_config {
	uint32_t mac_base;
	uint32_t sys_ctrl_base;
	uint32_t irq_num;
};

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

esp_err_t eth_esp32_rx(void *buffer, uint16_t len, void *eb)
{
    struct net_pkt *pkt;
    struct net_if *iface;
    iface = net_if_get_default();
    pkt = net_pkt_rx_alloc_with_buffer(iface, len,
					   AF_UNSPEC, 0, K_NO_WAIT);
    net_pkt_write(pkt, buffer, len);
    if (net_recv_data(iface, pkt) < 0) {
        ets_printf("Failed to push data\n");
    }
    esp_wifi_internal_free_rx_buffer(eb);
    return ESP_OK;
}

void esp_wifi_register_rx_callback(void)
{
	esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, (wifi_rxcb_t)eth_esp32_rx);
}

static void eth_esp32_init(struct net_if *iface)
{
	struct device *dev = net_if_get_device(iface);
	struct eth_esp32_runtime *dev_data = DEV_DATA(dev);

	dev_data->iface = iface;

	esp_read_mac(dev_data->mac_addr, ESP_MAC_WIFI_STA);

	/* Assign link local address. */
	net_if_set_link_addr(iface,
			     dev_data->mac_addr, 6, NET_LINK_ETHERNET);

	ethernet_init(iface);
}

#if defined(CONFIG_NET_STATISTICS_ETHERNET)
static struct net_stats_eth *eth_esp32_stats(struct device *dev)
{
	return &(DEV_DATA(dev)->stats);
}
#endif

static int eth_esp32_dev_init(struct device *dev)
{
	esp_timer_init();
	esp_event_init();
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT() ;
    esp_err_t ret = esp_wifi_init(&config);
    ret |= esp_wifi_set_mode(WIFI_MODE_STA);
    ret |= esp_wifi_start();
	return ret;
}

DEVICE_DECLARE(eth_esp32);

struct eth_esp32_runtime eth_data = {
	.mac_addr = {0},
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
		&eth_data, NULL, CONFIG_ETH_INIT_PRIORITY,
		&eth_esp32_apis, ETHERNET_L2,
		NET_L2_GET_CTX_TYPE(ETHERNET_L2), NET_ETH_MTU);

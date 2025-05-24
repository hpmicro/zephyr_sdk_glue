/*
 * Copyright (c) 2025 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#define DT_DRV_COMPAT hpmicro_hpm_eth
#define LOG_LEVEL CONFIG_ETHERNET_LOG_LEVEL

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ethernet_hpm_eth);

#include <errno.h>
#include <stdbool.h>
#include <soc.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/crc.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/phy.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/net/lldp.h>
#include <zephyr/drivers/hwinfo.h>
#include <ethernet/eth.h>
#include <ethernet/eth_stats.h>

#include "hpm_enet_drv.h"
#include "hpm_clock_drv.h"

#if defined(CONFIG_ETH_PHY) && CONFIG_ETH_PHY
#if defined(CONFIG_ETH_PHY_RTL8211) && CONFIG_ETH_PHY_RTL8211
#define __USE_RTL8211 1
#elif defined(CONFIG_ETH_PHY_RTL8201) && CONFIG_ETH_PHY_RTL8201
#define __USE_RTL8201 1
#endif
#include "hpm_enet_phy_common.h"
#endif

#if defined(CONFIG_ETH_HPM_RGMII) && CONFIG_ETH_HPM_RGMII
#define MEDIA_ITF_TYPE       enet_inf_rgmii
#elif defined(CONFIG_ETH_HPM_RMII) && CONFIG_ETH_HPM_RMII
#define MEDIA_ITF_TYPE       enet_inf_rmii
#endif

#ifndef ETH_HPM_BUF_TIMEOUT
#define ETH_HPM_BUF_TIMEOUT	 K_MSEC(20)
#endif

#ifndef ETH_HPM_TX_TIMEOUT
#define ETH_HPM_TX_TIMEOUT	 K_MSEC(20)
#endif

struct hpm_eth_config {
	ENET_Type *base;
	uint32_t clock_name;
	uint32_t clock_src;
	uint32_t clock_div;
	const struct gpio_dt_spec reset_phy_gpio;
	void (*irq_config_func)(const struct device *dev);
	const struct pinctrl_dev_config *pincfg;
};

struct hpm_eth_data {
	struct net_if *iface;
	uint8_t mac_addr[6];
	enet_desc_t desc;
	enet_inf_type_t  media_itf;
	bool link_up;
	struct k_mutex tx_mutex;
	struct k_sem rx_sem;
	struct k_sem tx_sem;
	struct k_thread rx_thread;
	K_KERNEL_STACK_MEMBER(rx_thread_stack, CONFIG_ETH_HPM_RX_THREAD_STACK_SIZE);
};

__attribute__((section(".nocache"))) __attribute__((aligned(ENET_SOC_DESC_ADDR_ALIGNMENT)))
enet_rx_desc_t dma_rx_desc_tab[CONFIG_ETH_HPM_RX_BUFF_COUNT];

__attribute__((section(".nocache")))__attribute__((aligned(ENET_SOC_DESC_ADDR_ALIGNMENT)))
enet_tx_desc_t dma_tx_desc_tab[CONFIG_ETH_HPM_TX_BUFF_COUNT];

__attribute__((section(".nocache")))__attribute__((aligned(ENET_SOC_BUFF_ADDR_ALIGNMENT)))
uint8_t rx_buff[CONFIG_ETH_HPM_RX_BUFF_COUNT][CONFIG_ETH_HPM_RX_BUFF_SIZE];

__attribute__((section(".nocache"))) __attribute__((aligned(ENET_SOC_BUFF_ADDR_ALIGNMENT)))
uint8_t tx_buff[CONFIG_ETH_HPM_TX_BUFF_COUNT][CONFIG_ETH_HPM_TX_BUFF_SIZE];

static struct net_if *get_iface(struct hpm_eth_data *data)
{
	return data->iface;
}

static int eth_tx(const struct device *dev, struct net_pkt *pkt)
{
	struct hpm_eth_data *data = dev->data;
	const struct hpm_eth_config *cfg = dev->config;
	int res;
	size_t pkt_len;
	uint16_t frame_length;
	enet_tx_desc_t *dma_tx_desc = data->desc.tx_desc_list_cur;

	__ASSERT_NO_MSG(pkt != NULL);
	__ASSERT_NO_MSG(pkt->frags != NULL);
	__ASSERT_NO_MSG(dev != NULL);
	__ASSERT_NO_MSG(data != NULL);

	k_mutex_lock(&data->tx_mutex, K_FOREVER);
	k_sem_reset(&data->tx_sem);

	pkt_len = net_pkt_get_len(pkt);
	if (pkt_len > CONFIG_ETH_HPM_TX_BUFF_SIZE) {
		LOG_ERR("PKT too big");
		res = -EPERM;
		goto error;
	}

	if (dma_tx_desc->tdes0_bm.own != 0) {
		LOG_ERR("Failed to get tx buffer");
		res = -EBUSY;
		goto error;
	}

	res = net_pkt_read(pkt, (uint8_t *)dma_tx_desc->tdes2_bm.buffer1, pkt_len);
	if (res) {
		LOG_ERR("Failed to copy packet to tx buffer (%d)", res);
		res = -ENOBUFS;
		goto error;
	}

	frame_length = (uint16_t)pkt_len + 4;

	res = enet_prepare_tx_desc(cfg->base, &data->desc.tx_desc_list_cur, &data->desc.tx_control_config, frame_length, data->desc.tx_buff_cfg.size);
	if (res != ENET_SUCCESS) {
		LOG_ERR("Failed to tx frame (%d)", res);
		res = -EIO;
		goto error;
	}

	/* Wait for the transmission to complete */
	if (k_sem_take(&data->tx_sem, ETH_HPM_TX_TIMEOUT) != 0) {
		LOG_ERR("Timeout transmitting frame");
		res = -EIO;
		goto error;
	}

error:
	k_mutex_unlock(&data->tx_mutex);

	return res;
}

static struct net_pkt *eth_rx(const struct device *dev)
{
	const struct hpm_eth_config *cfg;
	struct hpm_eth_data *data;
	struct net_pkt *pkt = NULL;
	enet_frame_t frame = {0, 0, 0};
    enet_rx_desc_t *dma_rx_desc;

	__ASSERT_NO_MSG(dev != NULL);

	data = dev->data;
	cfg = dev->config;

	__ASSERT_NO_MSG(data != NULL);
	__ASSERT_NO_MSG(cfg != NULL);

	frame = enet_get_received_frame_interrupt(&data->desc.rx_desc_list_cur, &data->desc.rx_frame_info, CONFIG_ETH_HPM_RX_BUFF_COUNT);

	if (frame.length == 0) {
		return NULL;
	}

	pkt = net_pkt_rx_alloc_with_buffer(get_iface(data), frame.length, AF_UNSPEC, 0, ETH_HPM_BUF_TIMEOUT);
	if (!pkt) {
		LOG_ERR("Failed to obtain RX buffer");
		goto release_desc;
	}

	if (net_pkt_write(pkt, (uint8_t *)frame.buffer, frame.length)) {
		LOG_ERR("Failed to append RX buffer to context buffer");
		net_pkt_unref(pkt);
		pkt = NULL;
		goto release_desc;
	}

release_desc:
	/* Release descriptors to DMA */
	dma_rx_desc = frame.rx_desc;

	/* Set Own bit in Rx descriptors: gives the buffers back to DMA */
	for (int i = 0; i < data->desc.rx_frame_info.seg_count; i++) {
		dma_rx_desc->rdes0_bm.own = 1;
		dma_rx_desc = (enet_rx_desc_t *)(dma_rx_desc->rdes3_bm.next_desc);
	}

	/* Clear Segment_Count */
	data->desc.rx_frame_info.seg_count = 0;

	/* Resume Rx Process */
	enet_rx_resume(cfg->base);

	return pkt;
}

static void rx_thread(void *arg1, void *unused1, void *unused2)
{
	const struct device *dev;
	struct hpm_eth_data *data;
	struct net_if *iface;
	struct net_pkt *pkt;
	int res;

	__ASSERT_NO_MSG(arg1 != NULL);
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);

	dev = (const struct device *)arg1;
	data = dev->data;

	__ASSERT_NO_MSG(dev != NULL);
	__ASSERT_NO_MSG(data != NULL);

	while (1) {
		res = k_sem_take(&data->rx_sem, K_FOREVER);
		__ASSERT_NO_MSG(res == 0);

		if (res == 0) {
			if (data->link_up != true) {
				data->link_up = true;
				net_eth_carrier_on(get_iface(data));
			}
		}

		while ((pkt = eth_rx(dev)) != NULL) {
			iface = net_pkt_iface(pkt);
			res = net_recv_data(iface, pkt);
			if (res < 0) {
				LOG_ERR("Failed to enqueue frame into RX queue: %d", res);
				net_pkt_unref(pkt);
			}
		}
	}
}

static void eth_rx_callback(const struct device *dev)
{
	struct hpm_eth_data *data = dev->data;

	__ASSERT_NO_MSG(data != NULL);
	k_sem_give(&data->rx_sem);
}

static void eth_tx_callback(const struct device *dev)
{
	struct hpm_eth_data *data = dev->data;

	__ASSERT_NO_MSG(data != NULL);
	k_sem_give(&data->tx_sem);
}

__attribute__((section(".isr"))) static void hpm_eth_isr(const struct device *dev)
{
	const struct hpm_eth_config *cfg = dev->config;

	uint32_t status;
    uint32_t rxgbfrmis;
    uint32_t intr_status;

	/* Process pending Ethernet interrupts */
    status = cfg->base->DMA_STATUS;
    rxgbfrmis = cfg->base->MMC_INTR_RX;
    intr_status = cfg->base->INTR_STATUS;

	/* Check if there are pending "normal" interrupts */

    if (ENET_DMA_STATUS_GLPII_GET(status)) {
	    /* read LPI_CSR to clear interrupt status */
	    cfg->base->LPI_CSR;
    }

	if (ENET_INTR_STATUS_RGSMIIIS_GET(intr_status)) {
		/* read XMII_CSR to clear interrupt status */
		cfg->base->XMII_CSR;
	}

	if (ENET_DMA_STATUS_TI_GET(status)) {
		cfg->base->DMA_STATUS |= ENET_DMA_STATUS_TI_MASK;
		eth_tx_callback(dev);
	}

	if (ENET_DMA_STATUS_RI_GET(status)) {
		cfg->base->DMA_STATUS |= ENET_DMA_STATUS_RI_MASK;
		eth_rx_callback(dev);
	}

	if (ENET_MMC_INTR_RX_RXCTRLFIS_GET(rxgbfrmis)) {
		cfg->base->RXFRAMECOUNT_GB;
	}
}

static void generate_mac(uint8_t *mac_addr)
{
	mac_addr[0] = CONFIG_ETH_HPM_MAC_ADDR0;
	mac_addr[1] = CONFIG_ETH_HPM_MAC_ADDR1;
	mac_addr[2] = CONFIG_ETH_HPM_MAC_ADDR2;
	mac_addr[3] = CONFIG_ETH_HPM_MAC_ADDR3;
	mac_addr[4] = CONFIG_ETH_HPM_MAC_ADDR4;
	mac_addr[5] = CONFIG_ETH_HPM_MAC_ADDR5;
}

static int eth_reset_phy(const struct device *dev)
{
	const struct hpm_eth_config *cfg = dev->config;
	int res;

	if (gpio_is_ready_dt(&cfg->reset_phy_gpio)) {
        gpio_pin_configure_dt(&cfg->reset_phy_gpio, GPIO_OUTPUT_ACTIVE);
		k_usleep(1000);
		gpio_pin_configure_dt(&cfg->reset_phy_gpio, GPIO_OUTPUT_INACTIVE);
		res = 0;
    } else {
        LOG_ERR("no reset phy gpio");
		res = -1;
    }

	return res;
}

static int eth_init_controller(const struct device *dev)
{
	const struct hpm_eth_config *cfg = dev->config;
	struct hpm_eth_data *data = dev->data;
	enet_int_config_t int_config = {.int_enable = 0, .int_mask = 0};
    enet_mac_config_t mac_config;
	enet_tx_control_config_t tx_control_config;

#if defined(CONFIG_ETH_PHY)	&& CONFIG_ETH_PHY
#if defined(CONFIG_ETH_HPM_RGMII) && CONFIG_ETH_HPM_RGMII
    rtl8211_config_t phy_config;
#elif defined(CONFIG_ETH_HPM_RMII) && CONFIG_ETH_HPM_RMII
    rtl8201_config_t phy_config;
#endif
#endif

    /* Initialize td, rd and the corresponding buffers */
    memset((uint8_t *)dma_tx_desc_tab, 0x00, sizeof(dma_tx_desc_tab));
    memset((uint8_t *)dma_rx_desc_tab, 0x00, sizeof(dma_rx_desc_tab));
    memset((uint8_t *)rx_buff, 0x00, sizeof(rx_buff));
    memset((uint8_t *)tx_buff, 0x00, sizeof(tx_buff));

    data->desc.tx_desc_list_head = (enet_tx_desc_t *)core_local_mem_to_sys_address(0, (uint32_t)dma_tx_desc_tab);
    data->desc.rx_desc_list_head = (enet_rx_desc_t *)core_local_mem_to_sys_address(0, (uint32_t)dma_rx_desc_tab);

    data->desc.tx_buff_cfg.buffer = core_local_mem_to_sys_address(0, (uint32_t)tx_buff);
    data->desc.tx_buff_cfg.count = CONFIG_ETH_HPM_TX_BUFF_COUNT;
    data->desc.tx_buff_cfg.size = CONFIG_ETH_HPM_TX_BUFF_SIZE;

    data->desc.rx_buff_cfg.buffer = core_local_mem_to_sys_address(0, (uint32_t)rx_buff);
    data->desc.rx_buff_cfg.count = CONFIG_ETH_HPM_RX_BUFF_COUNT;
    data->desc.rx_buff_cfg.size = CONFIG_ETH_HPM_RX_BUFF_SIZE;

    /*Get a default control config for tx descriptor */
    enet_get_default_tx_control_config(cfg->base, &tx_control_config);

	/* Set the control config for tx descriptor */
    tx_control_config.cic = 0;
    tx_control_config.enable_ioc = true;
    memcpy(&data->desc.tx_control_config, &tx_control_config, sizeof(enet_tx_control_config_t));

    /* Set MAC0 address */
    mac_config.mac_addr_high[0] = data->mac_addr[5] << 8 |
                                  data->mac_addr[4];
    mac_config.mac_addr_low[0]  = data->mac_addr[3] << 24 |
                                  data->mac_addr[2] << 16 |
                                  data->mac_addr[1] << 8 |
                                  data->mac_addr[0];

	mac_config.valid_max_count  = 1;

    /* Set DMA PBL */
    mac_config.dma_pbl = enet_pbl_32;

    /* Set SARC */
    mac_config.sarc = enet_sarc_replace_mac0;

    /* Set the interrupt enable mask */
    int_config.int_enable = enet_normal_int_sum_en    /* Enable normal interrupt summary */
                          | enet_receive_int_en       /* Enable receive interrupt */
                          | enet_transmit_int_en;     /* ENable transmit interrupt */

    int_config.int_mask = enet_rgsmii_int_mask; /* Disable RGSMII interrupt */

    /* Initialize enet controller */
    enet_controller_init(cfg->base, data->media_itf, &data->desc, &mac_config, &int_config);

    /* Disable LPI interrupt */
    enet_disable_lpi_interrupt(cfg->base);

	/* Enable IRQ */
	__ASSERT_NO_MSG(cfg->irq_config_func != NULL);
	cfg->irq_config_func(dev);

	/* Initialize phy */
#if defined(CONFIG_ETH_PHY) && CONFIG_ETH_PHY
#if defined(CONFIG_ETH_HPM_RGMII) && CONFIG_ETH_HPM_RGMII
	rtl8211_reset(cfg->base);
	rtl8211_basic_mode_default_config(cfg->base, &phy_config);
	if (rtl8211_basic_mode_init(cfg->base, &phy_config) == true) {
#elif defined(CONFIG_ETH_HPM_RMII) && CONFIG_ETH_HPM_RMII
	    rtl8201_reset(cfg->base);
	    rtl8201_basic_mode_default_config(cfg->base, &phy_config);
		phy_config.txc_input = CONFIG_ETH_RMII_REFCLK ? true : false;
	if (rtl8201_basic_mode_init(cfg->base, &phy_config) == true) {
#endif
		LOG_INF("Enet phy init passed !\n");
	    return status_success;
	} else {
	    LOG_INF("Enet phy init failed !\n");
		return status_fail;
	}
#endif
	return status_success;
}

static int hpm_eth_init(const struct device *dev)
{
	const struct hpm_eth_config *cfg;
	struct hpm_eth_data *data;
	int res;

	__ASSERT_NO_MSG(dev != NULL);

	data = dev->data;
	cfg = dev->config;

	__ASSERT_NO_MSG(data != NULL);
	__ASSERT_NO_MSG(cfg != NULL);

	/* Configure Pinmux */
	res = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (res < 0) {
		LOG_ERR("Could not configure ethernet pins");
		return res;
	}

	/* Reset PHY */
	eth_reset_phy(dev);

	/* Configure Clock */
	clock_set_source_divider(cfg->clock_name, cfg->clock_src, cfg->clock_div);
	clock_add_to_group(cfg->clock_name, 0);

	clock_add_to_group(clock_eth0, 0);

#if defined(CONFIG_ETH_HPM_RGMII) && CONFIG_ETH_HPM_RGMII
	/* Set RGMII clock delay */
	enet_rgmii_set_clock_delay(cfg->base, CONFIG_ETH_HPM_TX_DELAY, CONFIG_ETH_HPM_RX_DELAY);
#elif defined(CONFIG_ETH_HPM_RMII) && CONFIG_ETH_HPM_RMII
	/* Set RMII reference clock */
	enet_rmii_enable_clock(cfg->base, CONFIG_ETH_RMII_REFCLK);
	LOG_INF("Reference Clock: %s\n", CONFIG_ETH_RMII_REFCLK ? "Internal Clock" : "External Clock");
#endif

	/* Get MAC Address */
	generate_mac(data->mac_addr);
	data->media_itf = MEDIA_ITF_TYPE;

	/* Initialize Controller */
	eth_init_controller(dev);

	/* Initialize semaphores */
	k_mutex_init(&data->tx_mutex);
	k_sem_init(&data->rx_sem, 0, 1);
	k_sem_init(&data->tx_sem, 0, 1);

	/* Start interruption-poll thread */
	k_thread_create(&data->rx_thread, data->rx_thread_stack,
		K_KERNEL_STACK_SIZEOF(data->rx_thread_stack),
		rx_thread, (void *) dev, NULL, NULL,
		K_PRIO_COOP(CONFIG_ETH_HPM_RX_THREAD_PRIO),
		0, K_NO_WAIT);

	k_thread_name_set(&data->rx_thread, "hpm_eth");

	return 0;
}

static void eth_iface_init(struct net_if *iface)
{
	const struct device *dev;
	struct hpm_eth_data *data;

	__ASSERT_NO_MSG(iface != NULL);

	dev = net_if_get_device(iface);
	__ASSERT_NO_MSG(dev != NULL);

	data = dev->data;
	__ASSERT_NO_MSG(data != NULL);

	if (data->iface == NULL) {
		data->iface = iface;
	}

	/* Register Ethernet MAC Address with the upper layer */
	net_if_set_link_addr(iface, data->mac_addr,
			     sizeof(data->mac_addr),
			     NET_LINK_ETHERNET);

	LOG_DBG("MAC %02x:%02x:%02x:%02x:%02x:%02x",
			data->mac_addr[0], data->mac_addr[1],
			data->mac_addr[2], data->mac_addr[3],
			data->mac_addr[4], data->mac_addr[5]);

	ethernet_init(iface);

	net_if_carrier_on(iface);
}

static enum ethernet_hw_caps eth_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);

	return ETHERNET_LINK_10BASE_T | ETHERNET_LINK_100BASE_T
#if defined(CONFIG_ETH_HPM_RGMII)
		| ETHERNET_LINK_1000BASE_T
#endif
#if defined(CONFIG_ETH_HPM_PROMISCUOUS_MODE)
        | ETHERNET_PROMISC_MODE
#endif
#if defined(CONFIG_ETH_HPM_HW_CHECKSUM)
		| ETHERNET_HW_RX_CHKSUM_OFFLOAD
		| ETHERNET_HW_TX_CHKSUM_OFFLOAD
#endif
		;
}

static int eth_set_config(const struct device *dev,
				    enum ethernet_config_type type,
				    const struct ethernet_config *config)
{
	struct hpm_eth_data *data = dev->data;
	const struct hpm_eth_config *cfg = dev->config;
    int res;

	switch (type) {
	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS:
		memcpy(data->mac_addr, config->mac_address.addr, 6);

		cfg->base->MAC_ADDR_0_HIGH = (data->mac_addr[5] << 8) |
		                              data->mac_addr[4];
		cfg->base->MAC_ADDR_0_LOW = (data->mac_addr[3] << 24) |
									(data->mac_addr[2] << 16) |
									(data->mac_addr[1] << 8) |
									 data->mac_addr[0];

		net_if_set_link_addr(data->iface, data->mac_addr,
				     sizeof(data->mac_addr),
				     NET_LINK_ETHERNET);

		LOG_INF("MAC is updated to: %02x:%02x:%02x:%02x:%02x:%02x",
				data->mac_addr[0], data->mac_addr[1], data->mac_addr[2],
				data->mac_addr[3], data->mac_addr[4], data->mac_addr[5]);
		res = 0;
		break;
	case ETHERNET_CONFIG_TYPE_PROMISC_MODE:
#if defined(CONFIG_ETH_HPM_PROMISCUOUS_MODE)
		if (config->promisc_mode) {
			cfg->base->MACFF |= ENET_MACFF_PR_MASK;
		} else {
			cfg->base->MACFF &= ~ENET_MACFF_PR_MASK;
		}
		res = 0;
#endif /* CONFIG_ETH_HPM_PROMISCUOUS_MODE */
		break;
	default:
		break;
	}

	return res;
}

static const struct ethernet_api eth_api = {
	.iface_api.init = eth_iface_init,
	.get_capabilities = eth_get_capabilities,
	.set_config = eth_set_config,
	.send = eth_tx
};

#define ETH_HPM_INIT(n)						\
static void hpm_eth_config_func_##n(const struct device *dev); \
								\
PINCTRL_DT_INST_DEFINE(n);					\
								\
static const struct hpm_eth_config hpm_eth_config_##n = {	\
	.base = (ENET_Type *)DT_INST_REG_ADDR(n),		\
	.clock_name = DT_INST_CLOCKS_CELL_BY_IDX(n, DT_INST_CLOCKS_HAS_IDX(n, 1), name),\
	.clock_src = DT_INST_CLOCKS_CELL_BY_IDX(n, DT_INST_CLOCKS_HAS_IDX(n, 1), src),\
	.clock_div = DT_INST_CLOCKS_CELL_BY_IDX(n, DT_INST_CLOCKS_HAS_IDX(n, 1), div),\
	.irq_config_func = hpm_eth_config_func_##n,		\
	.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n), \
    .reset_phy_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}), \
};								\
								\
static struct hpm_eth_data hpm_eth_data_##n;								\
								\
ETH_NET_DEVICE_DT_INST_DEFINE(n,						\
	hpm_eth_init,					\
	NULL,							\
	&hpm_eth_data_##n,					\
	&hpm_eth_config_##n,				\
	CONFIG_ETH_INIT_PRIORITY,				\
	&eth_api,						\
	NET_ETH_MTU); \
								\
static void hpm_eth_config_func_##n(const struct device *dev) \
{								\
	IRQ_CONNECT(DT_INST_IRQN(n),				\
	DT_INST_IRQ(n, priority), hpm_eth_isr,	\
	DEVICE_DT_INST_GET(n), 0);			\
								\
	irq_enable(DT_INST_IRQN(n));				\
}

DT_INST_FOREACH_STATUS_OKAY(ETH_HPM_INIT)

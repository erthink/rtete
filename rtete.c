#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/netdevice.h>
#include <linux/spi/spi.h>
#include <linux/etherdevice.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/rtnetlink.h>
#include <net/rtnetlink.h>
#include <linux/u64_stats_sync.h>

/*
	IDT82P2281 по управлению (чтение/запись регистров) сидит на SPI1
	OMAP-L138 (чип-селект ножка GPIO2-5, т.е. используется не аппаратный
	CS OMAP-а  а ножка общего назначения). Данный SPI-контроллер еще
	используется для доступа к регистрам альтеры и UART-а.

	TDM интерфейс (MCBSP1) OMAP-а подключен к альтере (все клоки и
	фрейм-sync-и для OMAPа входные). Там реализован (пока не готов - Леша
	Васильев доделывает) простой HDLC контроллер для HDLC пакетов
	фиксированной длины (один фрейм на TDM шине - это один HDLC пакет -
	альтера выдаст флажки, сделает бит стаффинг, посчитает контрольную
	сумму и выдаст в нефреймированный G.703).
*/

#define RTETE_TX_QUEUE_LEN	4
#define RTETE_IO_CHUNK		256

struct rtete {
	struct spi_device *spi_altera;
	struct spi_device *spi_idt82;
	struct net_device *netdev;

	/* Lock for shared data */
	spinlock_t lock;

	struct spi_message *a, *b;
	u8* dma_buf;
	dma_addr_t dma_addr;

	struct sk_buff *rx_skb;
	struct sk_buff *tx_skb;
	struct sk_buff_head queue;
	unsigned rx_flags;
	u8 tx_char;
};

static int rtete_is_running(struct rtete *rtete) {
	return netif_running(rtete->netdev);
}

#define GOT_ERR	256
#define GOT_ESC	512
#define END		255	/* indicates end of frame	*/
#define ESC		254	/* indicates byte stuffing	*/
#define ESC_END	253	/* ESC ESC_END means END 'data' */
#define ESC_ESC	252	/* ESC ESC_ESC means ESC 'data' */

static void rtete_push(struct rtete *rtete, u8* buf, unsigned len) {
	struct net_device *dev = rtete->netdev;
	unsigned c;

	while(likely(--len)) {
		if (unlikely(! rtete->rx_skb)) {
			rtete->rx_skb = dev_alloc_skb(dev->mtu);
			if (unlikely(rtete->rx_skb == NULL)) {
				printk(KERN_WARNING "%s: memory squeeze, dropping packet.\n", dev->name);
				dev->stats.rx_dropped++;
				return;
			}
			rtete->rx_skb->dev = dev;
		}

		c = *buf++ | rtete->rx_flags;
		if (unlikely(c >= END)) {
			switch (c) {
			case ESC:
				/* LY: got a escape */
				rtete->rx_flags = GOT_ESC;
				continue;
			case GOT_ESC | ESC_END:
				/* LY: escaped END in the 'data' */
				rtete->rx_flags = 0;
				c = END;
				break;
			case GOT_ESC | ESC_ESC:
				/* LY: escaped ESC in the 'data' */
				rtete->rx_flags = 0;
				c = ESC;
				break;
			case END | GOT_ERR:
				/* LY: frame-END after a framing error, reset fsm */
				rtete->rx_flags = 0;
				rtete->rx_skb->tail = rtete->rx_skb->head;
				continue;
			case END:
				/* LY: frame-END */
				if (rtete->rx_skb->tail == rtete->rx_skb->head)
					/* LY: juts a pad, not payload */
					continue;
				rtete->rx_skb->len = rtete->rx_skb->head - rtete->rx_skb->tail;
				if (unlikely(rtete->rx_skb->len < 68)) {
					rtete->rx_skb->tail = rtete->rx_skb->head;
					dev->stats.rx_length_errors++;
					continue;
				}
				skb_reset_mac_header(rtete->rx_skb);
				rtete->rx_skb->protocol = eth_type_trans(rtete->rx_skb, dev);
				dev->stats.rx_bytes += rtete->rx_skb->len;
				dev->stats.rx_packets++;
				netif_receive_skb(rtete->rx_skb);
				rtete->rx_skb = NULL;
				continue;
			default:
				rtete->rx_flags = GOT_ERR;
				dev->stats.rx_frame_errors++;
				continue;
			}
		}

		if (unlikely(rtete->rx_skb->tail == rtete->rx_skb->end)) {
			rtete->rx_flags = GOT_ERR;
			rtete->rx_skb->tail = rtete->rx_skb->head;
			dev->stats.rx_length_errors++;
			continue;
		}
		*rtete->rx_skb->tail++ = c;
	}
}

static void rtete_pull(struct rtete *rtete, char* buf, unsigned len) {
	u8 c;

	while(--len) {
		if (unlikely(rtete->tx_char)) {
			*buf++ = rtete->tx_char;
			rtete->tx_char = 0;
			continue;
		}

		if (unlikely(! rtete->tx_skb)) {
			if (skb_queue_empty(&rtete->queue)) {
				memset(buf, len, END);
				break;
			}
			rtete->tx_skb = __skb_dequeue_tail(&rtete->queue);
		}

		if (unlikely(rtete->tx_skb->head == rtete->tx_skb->tail)) {
			*buf++ = END;
			dev_kfree_skb(rtete->tx_skb);
			rtete->tx_skb = NULL;
			continue;
		}

		c = *rtete->tx_skb->head++;
		if (unlikely(c >= ESC)) {
			rtete->tx_char = (c == ESC) ? ESC_ESC : ESC_END;
			c = ESC;
		}

		*buf++ = c;
	}
}

static void rtete_pump(struct rtete *rtete, struct spi_message* m) {
	unsigned long flags;

	spin_lock_irqsave(&rtete->queue.lock, flags);
	if (likely(rtete_is_running(rtete))) {
		struct spi_transfer *t = (struct spi_transfer *)(m + 1);
		rtete_push(rtete, t->rx_buf, t->len);
		rtete_pull(rtete, (char*) t->tx_buf, t->len);

		if (unlikely(spi_async(rtete->spi_altera, m))) {
			if (rtete->rx_skb)
				rtete->netdev->stats.rx_over_errors++;
			if (rtete->tx_skb)
				rtete->netdev->stats.rx_fifo_errors++;
			m->spi = NULL;
		}
	} else
		m->spi = NULL;
	spin_unlock_irqrestore(&rtete->queue.lock, flags);

	if (rtete_is_running(rtete) && skb_queue_len(&rtete->queue) < RTETE_TX_QUEUE_LEN)
		netif_wake_queue(rtete->netdev);
}

static void rtete_complete_a(void *ctx) {
	struct rtete *rtete = ctx;
	rtete_pump(rtete, rtete->a);
}

static void rtete_complete_b(void *ctx) {
	struct rtete *rtete = ctx;
	rtete_pump(rtete, rtete->b);
}

static void rtete_rammer(struct rtete *rtete) {
	if (rtete_is_running(rtete)) {
		if (! rtete->a->spi)
			rtete_pump(rtete, rtete->a);
		if (! rtete->b->spi)
			rtete_pump(rtete, rtete->b);
	}
}

static netdev_tx_t rtete_dev_xmit(struct sk_buff *skb, struct net_device *dev) {
	struct rtete *rtete;
	unsigned long flags;
	int rc;

	if (unlikely(skb_linearize(skb))) {
		dev->stats.tx_dropped++;
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	rtete = netdev_priv(dev);
	spin_lock_irqsave(&rtete->queue.lock, flags);
	if (unlikely(! netif_running(dev))) {
		printk(KERN_WARNING "%s: xmit call when iface is down\n", dev->name);
		dev->stats.tx_dropped++;
		dev_kfree_skb(skb);
		rc = NETDEV_TX_OK;
	} else if (unlikely(skb->len > dev->mtu)) {
		printk(KERN_WARNING "%s: truncating oversized transmit packet!\n", dev->name);
		dev->stats.tx_dropped++;
		dev_kfree_skb(skb);
		rc = NETDEV_TX_OK;
	} else {
		if (! rtete->tx_skb) {
			rtete->tx_skb = skb;
			rtete_rammer(rtete);
		} else {
			__skb_queue_tail(&rtete->queue, skb);
			if (skb_queue_len(&rtete->queue) > RTETE_TX_QUEUE_LEN)
				netif_stop_queue(dev);
		}
		rc = NETDEV_TX_OK;
	}

	spin_unlock_irqrestore(&rtete->queue.lock, flags);
	return rc;
}

/*-------------------------------------------------------------------*/

/*
 * default ethernet MAC address from Cronyx Enginneering "INDIVIDUAL" LAN
 * MAC Address Block
 */
static u8 default_mac_address[ETH_ALEN] = { 0x00, 0x50, 0xC2, 0x06, 0x28, 0x00 };

static int rtete_set_addr(struct net_device *dev, void *p)
{
	struct sockaddr *addr = p;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	if (netif_running (dev))
		return -EBUSY;

	memcpy(dev->dev_addr, addr->sa_data, ETH_ALEN);
	return 0;
}
static int rtete_change_mtu(struct net_device *dev, int mtu)
{
	if (mtu < 68 || mtu > 65534)
		return -EINVAL;

	dev->mtu = mtu;
	return 0;
}

static int rtete_dev_init(struct net_device *dev) {
	struct rtete *rtete;
	struct spi_transfer *t;

	rtete = netdev_priv(dev);
	rtete->a = spi_message_alloc(1, GFP_KERNEL);
	rtete->b = spi_message_alloc(1, GFP_KERNEL);
	rtete->dma_buf = dma_alloc_coherent(&rtete->spi_altera->dev, RTETE_IO_CHUNK * 4,
										&rtete->dma_addr, GFP_KERNEL | GFP_DMA);
	if (! rtete->a || ! rtete->b || ! rtete->dma_buf)
		return -ENOMEM;

	rtete->a->context = rtete;
	rtete->a->complete = rtete_complete_a;
	rtete->a->is_dma_mapped = 1;
	t = (struct spi_transfer *)(rtete->a + 1);
	t->tx_buf = rtete->dma_buf + RTETE_IO_CHUNK * 0;
	t->tx_dma = rtete->dma_addr + RTETE_IO_CHUNK * 0;
	t->rx_buf = rtete->dma_buf + RTETE_IO_CHUNK * 1;
	t->rx_dma = rtete->dma_addr + RTETE_IO_CHUNK * 1;
	t->len = RTETE_IO_CHUNK;

	rtete->b->context = rtete;
	rtete->b->complete = rtete_complete_b;
	rtete->b->is_dma_mapped = 1;
	t = (struct spi_transfer *)(rtete->b + 1);
	t->tx_buf = rtete->dma_buf + RTETE_IO_CHUNK * 2;
	t->tx_dma = rtete->dma_addr + RTETE_IO_CHUNK * 2;
	t->rx_buf = rtete->dma_buf + RTETE_IO_CHUNK * 3;
	t->rx_dma = rtete->dma_addr + RTETE_IO_CHUNK * 3;
	t->len = RTETE_IO_CHUNK;

	skb_queue_head_init(&rtete->queue);
	return 0;
}

static void rtete_dev_destroy(struct net_device *dev) {
	struct rtete *rtete = netdev_priv(dev);

	spi_message_free(rtete->a);
	spi_message_free(rtete->b);
	dma_free_coherent(&rtete->spi_altera->dev, RTETE_IO_CHUNK * 4,
					  rtete->dma_buf, rtete->dma_addr);

	dev_kfree_skb(rtete->rx_skb);
	dev_kfree_skb(rtete->tx_skb);
	free_netdev(dev);
}

static void rtete_dev_shutdown(struct net_device *dev) {
	struct rtete *rtete = netdev_priv(dev);

	do {
		/* LY: it is a simple way to flush all pending i/o. */
		spi_w8r8(rtete->spi_altera, -1);
	} while (rtete->a->spi || rtete->b->spi);
}

static int rtete_dev_up(struct net_device *dev) {
	struct rtete *rtete = netdev_priv(dev);
	unsigned long flags;

	spin_lock_irqsave(&rtete->queue.lock, flags);
	netif_start_queue(rtete->netdev);
	rtete_rammer(rtete);
	spin_unlock_irqrestore(&rtete->queue.lock, flags);

	return 0;
}

static int rtete_dev_down(struct net_device *dev) {
	struct rtete *rtete = netdev_priv(dev);
	unsigned long flags;

	spin_lock_irqsave(&rtete->queue.lock, flags);
	netif_stop_queue(dev);

	dev_kfree_skb(rtete->rx_skb);
	rtete->rx_skb = NULL;
	rtete->rx_flags = GOT_ERR;

	while (rtete->tx_skb) {
		rtete->netdev->stats.tx_aborted_errors++;
		dev_kfree_skb(rtete->tx_skb);
		rtete->tx_skb = __skb_dequeue_tail(&rtete->queue);
	}
	rtete->tx_char = END;
	spin_unlock_irqrestore(&rtete->queue.lock, flags);

	/* LY: it is a simple way to flush all pending i/o. */
	spi_w8r8(rtete->spi_altera, -1);
	return 0;
}

static void rtete_dev_poll(struct net_device *dev) {
	struct rtete *rtete = netdev_priv(dev);
	unsigned long flags;

	if (! netif_xmit_frozen_or_stopped(netdev_get_tx_queue(dev, 0))) {
		spin_lock_irqsave(&rtete->queue.lock, flags);
		rtete_rammer(rtete);
		spin_unlock_irqrestore(&rtete->queue.lock, flags);
	}
}

static const struct net_device_ops rtete_netdev_ops = {
	.ndo_init		= rtete_dev_init,
	.ndo_uninit		= rtete_dev_shutdown,
	.ndo_open		= rtete_dev_up,
	.ndo_stop		= rtete_dev_down,
	.ndo_start_xmit		= rtete_dev_xmit,
	.ndo_poll_controller = rtete_dev_poll,
	.ndo_validate_addr	= eth_validate_addr,
	/* .ndo_set_rx_mode	= set_multicast_list, */
	.ndo_set_mac_address	= rtete_set_addr,
	.ndo_change_mtu			= rtete_change_mtu,
	//.ndo_get_stats64	= rtete_get_stats64
};

static void rtete_setup(struct net_device *dev) {
	ether_setup(dev);

	/* Initialize the device structure. */
	dev->netdev_ops = &rtete_netdev_ops;
	dev->destructor = rtete_dev_destroy;

	/* Fill in device structure with ethernet-generic values. */
	dev->tx_queue_len = RTETE_TX_QUEUE_LEN;
	/* dev->flags &= ~IFF_MULTICAST;
	dev->features = 0; */

	/* set ethernet MAC address */
	random_ether_addr(dev->dev_addr);
	dev->dev_addr[ETH_ALEN - 1] = default_mac_address[ETH_ALEN - 1];
	dev->dev_addr[ETH_ALEN - 2] = default_mac_address[ETH_ALEN - 2];
	dev->dev_addr[ETH_ALEN - 3] = default_mac_address[ETH_ALEN - 3];
}

/*-------------------------------------------------------------------*/

int omapl138_rtbi_rtete_init(struct spi_device *spi_altera, struct spi_device *spi_idt82) {
	struct net_device *dev;
	struct rtete *rtete;
	int rc;

	dev = alloc_netdev(sizeof(struct rtete), "rtete%d", rtete_setup);
	if (!dev)
		return -ENOMEM;

	rtete = netdev_priv(dev);
	rtete->netdev = dev;
	rtete->spi_altera = spi_altera;
	rtete->spi_idt82 = spi_idt82;

	rc = register_netdev(dev);
	if(rc)
		rtete_dev_destroy(dev);
	return rc;
}


static int __init rtete_init_module(void) {
	printk ("rtete-init/loaded.\n");
	return 0;
}

static void __exit rtete_cleanup_module(void) {
	printk ("rtete-exit/unloaded.\n");
}

module_init(rtete_init_module);
module_exit(rtete_cleanup_module);
MODULE_LICENSE("GPL");
MODULE_ALIAS_RTNL_LINK("rtete");

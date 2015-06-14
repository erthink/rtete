#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/crc32.h>

#ifndef MAX_RT_PRIO
#	include <linux/sched/rt.h>
#endif

#include <linux/spi/spi.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

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

/*
 * NOTE:  terminology here is confusing.
 *
 *  - But it labels it a "Multi-channel Buffered Serial Port"
 *    (McBSP) as on older chips like the dm642 ... which was
 *    backward-compatible, possibly explaining that confusion.
 *
 *  - OMAP chips have a controller called McBSP, which is
 *    incompatible with the DaVinci flavor of McBSP.
 *
 *  - Newer DaVinci chips have a controller called McASP,
 *    incompatible with ASP and with either McBSP.
 *
 */

#define RTETE_TX_QUEUE_LEN	16
#define RTETE_IO_CHUNK		1024
#define	RTETE_SPI			0

/* LY: минимальный размер пакета, два mac, протокол, байт payload, crc */
#define	RTETE_PKTLEN_MIN		(6+6+2+1+4)

/* LY: байтик для для "отбеливания" данных xor-ом. */
#define	RTETE_SALT			0x42

struct iopcm {
	const char* filename;
	struct snd_pcm_substream *substream;
	struct file *filp;
	u8* buf;
	int chunk_frames;
};

struct iospi {
	struct spi_message *a, *b;
	u8* dma_buf;
	dma_addr_t dma_addr;
};

struct rtete {
	struct spi_device *spi_altera;
	struct spi_device *spi_idt82;
	struct net_device *netdev;

	struct iospi iospi;
	struct iopcm rx, tx;
	int tx_watermark;

	struct task_struct *thread;

	u8 *rx_ptr, *rx_end;
	unsigned rx_flags;

	u8 *tx_ptr, *tx_end;
	u8 tx_char;

	u8 *rx_begin;
	struct sk_buff *rx_skb;
	struct sk_buff *tx_skb;
	struct sk_buff_head queue;
};

static int rtete_is_running(struct rtete *rtete);

#define DEBUG(rtete, format, ...) \
	printk(KERN_DEBUG "%s: " format, rtete->netdev->name, ##__VA_ARGS__)
#define INFO(rtete, format, ...) \
	printk(KERN_INFO "%s: " format, rtete->netdev->name, ##__VA_ARGS__)
#define NOTICE(rtete, format, ...) \
	printk(KERN_NOTICE "%s: " format, rtete->netdev->name, ##__VA_ARGS__)
#define WARNING(rtete, format, ...) \
	printk(KERN_WARNING "%s: " format, rtete->netdev->name, ##__VA_ARGS__)
#define ERROR(rtete, format, ...) \
	printk(KERN_ERR "%s: " format, rtete->netdev->name, ##__VA_ARGS__)

#define TRACE(rtete) \
	DEBUG(rtete, "%s %d\n", __FUNCTION__, __LINE__);

static void dump(const char *title, const u8 *data, int len)
{
	int i;

	printk ("rtete:");
	for (i = 0; i < len; i++) {
		if (!(i % 32))
			printk ("\n%s:", title);
		if (!(i % 4))
			printk (" ");
		printk ("%02x", data[i]);
	}
	printk ("\n");
}

static int rtete_get_carrier(struct rtete *rtete)
{
	// TODO
	return 1;
}

/*-------------------------------------------------------------------
 * Фрейминг и байт-стаффинг.
 */

#define GOT_ERR	256
#define GOT_ESC	512
#define END		255	/* indicates end of frame	*/
#define ESC		254	/* indicates byte stuffing	*/
#define ESC_END	253	/* ESC ESC_END means END 'data' */
#define ESC_ESC	252	/* ESC ESC_ESC means ESC 'data' */

static void rtete_rx_error(struct rtete *rtete)
{
	//TRACE(rtete);
	rtete->netdev->stats.rx_errors++;
	rtete->rx_flags = GOT_ERR;
}

static void rtete_parse_rx(struct rtete *rtete, u8* src, int src_len) {
	struct net_device *dev = rtete->netdev;
	int pkt_len;
	u32 crc1, crc2;

	do {
		unsigned c = *src++ | rtete->rx_flags;
		if (c >= ESC) {
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
				//TRACE(rtete);
				rtete->rx_flags = 0;
				rtete->rx_ptr = rtete->rx_begin;
				continue;
			case END:
				/* LY: frame-END */
				if (rtete->rx_ptr == rtete->rx_begin)
					/* LY: juts a pad, not payload */
					continue;
				if (unlikely(rtete->rx_ptr - rtete->rx_begin < RTETE_PKTLEN_MIN)) {
					//TRACE(rtete);
					dev->stats.rx_length_errors++;
					rtete->netdev->stats.rx_errors++;
					rtete->rx_ptr = rtete->rx_begin;
					continue;
				}

				pkt_len = rtete->rx_ptr - rtete->rx_begin - sizeof(u32);
				crc1 = ether_crc(pkt_len, rtete->rx_begin);
				crc2 = get_unaligned_le32(rtete->rx_ptr - sizeof(u32));
				if (crc1 != crc2) {
					//DEBUG(rtete, "rx-crc: len %d, crc 0x%08x / 0x%08x\n", pkt_len, crc1, crc2);
					dev->stats.rx_crc_errors++;
					rtete->netdev->stats.rx_errors++;
					rtete->rx_ptr = rtete->rx_begin;
					continue;
				}

				skb_put(rtete->rx_skb, pkt_len);
				dump("rx-pkt", rtete->rx_skb->data, rtete->rx_skb->len);
				dev->stats.rx_bytes += rtete->rx_skb->len;
				dev->stats.rx_packets++;
				skb_reset_mac_header(rtete->rx_skb);
				rtete->rx_skb->protocol = eth_type_trans(rtete->rx_skb, dev);
				netif_receive_skb(rtete->rx_skb);
				rtete->rx_skb = NULL;
				rtete->rx_ptr = rtete->rx_begin = rtete->rx_end = NULL;
				continue;
			default:
				//TRACE(rtete);
				if (rtete->rx_flags != GOT_ERR) {
					rtete->rx_flags = GOT_ERR;
					dev->stats.rx_frame_errors++;
				}
				continue;
			}
		}

		if (unlikely(rtete->rx_ptr == rtete->rx_end)) {
			//TRACE(rtete);
			if (rtete->rx_skb) {
				rtete->rx_flags = GOT_ERR;
				dev->stats.rx_length_errors++;
				rtete->rx_ptr = rtete->rx_begin;
				continue;
			}

			rtete->rx_skb = dev_alloc_skb(dev->mtu + sizeof(u32/* CRC */));
			if (unlikely(rtete->rx_skb == NULL)) {
				WARNING(rtete, "memory squeeze, dropping packet.\n");
				dev->stats.rx_dropped++;
				rtete->rx_flags = GOT_ERR;
				continue;
			}
			rtete->rx_skb->dev = dev;
			rtete->rx_begin = rtete->rx_skb->data;
			rtete->rx_ptr = rtete->rx_begin;
			rtete->rx_end = rtete->rx_begin + skb_availroom(rtete->rx_skb);
			BUG_ON(rtete->rx_ptr >= rtete->rx_end);
		}

		rtete->rx_ptr[0] = c ^ RTETE_SALT;
		rtete->rx_ptr += 1;
	} while(likely(--src_len));

	//TRACE(rtete);
}

static void rtete_tx_error(struct rtete *rtete)
{
	//TRACE(rtete);
	rtete->netdev->stats.tx_errors++;
	/* LY: додумать - желательно явно поломать фрейминг на передачу. */
	rtete->tx_char = END;
}

static void rtete_fill_tx(struct rtete *rtete, u8* dst, int dst_len) {
	do {
		u8 c;

		if (unlikely(rtete->tx_char)) {
			c = rtete->tx_char;
			rtete->tx_char = 0;
		} else if (unlikely(rtete->tx_ptr == rtete->tx_end)) {
			if (rtete->tx_skb) {
				dev_kfree_skb(rtete->tx_skb);
				rtete->tx_skb = NULL;
			}

#if RTETE_SPI
			if (skb_queue_empty(&rtete->queue)) {
				memset(dst, END, dst_len);
				break;
			}
			rtete->tx_skb = __skb_dequeue_tail(&rtete->queue);
#else
			rtete->tx_skb = skb_dequeue_tail(&rtete->queue);
			if (! rtete->tx_skb) {
				memset(dst, END, dst_len);
				break;
			}
#endif
			if (rtete_is_running(rtete) && skb_queue_len(&rtete->queue) < rtete->netdev->tx_queue_len)
				netif_wake_queue(rtete->netdev);

			dump("tx-pkt", rtete->tx_skb->data, rtete->tx_skb->len);
			rtete->netdev->stats.tx_bytes += rtete->tx_skb->len - sizeof(u32 /* crc */);
			rtete->netdev->stats.tx_packets++;
			rtete->tx_ptr = rtete->tx_skb->data;
			rtete->tx_end = rtete->tx_ptr + rtete->tx_skb->len;
			c = END;
		} else {
			c = rtete->tx_ptr[0] ^ RTETE_SALT;
			rtete->tx_ptr += 1;
			if (unlikely(c >= ESC)) {
				rtete->tx_char = (c == ESC) ? ESC_ESC : ESC_END;
				c = ESC;
			}
		}

		*dst++ = c;
	} while(likely(--dst_len));
}

/*-------------------------------------------------------------------
 * Взаимодействие с ALSA-PCM.
 */

static snd_pcm_sframes_t rtete_iopcm_read(struct rtete *rtete, void *buf, snd_pcm_uframes_t frames)
{
	struct snd_pcm_substream *substream = rtete->rx.substream;
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_pcm_sframes_t ret;
	mm_segment_t old_fs;

	while(1) {
		if (runtime->status->state == SNDRV_PCM_STATE_XRUN
				|| runtime->status->state == SNDRV_PCM_STATE_SUSPENDED) {
			TRACE(rtete);
			ret = snd_pcm_kernel_ioctl(substream, SNDRV_PCM_IOCTL_DRAIN, NULL);
			if (ret < 0) {
				ERROR(rtete, "Preparing sound card failed: %d\n", (int) ret);
				rtete_rx_error(rtete);
				return ret;
			}
		} else if (runtime->status->state == SNDRV_PCM_STATE_SETUP) {
			TRACE(rtete);
			ret = snd_pcm_kernel_ioctl(substream, SNDRV_PCM_IOCTL_PREPARE, NULL);
			if (ret < 0) {
				ERROR(rtete, "Preparing sound card failed: %d\n", (int) ret);
				rtete_rx_error(rtete);
				return ret;
			}
		}

		/* LY: TODO: тут можно заменить вызов snd_pcm_lib_read() на часть её потрохов,
		 * уменьшив тем самым overhead и смену сегментов. */
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		ret = snd_pcm_lib_read(substream, (void __user *) buf, frames);
		set_fs(old_fs);

		if (ret < 0) {
			if (ret == -EPIPE) {
				//DEBUG(rtete, "PCM-capture overrun\n");
				rtete->netdev->stats.rx_fifo_errors++;
				rtete_rx_error(rtete);
				ret = snd_pcm_kernel_ioctl(substream, SNDRV_PCM_IOCTL_DROP, NULL);
				if (ret == 0)
					continue;
			}
			ERROR(rtete, "PCM-read failed: %d\n", (int) ret);
			rtete_rx_error(rtete);
			if (ret == -ESTRPIPE && runtime->status->state != SNDRV_PCM_STATE_PREPARED)
				continue;
		}

		/* {
			int i;
			const u8* b = buf;
			for(i = 0; i < RTETE_IO_CHUNK; ++i)
				if (b[i] != END) {
					dump("rx-read", b+i, (RTETE_IO_CHUNK - i) > 128 ? 128 : (RTETE_IO_CHUNK - i));
					break;
				}
		} */

		//TRACE(rtete);
		return ret;
	}
}

static snd_pcm_sframes_t rtete_iopcm_write(struct rtete *rtete, void *buf, snd_pcm_uframes_t frames)
{
	struct snd_pcm_substream *substream = rtete->tx.substream;
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_pcm_sframes_t ret;
	mm_segment_t old_fs;

	/* {
		int i;
		const u8* b = buf;
		for(i = 0; i < RTETE_IO_CHUNK; ++i)
			if (b[i] != END) {
				dump("tx-write", b+i, (RTETE_IO_CHUNK - i) > 128 ? 128 : (RTETE_IO_CHUNK - i));
				break;
			}
	} */

	while(1) {
		if (runtime->status->state == SNDRV_PCM_STATE_XRUN
				|| runtime->status->state == SNDRV_PCM_STATE_SUSPENDED
				|| runtime->status->state == SNDRV_PCM_STATE_SETUP ) {
			//TRACE(rtete);
			ret = snd_pcm_kernel_ioctl(substream, SNDRV_PCM_IOCTL_PREPARE, NULL);
			if (ret < 0) {
				ERROR(rtete, "Preparing sound card failed: %d\n", (int) ret);
				rtete_tx_error(rtete);
				return ret;
			}
		}

		/* LY: TODO: тут можно заменить вызов snd_pcm_lib_write() на часть её потрохов,
		 * уменьшив тем самым overhead и смену сегментов. */
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		ret = snd_pcm_lib_write(substream, (void __user *) buf, frames);
		set_fs(old_fs);

		if (ret != frames) {
			if (ret == -EPIPE) {
				//DEBUG(rtete, "PCM-payback underrun\n");
				rtete->netdev->stats.tx_fifo_errors++;
				rtete_tx_error(rtete);
				if (rtete->netdev->stats.tx_fifo_errors % 3 == 0) {
					rtete->tx_watermark += rtete->tx_watermark / 4 + 1;
					NOTICE(rtete, "to avoid pcm-underrun queue extended to %d\n", rtete->tx_watermark);
				}
				continue;
			}

			ERROR(rtete, "PCM-write failed: %d\n", (int) ret);
			rtete_tx_error(rtete);
			if (ret == -ESTRPIPE && runtime->status->state != SNDRV_PCM_STATE_PREPARED)
				continue;
		}

		//TRACE(rtete);
		return ret;
	}
}

static int snd_interval_minmax(struct snd_interval *i, int min, int max)
{
	struct snd_interval mm;

	snd_interval_any(&mm);
	mm.min = min;
	mm.max = max;

	return snd_interval_refine(i, &mm);
}

static int rtete_setup_pcm(struct rtete *rtete, struct iopcm *alsa, const char *suffix)
{
	struct snd_pcm_hw_params *params = NULL;
	int err = -ENOMEM;
	struct snd_pcm_file *pcm = alsa->filp->private_data;
	struct snd_pcm_runtime *runtime = NULL;
	struct snd_mask mask;
	u8 silence[8];

	TRACE(rtete);
	alsa->substream = pcm->substream;
	runtime = alsa->substream->runtime;
	params = kzalloc(sizeof(*params), GFP_KERNEL);
	if (! params)
		goto bailout;

	/* LY: setup someone suitable mode. */
	_snd_pcm_hw_params_any(params);
	snd_mask_refine_set(hw_param_mask(params, SNDRV_PCM_HW_PARAM_ACCESS), SNDRV_PCM_ACCESS_RW_INTERLEAVED);

	snd_mask_none(&mask);
	snd_mask_set(&mask, SNDRV_PCM_FORMAT_U8);
	snd_mask_set(&mask, SNDRV_PCM_FORMAT_S8);
	/* snd_mask_set(&mask, SNDRV_PCM_FORMAT_U16_LE);
	snd_mask_set(&mask, SNDRV_PCM_FORMAT_S16_LE);
	snd_mask_set(&mask, SNDRV_PCM_FORMAT_U32_LE);
	snd_mask_set(&mask, SNDRV_PCM_FORMAT_S32_LE); */
	snd_mask_refine(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT), &mask);

	snd_interval_minmax(hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS), 2, 2);
	snd_interval_minmax(hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE), 44100, 64000);

	snd_interval_minmax(hw_param_interval(params, SNDRV_PCM_HW_PARAM_PERIOD_BYTES), RTETE_IO_CHUNK, RTETE_IO_CHUNK * 3 / 2);
	snd_interval_minmax(hw_param_interval(params, SNDRV_PCM_HW_PARAM_BUFFER_BYTES), RTETE_IO_CHUNK * 2, RTETE_IO_CHUNK * RTETE_TX_QUEUE_LEN * 2);

	err = snd_pcm_kernel_ioctl(alsa->substream, SNDRV_PCM_IOCTL_HW_REFINE, params);
	if (err == -EINVAL) {
		NOTICE(rtete, "pcm-%s fallback to non-raw format\n", suffix);
		snd_mask_any(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT));
		err = snd_pcm_kernel_ioctl(alsa->substream, SNDRV_PCM_IOCTL_HW_REFINE, params);
	}
	if (err < 0) {
		WARNING(rtete, "snd_pcm_kernel_ioctl(%s, SNDRV_PCM_IOCTL_HW_REFINE), %d\n", suffix, err);
	}

	err = snd_pcm_kernel_ioctl(alsa->substream, SNDRV_PCM_IOCTL_HW_PARAMS, params);
	if (err < 0) {
		ERROR(rtete, "snd_pcm_kernel_ioctl(%s, SNDRV_PCM_IOCTL_HW_PARAMS), %d\n", suffix, err);
		goto bailout;
	}

	INFO(rtete, "%s-pcm-config: access %x, format %x, channels %d, rate %d, "
		"frame-bytes %d, sample-bits %d\n", suffix,
		params_access(params), params_format(params),
		params_channels(params), params_rate(params),
		runtime->frame_bits / 8, runtime->sample_bits );

	INFO(rtete, "%s-pcm-info: period-frames %d, period-bytes %d, buffer-frames %d, buffer-bytes %d, "
		"runtime-buffer-frames %lu, runtime-buffer-bytes %zu, start-frames %lu, start-bytes %zu\n", suffix,
		params_period_size(params), hw_param_interval_c(params, SNDRV_PCM_HW_PARAM_PERIOD_BYTES)->min,
		params_buffer_size(params), params_buffer_bytes(params),
		runtime->buffer_size, snd_pcm_lib_buffer_bytes(alsa->substream),
		runtime->start_threshold, frames_to_bytes(runtime, runtime->start_threshold)
		);

	/* LY: ход конем, устанавливаем "тишину" */
	TRACE(rtete);
	memset(&silence, END, sizeof(silence));
	err = snd_pcm_format_set_silence(params_format(params), &silence, sizeof(silence) * 8 / runtime->sample_bits);
	if (err < 0) {
		WARNING(rtete, "snd_pcm_format_set_silence(%s, END-PADDING), %d\n", suffix, err);
	}

	if (snd_pcm_lib_buffer_bytes(alsa->substream) < RTETE_IO_CHUNK * 2) {
		WARNING(rtete, "pcm-%s not enought pcm-buffer length (%zu < %u)\n", suffix,
			  snd_pcm_lib_buffer_bytes(alsa->substream), RTETE_IO_CHUNK * 2);
	}

	alsa->chunk_frames = bytes_to_frames(runtime, RTETE_IO_CHUNK);
	if (frames_to_bytes(runtime, alsa->chunk_frames) != RTETE_IO_CHUNK) {
		ERROR(rtete, "pcm-%s unsuitable pcm-frame size (%d != %d)\n",
			  suffix, alsa->chunk_frames, RTETE_IO_CHUNK);
		err = -EIO;
		goto bailout;
	}

bailout:
	kfree(params);
	TRACE(rtete);
	return err;
}

static int rtete_iopcm_pump(struct rtete *rtete, int wait) {
	snd_pcm_sframes_t frames;
	//TRACE(rtete);

	/* LY: пополняем pcm-очередь на передачу, никогда не ждем здесь. */
	while(1) {
		if (! skb_queue_empty(&rtete->queue)) {
			/* LY: Есть сетевые пакеты на передачу.
			 * Смотрим есть ли место хотя-бы для одного RTETE_IO_CHUNK. */
			frames = snd_pcm_playback_avail(rtete->tx.substream->runtime);
			if (frames < rtete->tx.chunk_frames)
				break;
		} else {
			/* LY: Очередь сетевых пакетов пуста.
			 * Смотрим насколько заполнен pcm-буффер и не ставим на отправку лишних пустышек.
			 * Так задержка на отправку последующих пакетов будет меньше. */
			frames = snd_pcm_playback_hw_avail(rtete->tx.substream->runtime);
			if (frames > rtete->tx_watermark)
				break;
		}

		rtete_fill_tx(rtete, rtete->tx.buf, RTETE_IO_CHUNK);
		frames = rtete_iopcm_write(rtete, rtete->tx.buf, rtete->tx.chunk_frames);
		if (frames != rtete->tx.chunk_frames)
			break;
	}

	/* LY: теперь вычитываем pcm-очередь по приёму, не ждем если нельзя. */
	if (! wait) {
		frames = snd_pcm_capture_avail(rtete->rx.substream->runtime);
		if (frames < rtete->rx.chunk_frames) {
			//TRACE(rtete);
			return 0;
		}
	}

	//TRACE(rtete);
	frames = rtete_iopcm_read(rtete, rtete->rx.buf, rtete->rx.chunk_frames);
	if (frames > 0)
		rtete_parse_rx(rtete, rtete->rx.buf, frames_to_bytes(rtete->rx.substream->runtime, frames));

	//TRACE(rtete);
	return frames;
}

static int rtete_iopcm_thread(void *ctx) {
	struct rtete *rtete = ctx;
	int ret = 0;

	TRACE(rtete);
	while(! kthread_should_stop()) {

		/* LY: если были какие-то проблемы, то избегаем busyloop */
		if (ret < 0) {
			schedule_timeout(1);
			if (kthread_should_stop())
				break;
		}

		ret = rtete_iopcm_pump(rtete, 1 /* wait for data */);
	}

	TRACE(rtete);
	return 0;
}

static int rtete_iopcm_open(struct rtete *rtete)
{
	int err;
	TRACE(rtete);

	rtete->tx.buf = kmalloc(RTETE_IO_CHUNK, GFP_KERNEL | GFP_DMA);
	rtete->rx.buf = kmalloc(RTETE_IO_CHUNK, GFP_KERNEL | GFP_DMA);
	if (! rtete->tx.buf || ! rtete->rx.buf)
		return -ENOMEM;

	rtete->tx.filp = filp_open(rtete->tx.filename, O_WRONLY | O_NONBLOCK, 0);
	if (IS_ERR(rtete->tx.filp)) {
		err = PTR_ERR(rtete->tx.filp);
		rtete->tx.filp = NULL;
		ERROR(rtete, "No such PCM-playback device: %s\n", rtete->tx.filename);
		return err;
	}

	rtete->rx.filp = filp_open(rtete->rx.filename, O_RDONLY, 0);
	if (IS_ERR(rtete->rx.filp)) {
		err = PTR_ERR(rtete->rx.filp);
		rtete->rx.filp = NULL;
		ERROR(rtete, "No such PCM-capture device: %s\n", rtete->rx.filename);
		return err;
	}

	err = rtete_setup_pcm(rtete, &rtete->tx, "tx");
	if (err)
		return err;

	err = rtete_setup_pcm(rtete, &rtete->rx, "rx");
	if (err)
		return err;

	rtete->tx_watermark = rtete->tx.chunk_frames / 2;
	if (rtete->tx_watermark < rtete->tx.substream->runtime->period_size)
		rtete->tx_watermark = rtete->tx.substream->runtime->period_size;
	if (rtete->tx_watermark < rtete->rx.substream->runtime->period_size)
		rtete->tx_watermark = rtete->rx.substream->runtime->period_size;

	rtete->tx_watermark += rtete->tx_watermark / 2
			+ rtete->tx.substream->runtime->start_threshold;

	TRACE(rtete);
	return 0;
}

static void rtete_iopcm_close(struct rtete *rtete)
{
	TRACE(rtete);
	if (rtete->thread) {
		kthread_stop(rtete->thread);
		rtete->thread = NULL;
	}

	TRACE(rtete);
	if (rtete->tx.filp) {
		filp_close(rtete->tx.filp, NULL);
		rtete->tx.filp = NULL;
	}

	TRACE(rtete);
	if (rtete->rx.filp) {
		filp_close(rtete->rx.filp, NULL);
		rtete->rx.filp = NULL;
	}

	kfree(rtete->tx.buf);
	rtete->tx.buf = NULL;
	kfree(rtete->rx.buf);
	rtete->rx.buf = NULL;
	TRACE(rtete);
}

static int rtete_iopcm_start(struct rtete *rtete)
{
	struct sched_param sched_param;
	int err;

	TRACE(rtete);
	if (! rtete->thread) {
		rtete->thread = kthread_create(rtete_iopcm_thread, rtete, "%s/alsa", rtete->netdev->name);
		if (IS_ERR(rtete->thread)) {
			err = PTR_ERR(rtete->thread);
			rtete->thread = NULL;
			return err;
		}
		memset(&sched_param, 0, sizeof(sched_param));
		sched_param.sched_priority = MAX_RT_PRIO - 1;
		err = sched_setscheduler/*_nocheck*/(rtete->thread, SCHED_RR, &sched_param);
		if (err < 0) {
			WARNING(rtete, "sched_setscheduler(pipe-thread, %d), %d\n", sched_param.sched_priority, err);
		}

		snd_pcm_kernel_ioctl(rtete->tx.substream, SNDRV_PCM_IOCTL_DROP, NULL);
		snd_pcm_kernel_ioctl(rtete->rx.substream, SNDRV_PCM_IOCTL_DROP, NULL);

		err = snd_pcm_kernel_ioctl(rtete->tx.substream, SNDRV_PCM_IOCTL_PREPARE, NULL);
		if (err < 0) {
			ERROR(rtete, "snd_pcm_kernel_ioctl(tx, SNDRV_PCM_IOCTL_PREPARE), %d\n", err);
			return err;
		}

		err = snd_pcm_kernel_ioctl(rtete->rx.substream, SNDRV_PCM_IOCTL_PREPARE, NULL);
		if (err < 0) {
			ERROR(rtete, "snd_pcm_kernel_ioctl(rx, SNDRV_PCM_IOCTL_PREPARE), %d\n", err);
			return err;
		}

		rtete_iopcm_pump(rtete, 0);
		wake_up_process(rtete->thread);
	}

	return 0;
}

static void rtete_iopcm_stop(struct rtete *rtete)
{
	if (rtete->thread) {
		kthread_stop(rtete->thread);
		rtete->thread = NULL;
	}
}

/*-------------------------------------------------------------------
 * Вариант обмена "через SPI", ну а может пригодится...
 */

#ifndef CONFIG_SPI
int spi_async(struct spi_device *spi, struct spi_message *message) {
	return -ENOSYS;
}

int spi_write_then_read(struct spi_device *spi, const void *txbuf, unsigned n_tx, void *rxbuf, unsigned n_rx) {
	return -ENOSYS;
}
#endif

static void rtete_iospi_pump(struct rtete *rtete, struct spi_message* m)
{
	TRACE(rtete);
	if (likely(rtete_is_running(rtete))) {
		struct spi_transfer *t = (struct spi_transfer *)(m + 1);
		rtete_parse_rx(rtete, t->rx_buf, t->len);
		rtete_fill_tx(rtete, (char*) t->tx_buf, t->len);

		if (unlikely(spi_async(rtete->spi_altera, m))) {
			if (rtete->rx_skb)
				rtete->netdev->stats.rx_over_errors++;
			if (rtete->tx_skb)
				rtete->netdev->stats.tx_aborted_errors++;
			m->spi = NULL;
		}
	} else
		m->spi = NULL;
}

static void rtete_iospi_complete_a(void *ctx)
{
	struct rtete *rtete = ctx;
	unsigned long flags;
	TRACE(rtete);

	spin_lock_irqsave(&rtete->queue.lock, flags);
	rtete_iospi_pump(rtete, rtete->iospi.a);
	spin_unlock_irqrestore(&rtete->queue.lock, flags);
}

static void rtete_iospi_complete_b(void *ctx)
{
	struct rtete *rtete = ctx;
	unsigned long flags;
	TRACE(rtete);

	spin_lock_irqsave(&rtete->queue.lock, flags);
	rtete_iospi_pump(rtete, rtete->iospi.b);
	spin_unlock_irqrestore(&rtete->queue.lock, flags);
}

static void rtete_iospi_rammer(struct rtete *rtete) {
#if RTETE_SPI
	unsigned long flags;

	spin_lock_irqsave(&rtete->queue.lock, flags);
	if (! rtete->iospi.a->spi)
		rtete_iospi_pump(rtete, rtete->iospi.a);
	if (! rtete->iospi.b->spi)
		rtete_iospi_pump(rtete, rtete->iospi.b);
	spin_unlock_irqrestore(&rtete->queue.lock, flags);

	TRACE(rtete);
#endif
}

static int rtete_iospi_open(struct rtete *rtete) {
#if RTETE_SPI
	struct spi_transfer *t;
	TRACE(rtete);

	rtete->iospi.a = spi_message_alloc(1, GFP_KERNEL);
	rtete->iospi.b = spi_message_alloc(1, GFP_KERNEL);
	rtete->iospi.dma_buf = dma_alloc_coherent(&rtete->spi_altera->dev, RTETE_IO_CHUNK * 4,
								&rtete->iospi.dma_addr, GFP_KERNEL | GFP_DMA);

	if (! rtete->iospi.a || ! rtete->iospi.b || ! rtete->iospi.dma_buf)
		return -ENOMEM;

	rtete->iospi.a->context = rtete;
	rtete->iospi.a->complete = rtete_iospi_complete_a;
	rtete->iospi.a->is_dma_mapped = 1;
	t = (struct spi_transfer *)(rtete->iospi.a + 1);
	t->tx_buf = rtete->iospi.dma_buf + RTETE_IO_CHUNK * 0;
	t->tx_dma = rtete->iospi.dma_addr + RTETE_IO_CHUNK * 0;
	t->rx_buf = rtete->iospi.dma_buf + RTETE_IO_CHUNK * 1;
	t->rx_dma = rtete->iospi.dma_addr + RTETE_IO_CHUNK * 1;
	t->len = RTETE_IO_CHUNK;

	rtete->iospi.b->context = rtete;
	rtete->iospi.b->complete = rtete_iospi_complete_b;
	rtete->iospi.b->is_dma_mapped = 1;
	t = (struct spi_transfer *)(rtete->iospi.b + 1);
	t->tx_buf = rtete->iospi.dma_buf + RTETE_IO_CHUNK * 2;
	t->tx_dma = rtete->iospi.dma_addr + RTETE_IO_CHUNK * 2;
	t->rx_buf = rtete->iospi.dma_buf + RTETE_IO_CHUNK * 3;
	t->rx_dma = rtete->iospi.dma_addr + RTETE_IO_CHUNK * 3;
	t->len = RTETE_IO_CHUNK;
#endif

	return 0;
}

static void rtete_iospi_close(struct rtete *rtete) {
	TRACE(rtete);
	spi_message_free(rtete->iospi.a);
	rtete->iospi.a = NULL;

	spi_message_free(rtete->iospi.b);
	rtete->iospi.b = NULL;

	if (rtete->iospi.dma_buf) {
		dma_free_coherent(&rtete->spi_altera->dev, RTETE_IO_CHUNK * 4,
						  rtete->iospi.dma_buf, rtete->iospi.dma_addr);
		rtete->iospi.dma_buf = NULL;
		rtete->iospi.dma_addr = 0;
	}
}

/*-------------------------------------------------------------------
 * Сетевой уровень.
 */

static netdev_tx_t rtete_dev_xmit(struct sk_buff *skb, struct net_device *dev) {
	struct rtete *rtete = netdev_priv(dev);
	int rc;
	//TRACE(rtete);

	if (unlikely(skb_tailroom(skb) < sizeof(u32 /* crc */))) {
		dev->stats.tx_dropped++;
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	if (unlikely(! netif_running(dev))) {
		WARNING(rtete, "xmit call when iface is down\n");
		dev->stats.tx_dropped++;
		dev_kfree_skb(skb);
		rc = NETDEV_TX_OK;
	} else if (unlikely(skb->len > dev->mtu)) {
		WARNING(rtete, "truncating oversized transmit packet!\n");
		dev->stats.tx_dropped++;
		dev_kfree_skb(skb);
		rc = NETDEV_TX_OK;
	} else {
		u32 crc = ether_crc(skb->len, skb->data);
		//DEBUG(rtete, "xmit-crc: len %d->%d, crc 0x%08x\n", skb->len, skb->len - 4, crc);
		put_unaligned_le32(crc, skb_put(skb, sizeof(u32)));
		skb_queue_tail(&rtete->queue, skb);
		if (skb_queue_len(&rtete->queue) > rtete->netdev->tx_queue_len) {
			TRACE(rtete);
			netif_stop_queue(dev);
		}

		rc = NETDEV_TX_OK;
	}

	//TRACE(rtete);
	return rc;
}

static int rtete_is_running(struct rtete *rtete) {
	return netif_running(rtete->netdev);
}

static void rtete_report_carrier(struct rtete *rtete) {
	if (rtete_get_carrier(rtete))
		netif_carrier_on(rtete->netdev);
	else
		netif_carrier_off(rtete->netdev);
}

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

static int rtete_dev_up(struct net_device *dev) {
	struct rtete *rtete = netdev_priv(dev);
	int err;

	TRACE(rtete);
	rtete->tx_ptr = rtete->tx_end = NULL;
	rtete->rx_ptr = rtete->rx_begin = rtete->rx_end = NULL;
	rtete->rx_flags = GOT_ERR;
	rtete->tx_char = END;
	err = rtete_iopcm_start(rtete);
	if (err < 0)
		return err;

	rtete_iospi_rammer(rtete);
	TRACE(rtete);

	rtete_report_carrier(rtete);
	memset (&rtete->netdev->stats, 0, sizeof (rtete->netdev->stats));
	netif_start_queue(rtete->netdev);
	TRACE(rtete);

	return 0;
}

static int rtete_dev_down(struct net_device *dev) {
	struct rtete *rtete = netdev_priv(dev);
	unsigned long flags;
	TRACE(rtete);

	netif_stop_queue(dev);
	rtete_iopcm_stop(rtete);

	spin_lock_irqsave(&rtete->queue.lock, flags);
	dev_kfree_skb(rtete->rx_skb);
	rtete->rx_skb = NULL;
	rtete->rx_ptr = rtete->rx_begin = rtete->rx_end = NULL;
	rtete->rx_flags = GOT_ERR;

	TRACE(rtete);

	while (rtete->tx_skb) {
		rtete->netdev->stats.tx_aborted_errors++;
		dev_kfree_skb(rtete->tx_skb);
		rtete->tx_skb = skb_dequeue_tail(&rtete->queue);
	}
	rtete->tx_char = END;
	rtete->tx_ptr = rtete->tx_end = NULL;
	spin_unlock_irqrestore(&rtete->queue.lock, flags);
	TRACE(rtete);

	if (rtete->spi_altera) {
		/* LY: it is a simple way to flush all pending i/o. */
		spi_w8r8(rtete->spi_altera, -1);
	}

	rtete_report_carrier(rtete);

	TRACE(rtete);
	return 0;
}

static void rtete_dev_poll(struct net_device *dev) {
	struct rtete *rtete = netdev_priv(dev);

	if (! netif_xmit_frozen_or_stopped(netdev_get_tx_queue(dev, 0)))
		rtete_iospi_rammer(rtete);

	rtete_report_carrier(rtete);
	TRACE(rtete);
}

static int rtete_dev_init(struct net_device *dev) {
	struct rtete *rtete = netdev_priv(dev);
	int err;
	TRACE(rtete);

	skb_queue_head_init(&rtete->queue);

	err = rtete_iopcm_open(rtete);
	if (err)
		goto bailout;

	err = rtete_iospi_open(rtete);
	if (err)
		goto bailout;

bailout:
	TRACE(rtete);
	return err;
}

static void rtete_dev_shutdown(struct net_device *dev) {
	struct rtete *rtete = netdev_priv(dev);
	TRACE(rtete);

	/* LY: параноя */
	rtete_dev_down(dev);

	if (rtete->spi_altera) {
		do {
			/* LY: it is a simple way to flush all pending i/o. */
			spi_w8r8(rtete->spi_altera, -1);
		} while (rtete->iospi.a->spi || rtete->iospi.b->spi);
	}

	TRACE(rtete);
}

static void rtete_dev_destroy(struct net_device *dev) {
	struct rtete *rtete = netdev_priv(dev);

	/* LY: параноя */
	rtete_dev_shutdown(dev);

	rtete_iopcm_close(rtete);
	rtete_iospi_close(rtete);

	dev_kfree_skb(rtete->rx_skb);
	dev_kfree_skb(rtete->tx_skb);

	TRACE(rtete);
	free_netdev(dev);
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
	dev->flags |= IFF_POINTOPOINT;

	/* LY: crc */
	dev->needed_tailroom += sizeof(u32);

	/* set ethernet MAC address */
	random_ether_addr(dev->dev_addr);
	dev->dev_addr[ETH_ALEN - 1] = default_mac_address[ETH_ALEN - 1];
	dev->dev_addr[ETH_ALEN - 2] = default_mac_address[ETH_ALEN - 2];
	dev->dev_addr[ETH_ALEN - 3] = default_mac_address[ETH_ALEN - 3];
}

/*-------------------------------------------------------------------*/

//static const short da850_mcbsp1_pins[] = {
//	DA850_MCBSP1_CLKR, DA850_MCBSP1_CLKX, DA850_MCBSP1_FSR,
//	DA850_MCBSP1_FSX, DA850_MCBSP1_DR, DA850_MCBSP1_DX, DA850_MCBSP1_CLKS,
//	-1
//};

int omapl138_rtbi_rtete_init(struct spi_device *spi_altera,
							 struct spi_device *spi_idt82,
							 const char* pcm_tx,
							 const char* pcm_rx,
							 struct net_device **pdev)
{
	struct net_device *dev;
	struct rtete *rtete;
	int rc;

	if (pdev)
		*pdev = NULL;

#if RTETE_SPI
	if (! spi_altera || ! spi_idt82)
		return -EINVAL;
#endif

	dev = alloc_netdev(sizeof(struct rtete), "rtete%d", rtete_setup);
	if (!dev)
		return -ENOMEM;

	rtete = netdev_priv(dev);
	memset(rtete, 0, sizeof(struct rtete));

	rtete->netdev = dev;
	rtete->spi_altera = spi_altera;
	rtete->spi_idt82 = spi_idt82;
	rtete->rx.filename = pcm_rx;
	rtete->tx.filename = pcm_tx;

	rc = register_netdev(dev);
	TRACE(rtete);
	if(rc)
		rtete_dev_destroy(dev);
	else if (pdev)
		*pdev = dev;
	return rc;
}

/*-------------------------------------------------------------------*/

static char *pcm_tx = "/dev/snd/pcmC0D0p";
module_param(pcm_tx, charp, S_IRUGO);
MODULE_PARM_DESC(pcm_tx, "Playback PCM device file name");

static char *pcm_rx = "/dev/snd/pcmC0D0c";
module_param(pcm_rx, charp, S_IRUGO);
MODULE_PARM_DESC(pcm_rx, "Capture PCM device file name");

static struct net_device *rtete_netdev;

static int __init rtete_init_module(void) {
	printk(KERN_INFO "rtete-init/loaded.\n");
	return omapl138_rtbi_rtete_init(NULL, NULL, pcm_tx, pcm_rx, &rtete_netdev);
}

static void __exit rtete_cleanup_module(void) {
	if (rtete_netdev) {
		unregister_netdev(rtete_netdev);
		rtete_netdev = NULL;
	}
	printk (KERN_INFO "rtete-exit/unloaded.\n");
}

module_init(rtete_init_module);
module_exit(rtete_cleanup_module);
MODULE_LICENSE("GPL");

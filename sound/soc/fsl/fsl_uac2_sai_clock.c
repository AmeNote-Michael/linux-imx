// SPDX-License-Identifier: GPL-2.0
/*
 * fsl_uac2_sai_clock.c - SAI bit-counter driven UAC2 async feedback
 *
 * Samples the SAI receive bit counter (RBCTN) at a fixed interval and writes
 * the measured LRCLK frequency as a pitch ratio to the "Capture Pitch 1000000"
 * ALSA control on every UAC2_Gadget card.  This lets f_uac2's isochronous
 * feedback endpoint track the hardware reference clock without depending on
 * userspace audio daemon state.
 *
 * The reference SAI is specified via the "ref-sai" device tree phandle.
 * The RX side is used because the RX pins are unused in hardware and RERE
 * is never touched by ALSA (no capture stream on the interprocessor card).
 * The default fsl_sai sync mode (synchronous[RX]=true) routes TX bit clock
 * and frame sync internally to RX, so RBCTN counts the same hardware-locked
 * BCLK as TBCTN would.  TERE must be asserted (TX running) for the bit clock
 * to reach the RX counter; when the TX is idle, delta_bits==0 and the pitch
 * write is skipped so the last valid value is preserved.
 *
 * A permanent pm_runtime reference is held on the SAI device so the
 * functional clock stays enabled and RTCTL[RSEN] is not lost across suspend.
 */
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <sound/control.h>
#include <sound/core.h>
#include "fsl_sai.h"

/* 12.288 MHz BCLK / 48 kHz LRCLK = 256 BCLK edges per LRCLK period */
#define BCLK_PER_LRCLK	256ULL
#define NOMINAL_SRATE	48000ULL
#define PITCH_UNITY	1000000ULL
#define POLL_MS		100

#define PITCH_CTL_NAME	"Capture Pitch 1000000"
#define CARD_MATCH	"UAC2_Gadget"

struct uac2_sai_clock_priv {
	struct device		*dev;
	struct device		*sai_dev;	/* get_device() + pm_runtime ref, kept until remove */
	struct fsl_sai		*sai;
	struct task_struct	*task;
	u32			 b0;		/* previous RBCTN snapshot */
	ktime_t			 t0;		/* ktime at b0 snapshot */
};

static void write_pitch_to_uac2_cards(u32 pitch)
{
	struct snd_ctl_elem_id id = {};
	int i;

	id.iface = SNDRV_CTL_ELEM_IFACE_PCM;
	strscpy(id.name, PITCH_CTL_NAME, sizeof(id.name));

	for (i = 0; i < SNDRV_CARDS; i++) {
		struct snd_card *card = snd_card_ref(i);
		struct snd_kcontrol *kctl;

		if (!card)
			continue;

		if (!strstr(card->longname, CARD_MATCH)) {
			snd_card_unref(card);
			continue;
		}

		/*
		 * Hold controls_rwsem read while calling put() so the kctl
		 * pointer stays valid for the duration of the call.
		 * u_audio_pitch_put() only writes prm->pitch and acquires no
		 * additional locks, so a read lock is sufficient.
		 */
		down_read(&card->controls_rwsem);
		kctl = snd_ctl_find_id_locked(card, &id);
		if (kctl) {
			struct snd_ctl_elem_value val = {};

			val.value.integer.value[0] = pitch;
			kctl->put(kctl, &val);
		}
		up_read(&card->controls_rwsem);

		snd_card_unref(card);
	}
}

static int uac2_sai_clock_thread(void *data)
{
	struct uac2_sai_clock_priv *priv = data;
	struct fsl_sai *sai = priv->sai;

	while (!kthread_should_stop()) {
		u32 b1, delta_bits;
		ktime_t t1;
		u64 delta_ns, lrclk_hz, pitch;

		msleep(POLL_MS);

		if (kthread_should_stop())
			break;

		regmap_read(sai->regmap, FSL_SAI_RBCTN, &b1);
		t1 = ktime_get();

		delta_bits = b1 - priv->b0;	/* u32 subtraction is wrap-safe */
		delta_ns   = ktime_to_ns(ktime_sub(t1, priv->t0));

		if (unlikely(delta_ns == 0 || delta_bits == 0))
			goto next;

		lrclk_hz = div64_u64((u64)delta_bits * NSEC_PER_SEC,
				      delta_ns * BCLK_PER_LRCLK);
		pitch    = div64_u64(lrclk_hz * PITCH_UNITY, NOMINAL_SRATE);
		pitch    = clamp_t(u64, pitch, 750000ULL, 1005000ULL);

		write_pitch_to_uac2_cards((u32)pitch);
next:
		priv->b0 = b1;
		priv->t0 = t1;
	}

	return 0;
}

static int uac2_sai_clock_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *sai_np;
	struct platform_device *sai_pdev;
	struct fsl_sai *sai;
	struct uac2_sai_clock_priv *priv;
	unsigned int ofs;
	int ret;

	sai_np = of_parse_phandle(np, "ref-sai", 0);
	if (!sai_np) {
		dev_err(dev, "missing ref-sai phandle\n");
		return -EINVAL;
	}

	sai_pdev = of_find_device_by_node(sai_np);
	of_node_put(sai_np);
	if (!sai_pdev)
		return dev_err_probe(dev, -EPROBE_DEFER, "SAI device not found\n");

	sai = dev_get_drvdata(&sai_pdev->dev);
	if (!sai) {
		put_device(&sai_pdev->dev);
		return dev_err_probe(dev, -EPROBE_DEFER, "SAI driver data not ready\n");
	}

	if (!(sai->verid.feature & FSL_SAI_VERID_TSTMP_EN)) {
		put_device(&sai_pdev->dev);
		dev_err(dev, "SAI does not support bit counter (TSTMP_EN absent)\n");
		return -ENODEV;
	}

	/*
	 * SAI is pm_runtime suspended at probe time (no active PCM stream yet).
	 * Acquire a permanent runtime reference so the functional clock stays
	 * on and RTCTL[RSEN] is never lost.  Released in remove().
	 */
	ret = pm_runtime_get_sync(&sai_pdev->dev);
	if (ret < 0) {
		put_device(&sai_pdev->dev);
		return dev_err_probe(dev, ret, "failed to wake SAI\n");
	}

	ofs = sai->soc_data->reg_offset;

	/*
	 * Enable receive timestamp counting.  RCSR[RERE] enables the receiver
	 * so RBCTN increments on each bit clock edge.  RTCTL[RSEN] arms the
	 * counter.  The default fsl_sai sync mode (synchronous[RX]=true) sets
	 * RCR2[SYNC] so RX uses the TX bit clock; no external RX pins needed.
	 * ALSA never opens a capture stream on the interprocessor card so RERE
	 * is safe to assert here permanently.
	 */
	ret = regmap_update_bits(sai->regmap, FSL_SAI_RCSR(ofs),
				 FSL_SAI_CSR_TERE, FSL_SAI_CSR_TERE);
	if (ret)
		goto err_pm;

	ret = regmap_update_bits(sai->regmap, FSL_SAI_RTCTL,
				 FSL_SAI_xTCTL_TSEN, FSL_SAI_xTCTL_TSEN);
	if (ret)
		goto err_pm;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto err_pm;
	}

	priv->dev     = dev;
	priv->sai_dev = &sai_pdev->dev;	/* keep get_device() + pm_runtime refs */
	priv->sai     = sai;
	regmap_read(sai->regmap, FSL_SAI_RBCTN, &priv->b0);
	priv->t0 = ktime_get();

	platform_set_drvdata(pdev, priv);

	priv->task = kthread_run(uac2_sai_clock_thread, priv, "uac2-sai-clock");
	if (IS_ERR(priv->task)) {
		ret = PTR_ERR(priv->task);
		goto err_pm;
	}

	dev_info(dev, "started: SAI RX bit counter -> UAC2 pitch (100 ms poll)\n");
	return 0;

err_pm:
	pm_runtime_put_sync(&sai_pdev->dev);
	put_device(&sai_pdev->dev);
	return ret;
}

static int uac2_sai_clock_remove(struct platform_device *pdev)
{
	struct uac2_sai_clock_priv *priv = platform_get_drvdata(pdev);

	kthread_stop(priv->task);
	pm_runtime_put_sync(priv->sai_dev);
	put_device(priv->sai_dev);
	return 0;
}

static const struct of_device_id uac2_sai_clock_dt_ids[] = {
	{ .compatible = "mt,uac2-sai-clock" },
	{}
};
MODULE_DEVICE_TABLE(of, uac2_sai_clock_dt_ids);

static struct platform_driver uac2_sai_clock_driver = {
	.driver = {
		.name		= "uac2-sai-clock",
		.of_match_table	= uac2_sai_clock_dt_ids,
	},
	.probe	= uac2_sai_clock_probe,
	.remove	= uac2_sai_clock_remove,
};
module_platform_driver(uac2_sai_clock_driver);

MODULE_DESCRIPTION("UAC2 SAI bit-counter async feedback driver");
MODULE_LICENSE("GPL");

// SPDX-License-Identifier: GPL-2.0
/*
 * Allied Vision Technologies GmbH Alvium camera driver
 *
 * Copyright (C) 2023 Tommaso Merciai
 * Copyright (C) 2023 Martin Hecht
 * Copyright (C) 2023 Avnet EMG GmbH
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <media/mipi-csi2.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "alvium-csi2.h"

static const struct v4l2_mbus_framefmt alvium_csi2_default_fmt = {
	.code = MEDIA_BUS_FMT_UYVY8_1X16,
	.width = 640,
	.height = 480,
	.colorspace = V4L2_COLORSPACE_SRGB,
	.ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(V4L2_COLORSPACE_SRGB),
	.quantization = V4L2_QUANTIZATION_FULL_RANGE,
	.xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(V4L2_COLORSPACE_SRGB),
	.field = V4L2_FIELD_NONE,
};

static const struct alvium_pixfmt alvium_csi2_fmts[] = {
	{
		/* UYVY8_2X8 */
		.id = ALVIUM_FMT_UYVY8_2X8,
		.code = MEDIA_BUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.fmt_av_bit = ALVIUM_BIT_YUV422_8,
		.bay_av_bit = ALVIUM_BIT_BAY_NONE,
		.mipi_fmt_regval = MIPI_CSI2_DT_YUV422_8B,
		.bay_fmt_regval = -1,
		.is_raw = 0,
	}, {
		/* UYVY8_1X16 */
		.id = ALVIUM_FMT_UYVY8_1X16,
		.code = MEDIA_BUS_FMT_UYVY8_1X16,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.fmt_av_bit = ALVIUM_BIT_YUV422_8,
		.bay_av_bit = ALVIUM_BIT_BAY_NONE,
		.mipi_fmt_regval = MIPI_CSI2_DT_YUV422_8B,
		.bay_fmt_regval = -1,
		.is_raw = 0,
	}, {
		/* YUYV8_1X16 */
		.id = ALVIUM_FMT_YUYV8_1X16,
		.code = MEDIA_BUS_FMT_YUYV8_1X16,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.fmt_av_bit = ALVIUM_BIT_YUV422_8,
		.bay_av_bit = ALVIUM_BIT_BAY_NONE,
		.mipi_fmt_regval = MIPI_CSI2_DT_YUV422_8B,
		.bay_fmt_regval = -1,
		.is_raw = 0,
	}, {
		/* YUYV8_2X8 */
		.id = ALVIUM_FMT_YUYV8_2X8,
		.code = MEDIA_BUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.fmt_av_bit = ALVIUM_BIT_YUV422_8,
		.bay_av_bit = ALVIUM_BIT_BAY_NONE,
		.mipi_fmt_regval = MIPI_CSI2_DT_YUV422_8B,
		.bay_fmt_regval = -1,
		.is_raw = 0,
	}, {
		/* YUYV10_1X20 */
		.id = ALVIUM_FMT_YUYV10_1X20,
		.code = MEDIA_BUS_FMT_YUYV10_1X20,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.fmt_av_bit = ALVIUM_BIT_YUV422_10,
		.bay_av_bit = ALVIUM_BIT_BAY_NONE,
		.mipi_fmt_regval = MIPI_CSI2_DT_YUV422_10B,
		.bay_fmt_regval = -1,
		.is_raw = 0,
	}, {
		/* RGB888_1X24 */
		.id = ALVIUM_FMT_RGB888_1X24,
		.code = MEDIA_BUS_FMT_RGB888_1X24,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.fmt_av_bit = ALVIUM_BIT_RGB888,
		.bay_av_bit = ALVIUM_BIT_BAY_NONE,
		.mipi_fmt_regval = MIPI_CSI2_DT_RGB888,
		.bay_fmt_regval = -1,
		.is_raw = 0,
	}, {
		/* RBG888_1X24 */
		.id = ALVIUM_FMT_RBG888_1X24,
		.code = MEDIA_BUS_FMT_RBG888_1X24,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.fmt_av_bit = ALVIUM_BIT_RGB888,
		.bay_av_bit = ALVIUM_BIT_BAY_NONE,
		.mipi_fmt_regval = MIPI_CSI2_DT_RGB888,
		.bay_fmt_regval = -1,
		.is_raw = 0,
	}, {
		/* BGR888_1X24 */
		.id = ALVIUM_FMT_BGR888_1X24,
		.code = MEDIA_BUS_FMT_BGR888_1X24,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.fmt_av_bit = ALVIUM_BIT_RGB888,
		.bay_av_bit = ALVIUM_BIT_BAY_NONE,
		.mipi_fmt_regval = MIPI_CSI2_DT_RGB888,
		.bay_fmt_regval = -1,
		.is_raw = 0,
	}, {
		/* RGB888_3X8 */
		.id = ALVIUM_FMT_RGB888_3X8,
		.code = MEDIA_BUS_FMT_RGB888_3X8,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.fmt_av_bit = ALVIUM_BIT_RGB888,
		.bay_av_bit = ALVIUM_BIT_BAY_NONE,
		.mipi_fmt_regval = MIPI_CSI2_DT_RGB888,
		.bay_fmt_regval = -1,
		.is_raw = 0,
	}, {
		/* Y8_1X8 */
		.id = ALVIUM_FMT_Y8_1X8,
		.code = MEDIA_BUS_FMT_Y8_1X8,
		.colorspace = V4L2_COLORSPACE_RAW,
		.fmt_av_bit = ALVIUM_BIT_RAW8,
		.bay_av_bit = ALVIUM_BIT_BAY_MONO,
		.mipi_fmt_regval = MIPI_CSI2_DT_RAW8,
		.bay_fmt_regval = 0x00,
		.is_raw = 1,
	}, {
		/* SGRBG8_1X8 */
		.id = ALVIUM_FMT_SGRBG8_1X8,
		.code = MEDIA_BUS_FMT_SGRBG8_1X8,
		.colorspace = V4L2_COLORSPACE_RAW,
		.fmt_av_bit = ALVIUM_BIT_RAW8,
		.bay_av_bit = ALVIUM_BIT_BAY_GR,
		.mipi_fmt_regval = MIPI_CSI2_DT_RAW8,
		.bay_fmt_regval = 0x01,
		.is_raw = 1,
	}, {
		/* SRGGB8_1X8 */
		.id = ALVIUM_FMT_SRGGB8_1X8,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.colorspace = V4L2_COLORSPACE_RAW,
		.fmt_av_bit = ALVIUM_BIT_RAW8,
		.bay_av_bit = ALVIUM_BIT_BAY_RG,
		.mipi_fmt_regval = MIPI_CSI2_DT_RAW8,
		.bay_fmt_regval = 0x02,
		.is_raw = 1,
	}, {
		/* SGBRG8_1X8 */
		.id = ALVIUM_FMT_SGBRG8_1X8,
		.code = MEDIA_BUS_FMT_SGBRG8_1X8,
		.colorspace = V4L2_COLORSPACE_RAW,
		.fmt_av_bit = ALVIUM_BIT_RAW8,
		.bay_av_bit = ALVIUM_BIT_BAY_GB,
		.mipi_fmt_regval = MIPI_CSI2_DT_RAW8,
		.bay_fmt_regval = 0x03,
		.is_raw = 1,
	}, {
		/* SBGGR8_1X8 */
		.id = ALVIUM_FMT_SBGGR8_1X8,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.colorspace = V4L2_COLORSPACE_RAW,
		.fmt_av_bit = ALVIUM_BIT_RAW8,
		.bay_av_bit = ALVIUM_BIT_BAY_BG,
		.mipi_fmt_regval = MIPI_CSI2_DT_RAW8,
		.bay_fmt_regval = 0x04,
		.is_raw = 1,
	}, {
		/* Y10_1X10 */
		.id = ALVIUM_FMT_Y10_1X10,
		.code = MEDIA_BUS_FMT_Y10_1X10,
		.colorspace = V4L2_COLORSPACE_RAW,
		.fmt_av_bit = ALVIUM_BIT_RAW10,
		.bay_av_bit = ALVIUM_BIT_BAY_MONO,
		.mipi_fmt_regval = MIPI_CSI2_DT_RAW10,
		.bay_fmt_regval = 0x00,
		.is_raw = 1,
	}, {
		/* SGRBG10_1X10 */
		.id = ALVIUM_FMT_SGRBG10_1X10,
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.colorspace = V4L2_COLORSPACE_RAW,
		.fmt_av_bit = ALVIUM_BIT_RAW10,
		.bay_av_bit = ALVIUM_BIT_BAY_GR,
		.mipi_fmt_regval = MIPI_CSI2_DT_RAW10,
		.bay_fmt_regval = 0x01,
		.is_raw = 1,
	}, {
		/* SRGGB10_1X10 */
		.id = ALVIUM_FMT_SRGGB10_1X10,
		.code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.colorspace = V4L2_COLORSPACE_RAW,
		.fmt_av_bit = ALVIUM_BIT_RAW10,
		.bay_av_bit = ALVIUM_BIT_BAY_RG,
		.mipi_fmt_regval = MIPI_CSI2_DT_RAW10,
		.bay_fmt_regval = 0x02,
		.is_raw = 1,
	}, {
		/* SGBRG10_1X10 */
		.id = ALVIUM_FMT_SGBRG10_1X10,
		.code = MEDIA_BUS_FMT_SGBRG10_1X10,
		.colorspace = V4L2_COLORSPACE_RAW,
		.fmt_av_bit = ALVIUM_BIT_RAW10,
		.bay_av_bit = ALVIUM_BIT_BAY_GB,
		.mipi_fmt_regval = MIPI_CSI2_DT_RAW10,
		.bay_fmt_regval = 0x03,
		.is_raw = 1,
	}, {
		/* SBGGR10_1X10 */
		.id = ALVIUM_FMT_SBGGR10_1X10,
		.code = MEDIA_BUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_RAW,
		.fmt_av_bit = ALVIUM_BIT_RAW10,
		.bay_av_bit = ALVIUM_BIT_BAY_BG,
		.mipi_fmt_regval = MIPI_CSI2_DT_RAW10,
		.bay_fmt_regval = 0x04,
		.is_raw = 1,
	}, {
		/* Y12_1X12 */
		.id = ALVIUM_FMT_Y12_1X12,
		.code = MEDIA_BUS_FMT_Y12_1X12,
		.colorspace = V4L2_COLORSPACE_RAW,
		.fmt_av_bit = ALVIUM_BIT_RAW12,
		.bay_av_bit = ALVIUM_BIT_BAY_MONO,
		.mipi_fmt_regval = MIPI_CSI2_DT_RAW12,
		.bay_fmt_regval = 0x00,
		.is_raw = 1,
	}, {
		/* SGRBG12_1X12 */
		.id = ALVIUM_FMT_SGRBG12_1X12,
		.code = MEDIA_BUS_FMT_SGRBG12_1X12,
		.colorspace = V4L2_COLORSPACE_RAW,
		.fmt_av_bit = ALVIUM_BIT_RAW12,
		.bay_av_bit = ALVIUM_BIT_BAY_GR,
		.mipi_fmt_regval = MIPI_CSI2_DT_RAW12,
		.bay_fmt_regval = 0x01,
		.is_raw = 1,
	}, {
		/* SRGGB12_1X12 */
		.id = ALVIUM_FMT_SRGGB12_1X12,
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.colorspace = V4L2_COLORSPACE_RAW,
		.fmt_av_bit = ALVIUM_BIT_RAW12,
		.bay_av_bit = ALVIUM_BIT_BAY_RG,
		.mipi_fmt_regval = MIPI_CSI2_DT_RAW12,
		.bay_fmt_regval = 0x02,
		.is_raw = 1,
	}, {
		/* SGBRG12_1X12 */
		.id = ALVIUM_FMT_SGBRG12_1X12,
		.code = MEDIA_BUS_FMT_SGBRG12_1X12,
		.colorspace = V4L2_COLORSPACE_RAW,
		.fmt_av_bit = ALVIUM_BIT_RAW12,
		.bay_av_bit = ALVIUM_BIT_BAY_GB,
		.mipi_fmt_regval = MIPI_CSI2_DT_RAW12,
		.bay_fmt_regval = 0x03,
		.is_raw = 1,
	}, {
		/* SBGGR12_1X12 */
		.id = ALVIUM_FMT_SBGGR12_1X12,
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.colorspace = V4L2_COLORSPACE_RAW,
		.fmt_av_bit = ALVIUM_BIT_RAW12,
		.bay_av_bit = ALVIUM_BIT_BAY_BG,
		.mipi_fmt_regval = MIPI_CSI2_DT_RAW12,
		.bay_fmt_regval = 0x04,
		.is_raw = 1,
	}, {
		/* SBGGR14_1X14 */
		.id = ALVIUM_FMT_SBGGR14_1X14,
		.code = MEDIA_BUS_FMT_SBGGR14_1X14,
		.colorspace = V4L2_COLORSPACE_RAW,
		.fmt_av_bit = ALVIUM_BIT_RAW14,
		.bay_av_bit = ALVIUM_BIT_BAY_GR,
		.mipi_fmt_regval = MIPI_CSI2_DT_RAW14,
		.bay_fmt_regval = 0x01,
		.is_raw = 1,
	}, {
		/* SGBRG14_1X14 */
		.id = ALVIUM_FMT_SGBRG14_1X14,
		.code = MEDIA_BUS_FMT_SGBRG14_1X14,
		.colorspace = V4L2_COLORSPACE_RAW,
		.fmt_av_bit = ALVIUM_BIT_RAW14,
		.bay_av_bit = ALVIUM_BIT_BAY_RG,
		.mipi_fmt_regval = MIPI_CSI2_DT_RAW14,
		.bay_fmt_regval = 0x02,
		.is_raw = 1,
	}, {
		/* SRGGB14_1X14 */
		.id = ALVIUM_FMT_SRGGB14_1X14,
		.code = MEDIA_BUS_FMT_SRGGB14_1X14,
		.colorspace = V4L2_COLORSPACE_RAW,
		.fmt_av_bit = ALVIUM_BIT_RAW14,
		.bay_av_bit = ALVIUM_BIT_BAY_GB,
		.mipi_fmt_regval = MIPI_CSI2_DT_RAW14,
		.bay_fmt_regval = 0x03,
		.is_raw = 1,
	}, {
		/* SGRBG14_1X14 */
		.id = ALVIUM_FMT_SGRBG14_1X14,
		.code = MEDIA_BUS_FMT_SGRBG14_1X14,
		.colorspace = V4L2_COLORSPACE_RAW,
		.fmt_av_bit = ALVIUM_BIT_RAW14,
		.bay_av_bit = ALVIUM_BIT_BAY_BG,
		.mipi_fmt_regval = MIPI_CSI2_DT_RAW14,
		.bay_fmt_regval = 0x04,
		.is_raw = 1,
	},
	{ /* sentinel */ }
};

static int alvium_read(struct alvium_dev *alvium, u32 reg, u64 *val, int *err)
{
	if (reg & REG_BCRM_V4L2) {
		reg &= ~REG_BCRM_V4L2;
		reg += alvium->bcrm_addr;
	}

	return cci_read(alvium->regmap, reg, val, err);
}

static int alvium_write(struct alvium_dev *alvium, u32 reg, u64 val, int *err)
{
	if (reg & REG_BCRM_V4L2) {
		reg &= ~REG_BCRM_V4L2;
		reg += alvium->bcrm_addr;
	}

	return cci_write(alvium->regmap, reg, val, err);
}

static int alvium_write_hshake(struct alvium_dev *alvium, u32 reg, u64 val)
{
	struct device *dev = &alvium->i2c_client->dev;
	u64 hshake_bit;
	int ret = 0;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	/* reset handshake bit and write alvium reg */
	alvium_write(alvium, REG_BCRM_WRITE_HANDSHAKE_RW, 0, &ret);
	alvium_write(alvium, reg, val, &ret);
	if (ret) {
		dev_err(dev, "Fail to write reg\n");
		return ret;
	}

	/* poll handshake bit since bit0 = 1 */
	read_poll_timeout(alvium_read, hshake_bit,
			((hshake_bit & BCRM_HANDSHAKE_W_DONE_EN_BIT) == 1),
			15000, 45000, true,
			alvium, REG_BCRM_WRITE_HANDSHAKE_RW, &hshake_bit, &ret);
	if (ret) {
		dev_err(dev, "poll bit[0] = 1, hshake reg fail\n");
		return ret;
	}

	/* reset handshake bit, write 0 to bit0 */
	alvium_write(alvium, REG_BCRM_WRITE_HANDSHAKE_RW, 0, &ret);
	if (ret) {
		dev_err(dev, "Fail to reset hshake reg\n");
		return ret;
	}

	/* poll handshake bit since bit0 = 0 */
	read_poll_timeout(alvium_read, hshake_bit,
			((hshake_bit & BCRM_HANDSHAKE_W_DONE_EN_BIT) == 0),
			15000, 45000, true,
			alvium, REG_BCRM_WRITE_HANDSHAKE_RW, &hshake_bit, &ret);
	if (ret) {
		dev_err(dev, "poll bit[0] = 0, hshake reg fail\n");
		return ret;
	}

	return 0;
}

static int alvium_get_bcrm_vers(struct alvium_dev *alvium)
{
	struct device *dev = &alvium->i2c_client->dev;
	struct alvium_bcrm_vers *v;
	u64 val;
	int ret = 0;

	alvium_read(alvium, REG_BCRM_VERSION_R, &val, &ret);
	v = (struct alvium_bcrm_vers *) &val;

	dev_info(dev, "bcrm version: %u.%u\n", v->minor, v->major);

	return ret;
}

static int alvium_get_fw_version(struct alvium_dev *alvium)
{
	struct device *dev = &alvium->i2c_client->dev;
	struct alvium_fw_vers *fw_v;
	u64 val;
	int ret = 0;

	alvium_read(alvium, REG_BCRM_DEVICE_FIRMWARE_VERSION_R, &val, &ret);
	fw_v = (struct alvium_fw_vers *) &val;

	dev_info(dev, "fw version: %u.%u.%u.%u\n", fw_v->special, fw_v->major,
			 fw_v->minor, fw_v->patch);

	return ret;
}

static int alvium_get_bcrm_addr(struct alvium_dev *alvium)
{
	u64 val;
	int ret = 0;

	alvium_read(alvium, REG_BCRM_REG_ADDR_R, &val, &ret);
	alvium->bcrm_addr = val;

	return ret;
}

static int alvium_is_alive(struct alvium_dev *alvium)
{
	u64 val = 0;
	u8 hbeat_val = 0;
	int ret = 0;

	alvium_read(alvium, REG_BCRM_VERSION_R, &val, &ret);
	alvium_read(alvium, REG_BCRM_HEARTBEAT_R, &val, &ret);

	if (ret)
		return ret;

	hbeat_val = val;

	return hbeat_val;
}

static void alvium_print_avail_mipi_fmt(struct alvium_dev *alvium)
{
	struct device *dev = &alvium->i2c_client->dev;

	dev_dbg(dev, "avail mipi_fmt yuv420_8_leg: %u\n",
			  alvium->is_mipi_fmt_avail[ALVIUM_BIT_YUV420_8_LEG]);
	dev_dbg(dev, "avail mipi_fmt yuv420_8: %u\n",
			  alvium->is_mipi_fmt_avail[ALVIUM_BIT_YUV420_8]);
	dev_dbg(dev, "avail mipi_fmt yuv420_10: %u\n",
			  alvium->is_mipi_fmt_avail[ALVIUM_BIT_YUV420_10]);
	dev_dbg(dev, "avail mipi_fmt yuv420_8_csps: %u\n",
			  alvium->is_mipi_fmt_avail[ALVIUM_BIT_YUV420_8_CSPS]);
	dev_dbg(dev, "avail mipi_fmt yuv420_10_csps: %u\n",
			  alvium->is_mipi_fmt_avail[ALVIUM_BIT_YUV420_10_CSPS]);
	dev_dbg(dev, "avail mipi_fmt yuv422_8: %u\n",
			  alvium->is_mipi_fmt_avail[ALVIUM_BIT_YUV422_8]);
	dev_dbg(dev, "avail mipi_fmt yuv422_10: %u\n",
			  alvium->is_mipi_fmt_avail[ALVIUM_BIT_YUV422_10]);
	dev_dbg(dev, "avail mipi_fmt rgb888: %u\n",
			  alvium->is_mipi_fmt_avail[ALVIUM_BIT_RGB888]);
	dev_dbg(dev, "avail mipi_fmt rgb666: %u\n",
			  alvium->is_mipi_fmt_avail[ALVIUM_BIT_RGB666]);
	dev_dbg(dev, "avail mipi_fmt rgb565: %u\n",
			  alvium->is_mipi_fmt_avail[ALVIUM_BIT_RGB565]);
	dev_dbg(dev, "avail mipi_fmt rgb555: %u\n",
			  alvium->is_mipi_fmt_avail[ALVIUM_BIT_RGB555]);
	dev_dbg(dev, "avail mipi_fmt rgb444: %u\n",
			  alvium->is_mipi_fmt_avail[ALVIUM_BIT_RGB444]);
	dev_dbg(dev, "avail mipi_fmt raw6: %u\n",
			  alvium->is_mipi_fmt_avail[ALVIUM_BIT_RAW6]);
	dev_dbg(dev, "avail mipi_fmt raw7: %u\n",
			  alvium->is_mipi_fmt_avail[ALVIUM_BIT_RAW7]);
	dev_dbg(dev, "avail mipi_fmt raw8: %u\n",
			  alvium->is_mipi_fmt_avail[ALVIUM_BIT_RAW8]);
	dev_dbg(dev, "avail mipi_fmt raw10: %u\n",
			  alvium->is_mipi_fmt_avail[ALVIUM_BIT_RAW10]);
	dev_dbg(dev, "avail mipi_fmt raw12: %u\n",
			  alvium->is_mipi_fmt_avail[ALVIUM_BIT_RAW12]);
	dev_dbg(dev, "avail mipi_fmt raw14: %u\n",
			  alvium->is_mipi_fmt_avail[ALVIUM_BIT_RAW14]);
	dev_dbg(dev, "avail mipi_fmt jpeg: %u\n",
			  alvium->is_mipi_fmt_avail[ALVIUM_BIT_JPEG]);
}

static void alvium_print_avail_feat(struct alvium_dev *alvium)
{
	struct device *dev = &alvium->i2c_client->dev;

	dev_dbg(dev, "feature rev_x: %u\n", alvium->avail_ft.rev_x);
	dev_dbg(dev, "feature rev_y: %u\n", alvium->avail_ft.rev_y);
	dev_dbg(dev, "feature int_autop: %u\n", alvium->avail_ft.int_autop);
	dev_dbg(dev, "feature black_lvl: %u\n", alvium->avail_ft.black_lvl);
	dev_dbg(dev, "feature gain: %u\n", alvium->avail_ft.gain);
	dev_dbg(dev, "feature gamma: %u\n", alvium->avail_ft.gamma);
	dev_dbg(dev, "feature contrast: %u\n", alvium->avail_ft.contrast);
	dev_dbg(dev, "feature sat: %u\n", alvium->avail_ft.sat);
	dev_dbg(dev, "feature hue: %u\n", alvium->avail_ft.hue);
	dev_dbg(dev, "feature whiteb: %u\n", alvium->avail_ft.whiteb);
	dev_dbg(dev, "feature sharp: %u\n", alvium->avail_ft.sharp);
	dev_dbg(dev, "feature auto_exp: %u\n", alvium->avail_ft.auto_exp);
	dev_dbg(dev, "feature auto_gain: %u\n", alvium->avail_ft.auto_gain);
	dev_dbg(dev, "feature auto_whiteb: %u\n", alvium->avail_ft.auto_whiteb);
	dev_dbg(dev, "feature dev_temp: %u\n", alvium->avail_ft.dev_temp);
	dev_dbg(dev, "feature acq_abort: %u\n", alvium->avail_ft.acq_abort);
	dev_dbg(dev, "feature acq_fr: %u\n", alvium->avail_ft.acq_fr);
	dev_dbg(dev, "feature fr_trigger: %u\n", alvium->avail_ft.fr_trigger);
	dev_dbg(dev, "feature exp_acq_line: %u\n", alvium->avail_ft.exp_acq_line);
}

static void alvium_print_avail_bayer(struct alvium_dev *alvium)
{
	struct device *dev = &alvium->i2c_client->dev;

	dev_dbg(dev, "avail bayer mono: %u\n",
		  alvium->is_bay_avail[ALVIUM_BIT_BAY_MONO]);
	dev_dbg(dev, "avail bayer gr: %u\n",
		  alvium->is_bay_avail[ALVIUM_BIT_BAY_GR]);
	dev_dbg(dev, "avail bayer rg: %u\n",
		  alvium->is_bay_avail[ALVIUM_BIT_BAY_RG]);
	dev_dbg(dev, "avail bayer gb: %u\n",
		  alvium->is_bay_avail[ALVIUM_BIT_BAY_GB]);
	dev_dbg(dev, "avail bayer bg: %u\n",
		  alvium->is_bay_avail[ALVIUM_BIT_BAY_BG]);
}

static int alvium_get_feat_inq(struct alvium_dev *alvium)
{
	struct alvium_avail_feat *f;
	u64 val;
	int ret = 0;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	ret = alvium_read(alvium, REG_BCRM_FEATURE_INQUIRY_R, &val, &ret);
	f = (struct alvium_avail_feat *) &val;

	alvium->avail_ft = *f;
	alvium_print_avail_feat(alvium);

	return ret;
}

static int alvium_get_host_supp_csi_lanes(struct alvium_dev *alvium)
{
	u64 val;
	int ret = 0;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	alvium_read(alvium, REG_BCRM_CSI2_LANE_COUNT_RW, &val, &ret);
	alvium->h_sup_csi_lanes = val;

	return ret;
}

static int alvium_set_csi_lanes(struct alvium_dev *alvium)
{
	struct device *dev = &alvium->i2c_client->dev;
	u64 num_lanes;
	int ret;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	num_lanes = alvium->ep.bus.mipi_csi2.num_data_lanes;

	if (num_lanes <= alvium->h_sup_csi_lanes) {
		ret = alvium_write_hshake(alvium, REG_BCRM_CSI2_LANE_COUNT_RW,
								  num_lanes);
		if (ret) {
			dev_err(dev, "Fail to set csi lanes reg\n");
			return ret;
		}
	} else {
		return -EINVAL;
	}

	return 0;
}

static int alvium_set_lp2hs_delay(struct alvium_dev *alvium)
{
	struct device *dev = &alvium->i2c_client->dev;
	u64 lp2hs_delay;
	int ret;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	/*
	 * The purpose of this reg is force a DPhy reset
	 * for the period described by the microseconds on
	 * the reg, before it starts streaming.
	 *
	 * To be clear, with that value bigger than 0 the
	 * Alvium forces a dphy-reset on all lanes for that period.
	 * That means all lanes go up into low power state.
	 *
	 * This may help a csi2 rx ip to reset if that IP
	 * can't deal with a continous clock.
	 */
	lp2hs_delay = (u64)alvium->lp2hs_delay;

	ret = alvium_write_hshake(alvium, REG_BCRM_LP2HS_DELAY_RW, lp2hs_delay);
	if (ret) {
		dev_err(dev, "Fail to set streamon delay reg\n");
		return ret;
	}

	return 0;
}

static int alvium_get_csi_clk_params(struct alvium_dev *alvium)
{
	u64 val;
	int ret = 0;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	alvium_read(alvium, REG_BCRM_CSI2_CLOCK_MIN_R, &val, &ret);
	alvium->min_csi_clk = val;

	alvium_read(alvium, REG_BCRM_CSI2_CLOCK_MAX_R, &val, &ret);
	alvium->max_csi_clk = val;

	return ret;
}

static int alvium_set_csi_clk(struct alvium_dev *alvium)
{
	struct device *dev = &alvium->i2c_client->dev;
	u64 csi_clk;
	int ret;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	csi_clk = (u32)alvium->ep.link_frequencies[0];

	if (csi_clk > alvium->min_csi_clk ||
		csi_clk < alvium->max_csi_clk) {
		ret = alvium_write_hshake(alvium, REG_BCRM_CSI2_CLOCK_RW, csi_clk);
		if (ret) {
			dev_err(dev, "Fail to set csi lanes reg\n");
			return ret;
		}
	} else {
		return -EINVAL;
	}

	alvium->link_freq = alvium->ep.link_frequencies[0];

	return 0;
}

static int alvium_get_img_width_params(struct alvium_dev *alvium)
{
	u64 val;
	int ret = 0;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	alvium_read(alvium, REG_BCRM_IMG_WIDTH_MIN_R, &val, &ret);
	alvium->img_min_width = val;

	alvium_read(alvium, REG_BCRM_IMG_WIDTH_MAX_R, &val, NULL);
	alvium->img_max_width = val;

	alvium_read(alvium, REG_BCRM_IMG_WIDTH_INC_R, &val, &ret);
	alvium->img_inc_width = val;

	return ret;
}

static int alvium_get_img_height_params(struct alvium_dev *alvium)
{
	u64 val;
	int ret = 0;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	alvium_read(alvium, REG_BCRM_IMG_HEIGHT_MIN_R, &val, &ret);
	alvium->img_min_height = val;

	alvium_read(alvium, REG_BCRM_IMG_HEIGHT_MAX_R, &val, &ret);
	alvium->img_max_height = val;

	alvium_read(alvium, REG_BCRM_IMG_HEIGHT_INC_R, &val, &ret);
	alvium->img_inc_height = val;

	return ret;
}

static int alvium_set_img_width(struct alvium_dev *alvium, u32 width)
{
	struct device *dev = &alvium->i2c_client->dev;
	int ret;

	ret = alvium_write_hshake(alvium, REG_BCRM_IMG_WIDTH_RW, width);
	if (ret) {
		dev_err(dev, "Fail to set img width\n");
		return ret;
	}

	return 0;
}

static int alvium_set_img_height(struct alvium_dev *alvium, u32 height)
{
	struct device *dev = &alvium->i2c_client->dev;
	int ret;

	ret = alvium_write_hshake(alvium, REG_BCRM_IMG_HEIGHT_RW, height);
	if (ret) {
		dev_err(dev, "Fail to set img height\n");
		return ret;
	}

	return 0;
}

static int alvium_set_img_offx(struct alvium_dev *alvium, u32 offx)
{
	struct device *dev = &alvium->i2c_client->dev;
	int ret;

	ret = alvium_write_hshake(alvium, REG_BCRM_IMG_OFFSET_X_RW, offx);
	if (ret) {
		dev_err(dev, "Fail to set img offx\n");
		return ret;
	}

	return 0;
}

static int alvium_set_img_offy(struct alvium_dev *alvium, u32 offy)
{
	struct device *dev = &alvium->i2c_client->dev;
	int ret;

	ret = alvium_write_hshake(alvium, REG_BCRM_IMG_OFFSET_Y_RW, offy);
	if (ret) {
		dev_err(dev, "Fail to set img offy\n");
		return ret;
	}

	return 0;
}

static int alvium_get_offx_params(struct alvium_dev *alvium)
{
	u64 val;
	int ret = 0;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	alvium_read(alvium, REG_BCRM_IMG_OFFSET_X_MIN_R, &val, &ret);
	alvium->min_offx = val;

	alvium_read(alvium, REG_BCRM_IMG_OFFSET_X_MAX_R, &val, &ret);
	alvium->max_offx = val;

	alvium_read(alvium, REG_BCRM_IMG_OFFSET_X_INC_R, &val, &ret);
	alvium->inc_offx = val;

	return ret;
}

static int alvium_get_offy_params(struct alvium_dev *alvium)
{
	u64 val;
	int ret;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	ret = alvium_read(alvium, REG_BCRM_IMG_OFFSET_Y_MIN_R, &val, &ret);
	alvium->min_offy = val;

	alvium_read(alvium, REG_BCRM_IMG_OFFSET_Y_MAX_R, &val, &ret);
	alvium->max_offy = val;

	alvium_read(alvium, REG_BCRM_IMG_OFFSET_Y_INC_R, &val, &ret);
	alvium->inc_offy = val;

	return ret;
}

static int alvium_get_gain_params(struct alvium_dev *alvium)
{
	u64 val;
	int ret = 0;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	alvium_read(alvium, REG_BCRM_GAIN_MIN_R, &val, &ret);
	alvium->min_gain = val;

	alvium_read(alvium, REG_BCRM_GAIN_MAX_R, &val, &ret);
	alvium->max_gain = val;

	alvium_read(alvium, REG_BCRM_GAIN_INC_R, &val, &ret);
	alvium->inc_gain = val;

	return ret;
}

static int alvium_get_exposure_params(struct alvium_dev *alvium)
{
	u64 val;
	int ret = 0;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	alvium_read(alvium, REG_BCRM_EXPOSURE_TIME_MIN_R, &val, &ret);
	alvium->min_exp = val;

	alvium_read(alvium, REG_BCRM_EXPOSURE_TIME_MAX_R, &val, &ret);
	alvium->max_exp = val;

	alvium_read(alvium, REG_BCRM_EXPOSURE_TIME_INC_R, &val, &ret);
	alvium->inc_exp = val;

	return ret;
}

static int alvium_get_red_balance_ratio_params(struct alvium_dev *alvium)
{
	u64 val;
	int ret = 0;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	alvium_read(alvium, REG_BCRM_RED_BALANCE_RATIO_MIN_R, &val, &ret);
	alvium->min_rbalance = val;

	alvium_read(alvium, REG_BCRM_RED_BALANCE_RATIO_MAX_R, &val, &ret);
	alvium->max_rbalance = val;

	alvium_read(alvium, REG_BCRM_RED_BALANCE_RATIO_INC_R, &val, &ret);
	alvium->inc_rbalance = val;

	return ret;
}

static int alvium_get_blue_balance_ratio_params(struct alvium_dev *alvium)
{
	u64 val;
	int ret = 0;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	alvium_read(alvium, REG_BCRM_BLUE_BALANCE_RATIO_MIN_R, &val, &ret);
	alvium->min_bbalance = val;

	alvium_read(alvium, REG_BCRM_BLUE_BALANCE_RATIO_MAX_R, &val, &ret);
	alvium->max_bbalance = val;

	alvium_read(alvium, REG_BCRM_BLUE_BALANCE_RATIO_INC_R, &val, &ret);
	alvium->inc_bbalance = val;

	return ret;
}

static int alvium_get_hue_params(struct alvium_dev *alvium)
{
	u64 val;
	int ret = 0;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	alvium_read(alvium, REG_BCRM_HUE_MIN_R, &val, &ret);
	alvium->min_hue = (s32)val;

	alvium_read(alvium, REG_BCRM_HUE_MAX_R, &val, &ret);
	alvium->max_hue = (s32)val;

	alvium_read(alvium, REG_BCRM_HUE_INC_R, &val, &ret);
	alvium->inc_hue = (s32)val;

	return ret;
}

static int alvium_get_black_lvl_params(struct alvium_dev *alvium)
{
	u64 val;
	int ret = 0;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	alvium_read(alvium, REG_BCRM_BLACK_LEVEL_MIN_R, &val, &ret);
	alvium->min_black_lvl = (s32)val;

	alvium_read(alvium, REG_BCRM_BLACK_LEVEL_MAX_R, &val, &ret);
	alvium->max_black_lvl = (s32)val;

	alvium_read(alvium, REG_BCRM_BLACK_LEVEL_INC_R, &val, &ret);
	alvium->inc_black_lvl = (s32)val;

	return ret;
}

static int alvium_get_gamma_params(struct alvium_dev *alvium)
{
	u64 val;
	int ret = 0;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	alvium_read(alvium, REG_BCRM_GAMMA_MIN_R, &val, &ret);
	alvium->min_gamma = val;

	alvium_read(alvium, REG_BCRM_GAMMA_MAX_R, &val, &ret);
	alvium->max_gamma = val;

	alvium_read(alvium, REG_BCRM_GAMMA_INC_R, &val, &ret);
	alvium->inc_gamma = val;

	return ret;
}

static int alvium_get_sharpness_params(struct alvium_dev *alvium)
{
	u64 val;
	int ret = 0;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	alvium_read(alvium, REG_BCRM_SHARPNESS_MIN_R, &val, &ret);
	alvium->min_sharp = (s32)val;

	alvium_read(alvium, REG_BCRM_BLACK_LEVEL_MAX_R, &val, &ret);
	alvium->max_sharp = (s32)val;

	alvium_read(alvium, REG_BCRM_SHARPNESS_INC_R, &val, &ret);
	alvium->inc_sharp = (s32)val;

	return ret;
}

static int alvium_get_contrast_params(struct alvium_dev *alvium)
{
	u64 val;
	int ret = 0;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	alvium_read(alvium, REG_BCRM_CONTRAST_VALUE_MIN_R, &val, &ret);
	alvium->min_contrast = val;

	alvium_read(alvium, REG_BCRM_CONTRAST_VALUE_MAX_R, &val, &ret);
	alvium->max_contrast = val;

	alvium_read(alvium, REG_BCRM_CONTRAST_VALUE_INC_R, &val, &ret);
	alvium->inc_contrast = val;

	return ret;
}

static int alvium_get_saturation_params(struct alvium_dev *alvium)
{
	u64 val;
	int ret = 0;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	alvium_read(alvium, REG_BCRM_SATURATION_MIN_R, &val, &ret);
	alvium->min_sat = val;

	alvium_read(alvium, REG_BCRM_SATURATION_MAX_R, &val, &ret);
	alvium->max_sat = val;

	alvium_read(alvium, REG_BCRM_SATURATION_INC_R, &val, &ret);
	alvium->inc_sat = val;

	return ret;
}

static int alvium_set_bcm_mode(struct alvium_dev *alvium)
{
	int ret = 0;

	alvium_write(alvium, REG_GENCP_CHANGEMODE_W, ALVIUM_BCM_MODE, &ret);
	alvium->bcrm_mode = ALVIUM_BCM_MODE;

	return ret;
}

static int alvium_get_mode(struct alvium_dev *alvium)
{
	u64 bcrm_mode;
	int ret = 0;

	alvium_read(alvium, REG_GENCP_CURRENTMODE_R, &bcrm_mode, &ret);

	switch (bcrm_mode) {
	case ALVIUM_BCM_MODE:
		alvium->bcrm_mode = ALVIUM_BCM_MODE;
		break;
	case ALVIUM_GENCP_MODE:
		alvium->bcrm_mode = ALVIUM_GENCP_MODE;
		break;
	}

	return ret;
}

static int alvium_get_avail_mipi_data_format(struct alvium_dev *alvium)
{
	struct alvium_avail_mipi_fmt *avail_fmt;
	u64 val;
	int ret = 0;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	alvium_read(alvium, REG_BCRM_IMG_AVAILABLE_MIPI_DATA_FORMATS_R, &val, &ret);
	avail_fmt = (struct alvium_avail_mipi_fmt *) &val;

	alvium->is_mipi_fmt_avail[ALVIUM_BIT_YUV420_8_LEG]
				  = avail_fmt->yuv420_8_leg;
	alvium->is_mipi_fmt_avail[ALVIUM_BIT_YUV420_8]
				  = avail_fmt->yuv420_8;
	alvium->is_mipi_fmt_avail[ALVIUM_BIT_YUV420_10]
				  = avail_fmt->yuv420_10;
	alvium->is_mipi_fmt_avail[ALVIUM_BIT_YUV420_8_CSPS]
				  = avail_fmt->yuv420_8_csps;
	alvium->is_mipi_fmt_avail[ALVIUM_BIT_YUV420_10_CSPS]
				  = avail_fmt->yuv420_10_csps;
	alvium->is_mipi_fmt_avail[ALVIUM_BIT_YUV422_8]
				  = avail_fmt->yuv422_8;
	alvium->is_mipi_fmt_avail[ALVIUM_BIT_YUV422_10]
				  = avail_fmt->yuv422_10;
	alvium->is_mipi_fmt_avail[ALVIUM_BIT_RGB888]
				  = avail_fmt->rgb888;
	alvium->is_mipi_fmt_avail[ALVIUM_BIT_RGB666]
				  = avail_fmt->rgb666;
	alvium->is_mipi_fmt_avail[ALVIUM_BIT_RGB565]
				  = avail_fmt->rgb565;
	alvium->is_mipi_fmt_avail[ALVIUM_BIT_RGB555]
				  = avail_fmt->rgb555;
	alvium->is_mipi_fmt_avail[ALVIUM_BIT_RGB444]
				  = avail_fmt->rgb444;
	alvium->is_mipi_fmt_avail[ALVIUM_BIT_RAW6]
				  = avail_fmt->raw6;
	alvium->is_mipi_fmt_avail[ALVIUM_BIT_RAW7]
				  = avail_fmt->raw7;
	alvium->is_mipi_fmt_avail[ALVIUM_BIT_RAW8]
				  = avail_fmt->raw8;
	alvium->is_mipi_fmt_avail[ALVIUM_BIT_RAW10]
				  = avail_fmt->raw10;
	alvium->is_mipi_fmt_avail[ALVIUM_BIT_RAW12]
				  = avail_fmt->raw12;
	alvium->is_mipi_fmt_avail[ALVIUM_BIT_RAW14]
				  = avail_fmt->raw14;
	alvium->is_mipi_fmt_avail[ALVIUM_BIT_JPEG]
				  = avail_fmt->jpeg;

	alvium_print_avail_mipi_fmt(alvium);

	return ret;
}

static int alvium_setup_mipi_fmt(struct alvium_dev *alvium)
{
	int avail_fmt_cnt = 0;
	int sz = 0;

	alvium->alvium_csi2_fmt = NULL;

	/* calculate fmt array size */
	for (int fmt = 0; fmt < ALVIUM_NUM_SUPP_MIPI_DATA_FMT; fmt++) {
		if (alvium->is_mipi_fmt_avail[alvium_csi2_fmts[fmt].fmt_av_bit])
			if ((!alvium_csi2_fmts[fmt].is_raw) ||
				  (alvium->is_bay_avail[alvium_csi2_fmts[fmt].bay_av_bit]))
				sz++;
	}

	/* init alvium_csi2_fmt array */
	alvium->alvium_csi2_fmt_n = sz;
	alvium->alvium_csi2_fmt = kmalloc_array(sz,
						     sizeof(struct alvium_pixfmt),
						     GFP_KERNEL);

	/* Create the alvium_csi2 fmt array from formats available */
	for (int fmt = 0; fmt < ALVIUM_NUM_SUPP_MIPI_DATA_FMT; fmt++) {
		if (alvium->is_mipi_fmt_avail[alvium_csi2_fmts[fmt].fmt_av_bit])
			if ((!alvium_csi2_fmts[fmt].is_raw) ||
				  (alvium->is_bay_avail[alvium_csi2_fmts[fmt].bay_av_bit])) {
				alvium->alvium_csi2_fmt[avail_fmt_cnt] =
						alvium_csi2_fmts[fmt];
				avail_fmt_cnt++;
			}
	}

	return 0;
}

static int alvium_set_mipi_fmt(struct alvium_dev *alvium,
			       const struct alvium_pixfmt *pixfmt)
{
	struct device *dev = &alvium->i2c_client->dev;
	u64 reg_val;
	int ret;

	reg_val = pixfmt->mipi_fmt_regval;

	ret = alvium_write_hshake(alvium, REG_BCRM_IMG_MIPI_DATA_FORMAT_RW,
							  reg_val);
	if (ret) {
		dev_err(dev, "Fail to set mipi fmt\n");
		return ret;
	}

	return 0;
}

static int alvium_get_avail_bayer(struct alvium_dev *alvium)
{
	struct alvium_avail_bayer *avail_bay;
	u64 val;
	int ret = 0;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	alvium_read(alvium, REG_BCRM_IMG_BAYER_PATTERN_INQUIRY_R, &val, &ret);
	avail_bay = (struct alvium_avail_bayer *) &val;

	alvium->is_bay_avail[ALVIUM_BIT_BAY_MONO] = avail_bay->mono;
	alvium->is_bay_avail[ALVIUM_BIT_BAY_GR] = avail_bay->gr;
	alvium->is_bay_avail[ALVIUM_BIT_BAY_RG] = avail_bay->rg;
	alvium->is_bay_avail[ALVIUM_BIT_BAY_GB] = avail_bay->gb;
	alvium->is_bay_avail[ALVIUM_BIT_BAY_BG] = avail_bay->bg;

	alvium_print_avail_bayer(alvium);

	return ret;
}

static int alvium_set_bayer_pattern(struct alvium_dev *alvium,
				    const struct alvium_pixfmt *pixfmt)
{
	struct device *dev = &alvium->i2c_client->dev;
	u64 reg_val;
	int ret;

	reg_val = pixfmt->bay_fmt_regval;

	ret = alvium_write_hshake(alvium, REG_BCRM_IMG_BAYER_PATTERN_RW, reg_val);
	if (ret) {
		dev_err(dev, "Fail to set bayer pattern\n");
		return ret;
	}

	return 0;
}

static int alvium_get_frame_interval(struct alvium_dev *alvium)
{
	u64 val;
	int ret = 0;

	alvium_read(alvium, REG_BCRM_ACQUISITION_FRAME_RATE_MIN_R, &val, &ret);
	alvium->min_fr = val;

	alvium_read(alvium, REG_BCRM_ACQUISITION_FRAME_RATE_MAX_R, &val, &ret);
	alvium->max_fr = val;

	return 0;
}

static int alvium_set_frame_rate(struct alvium_dev *alvium)
{
	struct device *dev = &alvium->i2c_client->dev;
	u64 fr;
	int ret;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	fr = alvium->fr;

	ret = alvium_write_hshake(alvium, REG_BCRM_ACQUISITION_FRAME_RATE_RW, fr);
	if (ret) {
		dev_err(dev, "Fail to set frame rate lanes reg\n");
		return ret;
	}

	dev_dbg(dev, "set frame rate: %llu us\n", fr);

	return 0;
}

static int alvium_set_stream_mipi(struct alvium_dev *alvium, bool on)
{
	struct device *dev = &alvium->i2c_client->dev;
	int ret;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	if (on)
		ret = alvium_write_hshake(alvium, REG_BCRM_ACQUISITION_START_RW, 0x01);
	else
		ret = alvium_write_hshake(alvium, REG_BCRM_ACQUISITION_STOP_RW, 0x01);

	if (ret) {
		dev_err(dev, "Fail set_stream_mipi\n");
		return ret;
	}

	return 0;
}

static int alvium_get_gain(struct alvium_dev *alvium)
{
	u64 gain;
	int ret = 0;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	/* The unit is millibel (1 mB = 0.01 dB) */
	alvium_read(alvium, REG_BCRM_GAIN_RW, &gain, &ret);
	if (ret)
		return ret;

	return gain;
}

static int alvium_set_gain(struct alvium_dev *alvium, int gain)
{
	struct device *dev = &alvium->i2c_client->dev;
	int ret;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	/* The unit is millibel (1 mB = 0.01 dB) */
	ret = alvium_write_hshake(alvium, REG_BCRM_GAIN_RW, (u64)gain);
	if (ret) {
		dev_err(dev, "Fail to set gain value reg\n");
		return ret;
	}

	return 0;
}

static int alvium_set_autogain(struct alvium_dev *alvium, bool on)
{
	struct device *dev = &alvium->i2c_client->dev;
	int ret;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	if (!alvium->avail_ft.auto_gain)
		on = false;

	if (on)
		ret = alvium_write_hshake(alvium, REG_BCRM_GAIN_AUTO_RW, 0x02);
	else
		ret = alvium_write_hshake(alvium, REG_BCRM_GAIN_AUTO_RW, 0x00);

	if (ret) {
		dev_err(dev, "Fail to set autogain reg\n");
		return ret;
	}

	return 0;
}

static int alvium_get_exposure(struct alvium_dev *alvium)
{
	u64 exp;
	int ret = 0;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	/* Exposure time in ns */
	alvium_read(alvium, REG_BCRM_EXPOSURE_TIME_RW, &exp, &ret);
	if (ret)
		return ret;

	return exp;
}

static int alvium_set_autoexposure(struct alvium_dev *alvium, bool on)
{
	struct device *dev = &alvium->i2c_client->dev;
	int ret;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	if (!alvium->avail_ft.auto_exp)
		on = false;

	if (on)
		ret = alvium_write_hshake(alvium, REG_BCRM_EXPOSURE_AUTO_RW, 0x02);
	else
		ret = alvium_write_hshake(alvium, REG_BCRM_EXPOSURE_AUTO_RW, 0x00);

	if (ret) {
		dev_err(dev, "Fail to set autoexposure reg\n");
		return ret;
	}

	return 0;
}

static int alvium_set_exposure(struct alvium_dev *alvium, int exposure_ns)
{
	struct device *dev = &alvium->i2c_client->dev;
	int ret;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	ret = alvium_write_hshake(alvium, REG_BCRM_EXPOSURE_TIME_RW,
							  (u64)exposure_ns);
	if (ret) {
		dev_err(dev, "Fail to set exposure value reg\n");
		return ret;
	}

	return 0;
}

static int alvium_set_ctrl_gain(struct alvium_dev *alvium, bool auto_gain)
{
	struct alvium_ctrls *ctrls = &alvium->ctrls;
	int ret = 0;

	if (ctrls->auto_gain->is_new) {
		ret = alvium_set_autogain(alvium, auto_gain);
		if (ret)
			return ret;
	}

	if (!auto_gain && ctrls->gain->is_new) {
		u64 max_gain;
		u64 min_gain;

		ret = alvium_get_gain_params(alvium);
		if (ret)
			return ret;

		max_gain = alvium->max_gain;
		min_gain = alvium->min_gain;

		if (ctrls->gain->val >= min_gain &&
			  ctrls->gain->val <= max_gain) {
			ret = alvium_set_gain(alvium, ctrls->gain->val);
		} else {
			return -EINVAL;
		}
	}

	return ret;
}

static int alvium_set_blue_balance_ratio(struct alvium_dev *alvium, int blue)
{
	struct device *dev = &alvium->i2c_client->dev;
	int ret;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	ret = alvium_write_hshake(alvium, REG_BCRM_BLUE_BALANCE_RATIO_RW,
							  (u64)blue);
	if (ret) {
		dev_err(dev, "Fail to set blue ratio value reg\n");
		return ret;
	}

	return 0;
}

static int alvium_set_red_balance_ratio(struct alvium_dev *alvium, int red)
{
	struct device *dev = &alvium->i2c_client->dev;
	int ret;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	ret = alvium_write_hshake(alvium, REG_BCRM_RED_BALANCE_RATIO_RW, (u64)red);
	if (ret) {
		dev_err(dev, "Fail to set red ratio value reg\n");
		return ret;
	}

	return 0;
}

static int alvium_set_awb(struct alvium_dev *alvium, bool on)
{
	struct device *dev = &alvium->i2c_client->dev;
	int ret;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	if (!alvium->avail_ft.auto_whiteb)
		on = false;

	if (on)
		ret = alvium_write_hshake(alvium, REG_BCRM_WHITE_BALANCE_AUTO_RW, 0x02);
	else
		ret = alvium_write_hshake(alvium, REG_BCRM_WHITE_BALANCE_AUTO_RW, 0x00);

	if (ret) {
		dev_err(dev, "Fail to set awb reg\n");
		return ret;
	}

	return 0;
}

static int alvium_set_ctrl_exposure(struct alvium_dev *alvium,
				    enum v4l2_exposure_auto_type auto_exposure)
{
	struct alvium_ctrls *ctrls = &alvium->ctrls;
	bool auto_exp = (auto_exposure == V4L2_EXPOSURE_AUTO);
	int ret = 0;

	if (ctrls->auto_exp->is_new) {
		ret = alvium_set_autoexposure(alvium, auto_exp);
		if (ret)
			return ret;
	}

	if (!auto_exp && ctrls->exposure->is_new) {
		u64 max_exp;
		u64 min_exp;

		ret = alvium_get_exposure_params(alvium);
		if (ret)
			return ret;

		max_exp = alvium->max_exp;
		min_exp = alvium->min_exp;

		if (ctrls->exposure->val >= min_exp &&
			  ctrls->exposure->val <= max_exp) {
			ret = alvium_set_exposure(alvium, ctrls->exposure->val);
			if (ret)
				return ret;
		} else {
			return -EINVAL;
		}
	}

	return ret;
}

static int alvium_set_ctrl_white_balance(struct alvium_dev *alvium, bool awb)
{
	struct alvium_ctrls *ctrls = &alvium->ctrls;
	int ret = 0;

	if (ctrls->auto_wb->is_new) {
		ret = alvium_set_awb(alvium, awb);
		if (ret)
			return ret;
	}

	if (!awb && (ctrls->blue_balance->is_new)) {
		u64 blue = (u64)ctrls->blue_balance->val;
		u64 blue_max;
		u64 blue_min;

		ret = alvium_get_blue_balance_ratio_params(alvium);
		if (ret)
			return ret;

		blue_max = alvium->max_bbalance;
		blue_min = alvium->min_bbalance;

		if (blue >= blue_min &&
			  blue <= blue_max) {
			ret = alvium_set_blue_balance_ratio(alvium, blue);
			if (ret)
				return ret;
		} else {
			return -EINVAL;
		};
	}

	if (!awb && (ctrls->red_balance->is_new)) {
		u64 red = (u64)ctrls->red_balance->val;
		u64 red_max;
		u64 red_min;

		ret = alvium_get_red_balance_ratio_params(alvium);
		if (ret)
			return ret;

		red_max = alvium->max_rbalance;
		red_min = alvium->min_rbalance;

		if (red >= red_min &&
			  red <= red_max) {
			ret = alvium_set_red_balance_ratio(alvium, red);
			if (ret)
				return ret;
		} else {
			return -EINVAL;
		}
	}

	return ret;
}

static int alvium_set_ctrl_hue(struct alvium_dev *alvium, int val)
{
	struct device *dev = &alvium->i2c_client->dev;
	int ret;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	ret = alvium_write_hshake(alvium, REG_BCRM_HUE_RW, (u64)val);
	if (ret) {
		dev_err(dev, "Fail to set hue value reg\n");
		return ret;
	}

	return 0;
}

static int alvium_set_ctrl_contrast(struct alvium_dev *alvium, int val)
{
	struct device *dev = &alvium->i2c_client->dev;
	int ret;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	ret = alvium_write_hshake(alvium, REG_BCRM_CONTRAST_VALUE_RW, (u64)val);
	if (ret) {
		dev_err(dev, "Fail to set contrast value reg\n");
		return ret;
	}

	return 0;
}

static int alvium_set_ctrl_saturation(struct alvium_dev *alvium, int val)
{
	struct device *dev = &alvium->i2c_client->dev;
	int ret;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	ret = alvium_write_hshake(alvium, REG_BCRM_SATURATION_RW, (u64)val);
	if (ret) {
		dev_err(dev, "Fail to set contrast value reg\n");
		return ret;
	}

	return 0;
}

static int alvium_set_ctrl_gamma(struct alvium_dev *alvium, int val)
{
	struct device *dev = &alvium->i2c_client->dev;
	int ret;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	ret = alvium_write_hshake(alvium, REG_BCRM_GAMMA_RW, (u64)val);
	if (ret) {
		dev_err(dev, "Fail to set gamma value reg\n");
		return ret;
	}

	return 0;
}

static int alvium_set_ctrl_sharpness(struct alvium_dev *alvium, int val)
{
	struct device *dev = &alvium->i2c_client->dev;
	int ret;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	ret = alvium_write_hshake(alvium, REG_BCRM_SHARPNESS_RW, (u64)val);
	if (ret) {
		dev_err(dev, "Fail to set sharpness value reg\n");
		return ret;
	}

	return 0;
}

static int alvium_set_ctrl_hflip(struct alvium_dev *alvium, int val)
{
	struct device *dev = &alvium->i2c_client->dev;
	int ret;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	ret = alvium_write_hshake(alvium, REG_BCRM_IMG_REVERSE_X_RW, (u64)val);
	if (ret) {
		dev_err(dev, "Fail to set reverse_x value reg\n");
		return ret;
	}

	return 0;
}

static int alvium_set_ctrl_vflip(struct alvium_dev *alvium, int val)
{
	struct device *dev = &alvium->i2c_client->dev;
	int ret;

	if (!alvium->bcrm_addr)
		return -EINVAL;

	ret = alvium_write_hshake(alvium, REG_BCRM_IMG_REVERSE_Y_RW, (u64)val);
	if (ret) {
		dev_err(dev, "Fail to set reverse_y value reg\n");
		return ret;
	}

	return 0;
}

static int alvium_get_hw_features_params(struct alvium_dev *alvium)
{
	struct device *dev = &alvium->i2c_client->dev;
	int ret;

	ret = alvium_get_csi_clk_params(alvium);
	if (ret) {
		dev_err(dev, "Fail to read min/max csi clock regs\n");
		return ret;
	}

	ret = alvium_get_img_width_params(alvium);
	if (ret) {
		dev_err(dev, "Fail to read img width regs\n");
		return ret;
	}

	ret = alvium_get_img_height_params(alvium);
	if (ret) {
		dev_err(dev, "Fail to read img heigth regs\n");
		return ret;
	}

	ret = alvium_get_offx_params(alvium);
	if (ret) {
		dev_err(dev, "Fail to read offx regs\n");
		return ret;
	}

	ret = alvium_get_offy_params(alvium);
	if (ret) {
		dev_err(dev, "Fail to read offy regs\n");
		return ret;
	}

	ret = alvium_get_gain_params(alvium);
	if (ret) {
		dev_err(dev, "Fail to read gain regs\n");
		return ret;
	}

	ret = alvium_get_exposure_params(alvium);
	if (ret) {
		dev_err(dev, "Fail to read min/max exp regs\n");
		return ret;
	}

	ret = alvium_get_red_balance_ratio_params(alvium);
	if (ret) {
		dev_err(dev, "Fail to read red balance ratio regs\n");
		return ret;
	}

	ret = alvium_get_blue_balance_ratio_params(alvium);
	if (ret) {
		dev_err(dev, "Fail to read blue balance ratio regs\n");
		return ret;
	}

	ret = alvium_get_hue_params(alvium);
	if (ret) {
		dev_err(dev, "Fail to read hue regs\n");
		return ret;
	}

	ret = alvium_get_contrast_params(alvium);
	if (ret) {
		dev_err(dev, "Fail to read contrast regs\n");
		return ret;
	}

	ret = alvium_get_saturation_params(alvium);
	if (ret) {
		dev_err(dev, "Fail to read saturation regs\n");
		return ret;
	}

	ret = alvium_get_black_lvl_params(alvium);
	if (ret) {
		dev_err(dev, "Fail to read black lvl regs\n");
		return ret;
	}

	ret = alvium_get_gamma_params(alvium);
	if (ret) {
		dev_err(dev, "Fail to read gamma regs\n");
		return ret;
	}

	ret = alvium_get_sharpness_params(alvium);
	if (ret) {
		dev_err(dev, "Fail to read sharpness regs\n");
		return ret;
	}

	return 0;
}

static int alvium_get_hw_info(struct alvium_dev *alvium)
{
	struct device *dev = &alvium->i2c_client->dev;
	int ret;

	ret = alvium_get_bcrm_vers(alvium);
	if (ret) {
		dev_err(dev, "Fail to read bcrm version reg\n");
		return ret;
	}

	ret = alvium_get_bcrm_addr(alvium);
	if (ret) {
		dev_err(dev, "Fail to bcrm address reg\n");
		return ret;
	}

	ret = alvium_get_fw_version(alvium);
	if (ret) {
		dev_err(dev, "Fail to read fw version reg\n");
		return ret;
	}

	ret = alvium_get_host_supp_csi_lanes(alvium);
	if (ret) {
		dev_err(dev, "Fail to read host supported csi lanes reg\n");
		return ret;
	}

	ret = alvium_get_feat_inq(alvium);
	if (ret) {
		dev_err(dev, "Fail to read bcrm feature inquiry reg\n");
		return ret;
	}

	ret = alvium_get_hw_features_params(alvium);
	if (ret) {
		dev_err(dev, "Fail to read features params regs\n");
		return ret;
	}

	ret = alvium_get_avail_mipi_data_format(alvium);
	if (ret) {
		dev_err(dev, "Fail to read available mipi data formats reg\n");
		return ret;
	}

	ret = alvium_get_avail_bayer(alvium);
	if (ret) {
		dev_err(dev, "Fail to read available Bayer patterns reg\n");
		return ret;
	}

	ret = alvium_get_mode(alvium);
	if (ret) {
		dev_err(dev, "Fail to get current mode reg\n");
		return ret;
	}

	return 0;
}

static int alvium_hw_init(struct alvium_dev *alvium)
{
	struct device *dev = &alvium->i2c_client->dev;
	int ret;

	/* Set Alvium BCM mode*/
	ret = alvium_set_bcm_mode(alvium);
	if (ret) {
		dev_err(dev, "Fail to set BCM mode\n");
		return ret;
	}

	ret = alvium_set_csi_lanes(alvium);
	if (ret) {
		dev_err(dev, "Fail to set csi lanes\n");
		return ret;
	}

	ret = alvium_set_csi_clk(alvium);
	if (ret) {
		dev_err(dev, "Fail to set csi lanes\n");
		return ret;
	}

	ret = alvium_set_lp2hs_delay(alvium);
	if (ret) {
		dev_err(dev, "Fail to set streamon reg\n");
		return ret;
	}

	return 0;
}

/* --------------- Subdev Operations --------------- */

static int alvium_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct alvium_dev *alvium = sd_to_alvium(sd);

	fi->interval = alvium->frame_interval;

	return 0;
}

static int alvium_set_frame_interval(struct alvium_dev *alvium,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct device *dev = &alvium->i2c_client->dev;
	u64 req_fr, min_fr, max_fr;
	int ret;

	if (fi->interval.denominator == 0)
		return -EINVAL;

	ret = alvium_get_frame_interval(alvium);
	if (ret) {
		dev_err(dev, "Fail to get frame interval\n");
		return ret;
	}

	min_fr = alvium->min_fr;
	max_fr = alvium->max_fr;

	dev_dbg(dev, "fi->interval.numerator = %d\n", fi->interval.numerator);
	dev_dbg(dev, "fi->interval.denominator = %d\n", fi->interval.denominator);

	req_fr = (fi->interval.denominator * USEC_PER_SEC) /
				  fi->interval.numerator;

	if (req_fr >= max_fr &&
		  req_fr <= min_fr)
		return -EINVAL;

	alvium->fr = req_fr;
	alvium->frame_interval.numerator = fi->interval.numerator;
	alvium->frame_interval.denominator = fi->interval.denominator;

	return 0;
}

static int alvium_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct alvium_dev *alvium = sd_to_alvium(sd);
	int ret;

	if (alvium->streaming)
		return -EBUSY;

	ret = alvium_set_frame_interval(alvium, fi);
	if (!ret) {
		ret = alvium_set_frame_rate(alvium);
		if (ret)
			return -EIO;
	}

	return ret;
}

static int alvium_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct alvium_dev *alvium = sd_to_alvium(sd);
	const struct alvium_pixfmt *formats;
	unsigned int num_formats;

	formats = alvium->alvium_csi2_fmt;
	num_formats = alvium->alvium_csi2_fmt_n;

	if (code->index >= num_formats)
		return -EINVAL;

	code->code = formats[code->index].code;

	return 0;
}

static const struct alvium_pixfmt *
alvium_code_to_pixfmt(struct alvium_dev *alvium, u32 code)
{
	const struct alvium_pixfmt *formats = alvium->alvium_csi2_fmt;
	unsigned int i;

	for (i = 0; formats[i].code; ++i)
		if (formats[i].code == code)
			return &formats[i];

	return &formats[0];
}

static int alvium_set_mode(struct alvium_dev *alvium,
			   struct v4l2_subdev_state *state)
{
	struct device *dev = &alvium->i2c_client->dev;
	struct v4l2_subdev *sd = &alvium->sd;
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_rect *crop;
	int ret;

	crop = v4l2_subdev_get_pad_crop(sd, state, 0);
	fmt = v4l2_subdev_get_pad_format(sd, state, 0);

	if (fmt->width < alvium->img_max_width &&
		   fmt->width > alvium->img_min_width) {
		ret = alvium_set_img_width(alvium, fmt->width);
		if (ret)
			return ret;
	} else {
		dev_err(dev, "Unsupported width");
		return -EINVAL;
	}

	if (fmt->height < alvium->img_max_height &&
		   fmt->height > alvium->img_min_height) {
		ret = alvium_set_img_height(alvium, fmt->height);
		if (ret)
			return ret;
	} else {
		dev_err(dev, "Unsupported height");
		return -EINVAL;
	}

	ret = alvium_set_img_offx(alvium, crop->left);
	if (ret)
		return ret;

	ret = alvium_set_img_offy(alvium, crop->top);
	if (ret)
		return ret;

	return 0;
}

static int alvium_set_framefmt(struct alvium_dev *alvium,
			       struct v4l2_mbus_framefmt *format)
{
	struct device *dev = &alvium->i2c_client->dev;
	const struct alvium_pixfmt *alvium_csi2_fmt;
	int ret = 0;

	alvium_csi2_fmt = alvium_code_to_pixfmt(alvium, format->code);

	ret = alvium_set_mipi_fmt(alvium, alvium_csi2_fmt);
	if (ret)
		return ret;

	if (alvium_csi2_fmt->is_raw) {
		ret = alvium_set_bayer_pattern(alvium, alvium_csi2_fmt);
		if (ret)
			return ret;
	}

	dev_dbg(dev, "start: %s, mipi_fmt_regval regval = 0x%llx",
	  __func__, alvium_csi2_fmt->mipi_fmt_regval);

	return ret;
}

static int alvium_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct alvium_dev *alvium = sd_to_alvium(sd);
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_subdev_state *state;
	int ret = 0;

	state = v4l2_subdev_lock_and_get_active_state(sd);

	if (enable) {
		ret = __v4l2_ctrl_handler_setup(&alvium->ctrls.handler);
		if (ret)
			goto out;

		ret = alvium_set_mode(alvium, state);
		if (ret)
			goto out;

		fmt = v4l2_subdev_get_pad_format(sd, state, 0);
		ret = alvium_set_framefmt(alvium, fmt);
		if (ret)
			goto out;

		ret = alvium_set_stream_mipi(alvium, enable);
		if (ret)
			goto out;

	} else {
		ret = alvium_set_stream_mipi(alvium, enable);
		if (ret)
			goto out;
	}

	v4l2_subdev_unlock_state(state);
	alvium->streaming = !!enable;

	return 0;

out:
	v4l2_subdev_unlock_state(state);
	return ret;
}

static int alvium_init_cfg(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state)
{
	struct alvium_dev *alvium = sd_to_alvium(sd);
	struct alvium_mode *mode = &alvium->mode;
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_rect *crop;

	struct v4l2_subdev_format sd_fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.format = alvium_csi2_default_fmt,
	};

	struct v4l2_subdev_crop sd_crop = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.rect = {
			.left = mode->crop.left,
			.top = mode->crop.top,
			.width = mode->crop.width,
			.height = mode->crop.height,
		},
	};

	crop = v4l2_subdev_get_pad_crop(sd, state, 0);
	fmt = v4l2_subdev_get_pad_format(sd, state, 0);

	*crop = sd_crop.rect;
	*fmt = sd_fmt.format;

	return 0;
}

static int alvium_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *format)
{
	struct alvium_dev *alvium = sd_to_alvium(sd);
	const struct alvium_pixfmt *alvium_csi2_fmt;
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_rect *crop;

	fmt = v4l2_subdev_get_pad_format(sd, sd_state, 0);
	crop = v4l2_subdev_get_pad_crop(sd, sd_state, 0);

	if (format->format.width <= alvium->img_max_width &&
		  format->format.width >= alvium->img_min_width) {
		fmt->width = format->format.width;
	} else {
		return -EINVAL;
	}

	if (format->format.height <= alvium->img_max_height &&
		  format->format.height >= alvium->img_min_height) {
		fmt->height = format->format.height;
	} else {
		return -EINVAL;
	}

	alvium_csi2_fmt = alvium_code_to_pixfmt(alvium, format->format.code);
	fmt->code = alvium_csi2_fmt->code;

	/* Set also the crop width and height when set a new fmt */
	crop->width = fmt->width;
	crop->height = fmt->height;

	*fmt = format->format;

	return 0;
}

static int alvium_set_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	struct alvium_dev *alvium = sd_to_alvium(sd);
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_rect *crop;

	crop = v4l2_subdev_get_pad_crop(sd, sd_state, 0);
	fmt = v4l2_subdev_get_pad_format(sd, sd_state, 0);

	/*
	 * Alvium can only shift the origin of the img
	 * then we accept only value with the same value of the actual fmt
	 */
	if (sel->r.width != fmt->width ||
		  sel->r.height != fmt->height)
		return -EINVAL;

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	if ((sel->r.left + sel->r.width) <= alvium->img_max_width)
		crop->left = sel->r.left;
	else
		return -EINVAL;

	if ((sel->r.top + sel->r.height) <= alvium->img_max_height)
		crop->top = sel->r.top;
	else
		return -EINVAL;

	crop->width = sel->r.width;
	crop->height = sel->r.height;

	sel->r = *crop;

	return 0;
}

static int alvium_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	struct alvium_dev *alvium = sd_to_alvium(sd);

	switch (sel->target) {
	/* Current cropping area */
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_get_pad_crop(sd, sd_state, 0);
		break;
	/* Cropping bounds */
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = alvium->img_max_width;
		sel->r.height = alvium->img_max_height;
		break;
	/* Default cropping area */
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.top = alvium->min_offy;
		sel->r.left = alvium->min_offx;
		sel->r.width = alvium->img_max_width;
		sel->r.height = alvium->img_max_height;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int alvium_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct alvium_dev *alvium = sd_to_alvium(sd);
	int val;

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		val = alvium_get_gain(alvium);
		if (val < 0)
			return val;
		alvium->ctrls.gain->val = val;
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		val = alvium_get_exposure(alvium);
		if (val < 0)
			return val;
		alvium->ctrls.exposure->val = val;
		break;
	}

	return 0;
}

static int alvium_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct alvium_dev *alvium = sd_to_alvium(sd);
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		ret = alvium_set_ctrl_gain(alvium, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret = alvium_set_ctrl_exposure(alvium, ctrl->val);
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret = alvium_set_ctrl_white_balance(alvium, ctrl->val);
		break;
	case V4L2_CID_HUE:
		ret = alvium_set_ctrl_hue(alvium, ctrl->val);
		break;
	case V4L2_CID_CONTRAST:
		ret = alvium_set_ctrl_contrast(alvium, ctrl->val);
		break;
	case V4L2_CID_SATURATION:
		ret = alvium_set_ctrl_saturation(alvium, ctrl->val);
		break;
	case V4L2_CID_GAMMA:
		ret = alvium_set_ctrl_gamma(alvium, ctrl->val);
		break;
	case V4L2_CID_SHARPNESS:
		ret = alvium_set_ctrl_sharpness(alvium, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = alvium_set_ctrl_hflip(alvium, ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		ret = alvium_set_ctrl_vflip(alvium, ctrl->val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops alvium_ctrl_ops = {
	.g_volatile_ctrl = alvium_g_volatile_ctrl,
	.s_ctrl = alvium_s_ctrl,
};

static int alvium_ctrl_init(struct alvium_dev *alvium)
{
	const struct v4l2_ctrl_ops *ops = &alvium_ctrl_ops;
	struct alvium_ctrls *ctrls = &alvium->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	struct v4l2_fwnode_device_properties props;
	int ret;

	v4l2_ctrl_handler_init(hdl, 32);

	/* Pixel rate is fixed */
	ctrls->pixel_rate = v4l2_ctrl_new_std(hdl, ops,
					     V4L2_CID_PIXEL_RATE, 0,
					     ALVIUM_DEFAULT_PIXEL_RATE_MHZ, 1,
					     ALVIUM_DEFAULT_PIXEL_RATE_MHZ);

	/* Link freq is fixed */
	ctrls->link_freq = v4l2_ctrl_new_int_menu(hdl, ops,
					     V4L2_CID_LINK_FREQ,
					     0, 0, &alvium->link_freq);

	/* Auto/manual white balance */
	ctrls->auto_wb = v4l2_ctrl_new_std(hdl, ops,
						V4L2_CID_AUTO_WHITE_BALANCE,
						0, 1, 1,
						alvium->avail_ft.auto_whiteb ? 1 : 0);

	ctrls->blue_balance = v4l2_ctrl_new_std(hdl, ops,
					     V4L2_CID_BLUE_BALANCE,
					     alvium->min_bbalance,
					     alvium->max_bbalance,
					     alvium->inc_bbalance,
					     alvium->min_bbalance);
	ctrls->red_balance = v4l2_ctrl_new_std(hdl, ops,
					     V4L2_CID_RED_BALANCE,
					     alvium->min_rbalance,
					     alvium->max_rbalance,
					     alvium->inc_rbalance,
					     alvium->min_rbalance);

	/* Auto/manual exposure */
	ctrls->auto_exp = v4l2_ctrl_new_std_menu(hdl, ops,
						V4L2_CID_EXPOSURE_AUTO,
						V4L2_EXPOSURE_MANUAL, 0,
						alvium->avail_ft.auto_exp ?
						V4L2_EXPOSURE_AUTO : V4L2_EXPOSURE_MANUAL);

	ctrls->exposure = v4l2_ctrl_new_std(hdl, ops,
					     V4L2_CID_EXPOSURE,
					     alvium->min_exp,
					     alvium->max_exp,
					     alvium->inc_exp,
					     alvium->min_exp);

	/* Auto/manual gain */
	ctrls->auto_gain = v4l2_ctrl_new_std(hdl, ops,
						V4L2_CID_AUTOGAIN,
						0, 1, 1,
						alvium->avail_ft.auto_gain ? 1 : 0);

	if (alvium->avail_ft.gain)
		ctrls->gain = v4l2_ctrl_new_std(hdl, ops,
					     V4L2_CID_GAIN,
					     alvium->min_gain,
					     alvium->max_gain,
					     alvium->inc_gain,
					     alvium->min_gain);

	if (alvium->avail_ft.sat)
		ctrls->saturation = v4l2_ctrl_new_std(hdl, ops,
					     V4L2_CID_SATURATION,
					     alvium->min_sat,
					     alvium->max_sat,
					     alvium->inc_sat,
					     alvium->max_sat);

	if (alvium->avail_ft.hue)
		ctrls->hue = v4l2_ctrl_new_std(hdl, ops,
					     V4L2_CID_HUE,
					     alvium->min_hue,
					     alvium->max_hue,
					     alvium->inc_hue,
					     alvium->min_hue);

	if (alvium->avail_ft.contrast)
		ctrls->contrast = v4l2_ctrl_new_std(hdl, ops,
					     V4L2_CID_CONTRAST,
					     alvium->min_contrast,
					     alvium->max_contrast,
					     alvium->inc_contrast,
					     alvium->min_contrast);

	if (alvium->avail_ft.gamma)
		ctrls->gamma = v4l2_ctrl_new_std(hdl, ops,
					     V4L2_CID_GAMMA,
					     alvium->min_gamma,
					     alvium->max_gamma,
					     alvium->inc_gamma,
					     alvium->min_gamma);

	if (alvium->avail_ft.sharp)
		ctrls->sharpness = v4l2_ctrl_new_std(hdl, ops,
					     V4L2_CID_SHARPNESS,
					     alvium->min_sharp,
					     alvium->max_sharp,
					     alvium->inc_sharp,
					     alvium->min_sharp);

	if (alvium->avail_ft.rev_x)
		ctrls->hflip = v4l2_ctrl_new_std(hdl, ops,
					     V4L2_CID_HFLIP,
					     0, 1, 1, 0);

	if (alvium->avail_ft.rev_y)
		ctrls->vflip = v4l2_ctrl_new_std(hdl, ops,
					     V4L2_CID_VFLIP,
					     0, 1, 1, 0);

	if (hdl->error) {
		ret = hdl->error;
		goto free_ctrls;
	}

	ret = v4l2_fwnode_device_parse(&alvium->i2c_client->dev, &props);
	if (ret)
		goto free_ctrls;

	ret = v4l2_ctrl_new_fwnode_properties(hdl, ops, &props);
	if (ret)
		goto free_ctrls;

	ctrls->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	ctrls->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	ctrls->gain->flags |= V4L2_CTRL_FLAG_VOLATILE;
	ctrls->exposure->flags |= V4L2_CTRL_FLAG_VOLATILE;

	v4l2_ctrl_auto_cluster(3, &ctrls->auto_wb, 0, false);
	v4l2_ctrl_auto_cluster(2, &ctrls->auto_gain, 0, true);
	v4l2_ctrl_auto_cluster(2, &ctrls->auto_exp, 1, true);

	alvium->sd.ctrl_handler = hdl;
	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static int alvium_s_power(struct v4l2_subdev *sd, int on)
{
	return 0;
}

static const struct v4l2_subdev_core_ops alvium_core_ops = {
	.s_power = alvium_s_power,
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops alvium_video_ops = {
	.g_frame_interval	= alvium_g_frame_interval,
	.s_frame_interval	= alvium_s_frame_interval,
	.s_stream		= alvium_s_stream,
};

static const struct v4l2_subdev_pad_ops alvium_pad_ops = {
	.init_cfg = alvium_init_cfg,
	.enum_mbus_code = alvium_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = alvium_set_fmt,
	.get_selection = alvium_get_selection,
	.set_selection = alvium_set_selection,
};

static const struct v4l2_subdev_ops alvium_subdev_ops = {
	.core	= &alvium_core_ops,
	.pad	= &alvium_pad_ops,
	.video	= &alvium_video_ops,
};

static int alvium_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	return 0;
}


static const struct media_entity_operations alvium_sd_media_ops = {
	.link_setup = alvium_link_setup,
};

static int alvium_subdev_init(struct alvium_dev *alvium)
{
	struct i2c_client *client = alvium->i2c_client;
	struct device *dev = &alvium->i2c_client->dev;
	struct v4l2_subdev *sd = &alvium->sd;
	int ret;

	/* Setup initial frame interval*/
	alvium->frame_interval.numerator = 1;
	alvium->frame_interval.denominator = ALVIUM_DEFAULT_FR_HZ;
	alvium->fr = ALVIUM_DEFAULT_FR_HZ;

	/* Setup the initial mode */
	alvium->mode.fmt = alvium_csi2_default_fmt;
	alvium->mode.width = alvium_csi2_default_fmt.width;
	alvium->mode.height = alvium_csi2_default_fmt.height;
	alvium->mode.crop.left = alvium->min_offx;
	alvium->mode.crop.top = alvium->min_offy;
	alvium->mode.crop.width = alvium_csi2_default_fmt.width;
	alvium->mode.crop.height = alvium_csi2_default_fmt.height;

	/* init alvium sd */
	v4l2_i2c_subdev_init(sd, client, &alvium_subdev_ops);

	sd->flags |= V4L2_SUBDEV_FL_HAS_EVENTS | V4L2_SUBDEV_FL_HAS_DEVNODE;
	alvium->sd.entity.ops = &alvium_sd_media_ops;
	alvium->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ret = media_entity_pads_init(&sd->entity, 1, &alvium->pad);
	if (ret) {
		dev_err(dev, "Could not register media entity\n");
		return ret;
	}

	ret = alvium_ctrl_init(alvium);
	if (ret) {
		dev_err(dev, "Control initialization error %d\n", ret);
		goto entity_cleanup;
	}

	alvium->sd.state_lock = alvium->ctrls.handler.lock;

	ret = v4l2_subdev_init_finalize(sd);
	if (ret < 0) {
		dev_err(dev, "subdev initialization error %d\n", ret);
		goto err_ctrls;
	}

	return 0;

err_ctrls:
	v4l2_ctrl_handler_free(&alvium->ctrls.handler);
entity_cleanup:
	media_entity_cleanup(&alvium->sd.entity);
	return ret;
}

static void alvium_subdev_cleanup(struct alvium_dev *alvium)
{
	v4l2_subdev_cleanup(&alvium->sd);
	media_entity_cleanup(&alvium->sd.entity);
	v4l2_ctrl_handler_free(&alvium->ctrls.handler);
}

static int alvium_get_dt_data(struct alvium_dev *alvium)
{
	struct device *dev = &alvium->i2c_client->dev;
	struct fwnode_handle *fwnode = dev_fwnode(dev);
	struct fwnode_handle *endpoint;

	if (!fwnode)
		return -EINVAL;

	/* Only CSI2 is supported for now: */
	alvium->ep.bus_type = V4L2_MBUS_CSI2_DPHY;

	fwnode_property_read_u32(fwnode, "alliedvision,lp2hs-delay-us",
							  &alvium->lp2hs_delay);

	endpoint = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &alvium->ep)) {
		dev_err(dev, "could not parse endpoint\n");
		return 0;
	}

	if (alvium->ep.bus_type != V4L2_MBUS_CSI2_DPHY) {
		dev_err(dev, "unsupported bus type\n");
		return -EINVAL;
	}

	if (!alvium->ep.nr_of_link_frequencies) {
		dev_err(dev, "no link frequencies defined");
		return -EINVAL;
	}

	dev_dbg(dev, "freq: %llu\n", alvium->ep.link_frequencies[0]);
	dev_dbg(dev, "lanes: %d\n", alvium->ep.bus.mipi_csi2.num_data_lanes);

	return 0;
}

static int alvium_power_on(struct alvium_dev *alvium)
{
	struct device *dev = &alvium->i2c_client->dev;
	int ret = 0;

	if (regulator_is_enabled(alvium->reg_vcc)) {
		/* vcc-ext-in reg is already enabled */
		dev_dbg(dev, "vcc-ext-in reg already enabled");
	} else {
		ret = regulator_enable(alvium->reg_vcc);
		if (ret)
			return ret;

		/* alvium boot time 7s*/
		msleep(7000);
	}

	return 0;
}

static int alvium_power_off(struct alvium_dev *alvium)
{
	int ret = 0;

	ret = regulator_disable(alvium->reg_vcc);
	if (ret)
		return ret;

	return 0;
}

static int alvium_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct alvium_dev *alvium = sd_to_alvium(sd);
	int ret;

	ret = alvium_power_on(alvium);
	if (ret)
		return ret;

	ret = alvium_hw_init(alvium);
	if (ret)
		return ret;

	return 0;
}

static int alvium_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct alvium_dev *alvium = sd_to_alvium(sd);

	alvium_power_off(alvium);

	return 0;
}

static const struct dev_pm_ops alvium_pm_ops = {
	RUNTIME_PM_OPS(alvium_runtime_suspend, alvium_runtime_resume, NULL)
};

static int alvium_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct alvium_dev *alvium;
	int ret;

	alvium = devm_kzalloc(dev, sizeof(*alvium), GFP_KERNEL);
	if (!alvium)
		return -ENOMEM;

	alvium->i2c_client = client;

	alvium->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(alvium->regmap))
		return PTR_ERR(alvium->regmap);

	ret = alvium_get_dt_data(alvium);
	if (ret)
		return ret;

	alvium->reg_vcc = devm_regulator_get_optional(dev, "vcc-ext-in");
	if (IS_ERR(alvium->reg_vcc))
		return dev_err_probe(dev, PTR_ERR(alvium->reg_vcc),
			"no vcc-ext-in regulator provided\n");

	ret = alvium_power_on(alvium);
	if (ret)
		return ret;

	if (!alvium_is_alive(alvium)) {
		dev_err(dev, "Device detection failed: %d\n", ret);
		return -ENODEV;
	}

	ret = alvium_get_hw_info(alvium);
	if (ret) {
		dev_err(dev, "get_hw_info fail %d\n", ret);
		return -EPROBE_DEFER;
	}

	ret = alvium_hw_init(alvium);
	if (ret) {
		dev_err(dev, "hw_init fail %d\n", ret);
		return ret;
	}

	ret = alvium_setup_mipi_fmt(alvium);
	if (ret) {
		dev_err(dev, "setup_mipi_fmt fail %d\n", ret);
		return ret;
	}

	/*
	 * Enable runtime PM without autosuspend:
	 *
	 * Don't use pm autosuspend (alvium have ~7s boot time).
	 * Alvium has been powered manually:
	 *  - mark it as active
	 *  - increase the usage count without resuming the device.
	 */
	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);

	/* Initialize the V4L2 subdev. */
	ret = alvium_subdev_init(alvium);
	if (ret)
		goto err_pm;

	ret = v4l2_async_register_subdev(&alvium->sd);
	if (ret < 0) {
		dev_err(dev, "Could not register v4l2 device\n");
		goto err_subdev;
	}

	return 0;

err_subdev:
	alvium_subdev_cleanup(alvium);
err_pm:
	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
	alvium_power_off(alvium);
	return ret;
}

static void alvium_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct alvium_dev *alvium = sd_to_alvium(sd);
	struct device *dev = &alvium->i2c_client->dev;

	i2c_unregister_device(alvium->i2c_client);

	/*
	 * Disable runtime PM. In case runtime PM is disabled in the kernel,
	 * make sure to turn power off manually.
	 */
	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev))
		alvium_power_off(alvium);
	pm_runtime_set_suspended(dev);
}

static const struct i2c_device_id alvium_id[] = {
	{"alvium-csi2", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, alvium_id);

static const struct of_device_id alvium_of_ids[] = {
	{ .compatible = "alliedvision,alvium-csi2", },
	{ }
};
MODULE_DEVICE_TABLE(of, alvium_of_ids);

static struct i2c_driver alvium_i2c_driver = {
	.probe	= alvium_probe,
	.remove		= alvium_remove,
	.id_table = alvium_id,
	.driver	= {
		.name	= "alvium-csi2",
		.pm = pm_ptr(&alvium_pm_ops),
		.of_match_table = alvium_of_ids,
	},
};

module_i2c_driver(alvium_i2c_driver);

MODULE_DESCRIPTION("Allied Vision's Alvium Camera Driver");
MODULE_AUTHOR("Tommaso Merciai <tomm.merciai@gmail.com>");
MODULE_AUTHOR("Martin Hecht <martin.hecht@avnet.eu>");
MODULE_AUTHOR("Avnet Silica Software & Services EMEA");
MODULE_LICENSE("GPL v2");

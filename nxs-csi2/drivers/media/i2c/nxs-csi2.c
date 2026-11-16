// SPDX-License-Identifier: GPL-2.0-only
/*
 * nxs-csi2.c - V4L2 sub-device driver for a camera door of the NXS Device Interface
 *
 * Copyright (C) 2026 Aliensense
 *
 * The driver knows no sensor. A pod, an NXS Camera or any device of the
 * interface serves a camera door, and nxs-csi2-core reads its header and its
 * description, commits its rows and offers the rows no standard control maps
 * as custom controls. This adapter is the sub-device: its pad ops offer the
 * mode table as formats, the standard controls exposure, vertical blanking,
 * analogue gain and the flips stand on their rows, and the stream start parks
 * and gates the sensor. The contract is the NXS Device Interface manual
 * (redbrain constants/nxs_map.yaml), and the driver's sequence is
 * docs/bring-up/nxs-csi2-mainline.md in the redbrain tree.
 *
 * The line is the door's row of id LINE_TIME_NS, the line the camera runs in
 * ns, whatever the device names it. PIXEL_RATE is the mode's lane rate on its
 * lanes over the bits of one pixel on the wire, its data type's, and HBLANK
 * makes up the line at that rate. The exposure control counts lines of that
 * line, the exposure row's microseconds converted, and the vertical blanking
 * is the frame the rate row names, in lines, less the mode's height. A door
 * that serves no such row runs the mode's active pixels at the pixel rate,
 * HBLANK 0.
 *
 * A camera door whose node carries aliensense,modes, the camera's declared
 * mode, binds whether or not a pod answers at it, so a port's capture chain
 * binds with the links it has. While no pod answers, or the map serves no
 * mode yet, the door reports the declared mode, offers no row control and
 * fails its stream start with ENODEV. The map is read once a pod answers, at
 * the first format ask or stream start that finds it. A door without the
 * property waits for its map: its probe defers.
 *
 * A door whose description changed since it was read, its DESCRIPTOR_EPOCH
 * moved by another personality uploaded into the pod behind it, is read
 * again at the next format ask or stream start that meets it: the formats
 * become the new description's, and each row control follows a row of its id
 * and shape. A control whose row left the door or changed its shape turns
 * inactive, and a row the door gained takes its control at the next bind. No
 * control keeps a pointer into the rows across the read: HFLIP and VFLIP both
 * leave the FLIPS row and come back to it.
 *
 * A node with the property aliensense,passive is a camera whose stream
 * reaches this host while another host owns the bus the map answers on. The
 * driver then transfers nothing: its formats come from aliensense,modes, and
 * a stream start touches nothing. An active format set selects the first
 * mode of the asked size and code and takes that mode's rows: every row
 * control's range and activation follow, the core's among them. A size and
 * code no mode has keeps the mode, a try reports the mode in effect, and
 * another mode while the node streams is EBUSY. Its rows come from
 * aliensense,rows, the rows of the mode in effect, and take the controls a
 * door's rows take, converted to lines alike. A set stores the value, and
 * the control framework raises V4L2_EVENT_CTRL to every subscriber when it
 * changes, so a camera engine on this host hears what the host's ISP sets.
 * A passive node without the property offers no row control.
 *
 * A passive node whose tree names the exposure and the line rows has an
 * owner's side: three controls in one cluster, the line in effect in ns
 * and the exposure's minimum and maximum in us. The program that runs the
 * camera paints them from the camera's cells, in a call of their own. A
 * set takes the line as the line in effect and the bounds as
 * the exposure row's range, then repaints the ranges as a door's refresh
 * does. A try or a set of a minimum above the maximum is EINVAL, and the
 * line runs from the mode's shortest. A format set that selects another
 * mode returns that mode's tree rows until the owner paints again.
 *
 * A node with the property aliensense,deserializer sits behind a
 * deserializer, whose output is the endpoint: every mode rides the output's
 * lanes at its one link frequency, whatever the mode names for its sensor.
 *
 * A node with the property aliensense,monochrome is a camera whose sensor
 * carries no colour filter array: every mode streams on its monochrome code,
 * whatever Bayer order the map reports, where one personality serves both
 * parts of a sensor.
 */

#include <linux/bitops.h>
#include <linux/i2c.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/time64.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "nxs-csi2-core.h"

/* The cells of one mode of a passive node's aliensense,modes property. */
enum nxs_dt_cell {
	NXS_DT_WIDTH,
	NXS_DT_HEIGHT,
	NXS_DT_BIT_DEPTH,
	NXS_DT_PIXEL_PHASE,
	NXS_DT_RATE_MBPS,
	NXS_DT_LANES,
	NXS_DT_CELLS,
};

/* The cells of one row of a mode in a passive node's aliensense,rows property. */
enum nxs_dt_row_cell {
	NXS_DT_ROW_MODE,
	NXS_DT_ROW_ID,
	NXS_DT_ROW_MIN,
	NXS_DT_ROW_MAX,
	NXS_DT_ROW_STEP,
	NXS_DT_ROW_DEFAULT,
	NXS_DT_ROW_CELLS,
};

/* The rows aliensense,rows carries at most: eight for each mode of the table. */
#define NXS_DT_ROWS_MAX	(NXS_MODE_TABLE_MAX * 8)

/* The pixel phase of a mode with no Bayer order. */
#define NXS_PHASE_MONO 0xff

/* A passive node's owner controls, in their cluster's order. */
enum nxs_owner_ctrl {
	NXS_OWNER_LINE,
	NXS_OWNER_EXPOSURE_MIN,
	NXS_OWNER_EXPOSURE_MAX,
	NXS_OWNER_CTRLS,
};

/* nxs_ensure_map() while no pod answers at the door. */
#define NXS_DOOR_EMPTY 1

/* A mode the endpoint carries, and the link frequency it rides. */
struct nxs_fmt {
	const struct nxs_csi2_mode *mode;
	unsigned int link_freq;
};

struct nxs_csi2 {
	struct i2c_client *client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	/* The control handler's lock and the sub-device state's: the map, the rows, the commit. */
	struct mutex lock;

	struct v4l2_ctrl_handler ctrls;
	/*
	 * Clusters: one commit carries the exposure and the gain. The frame
	 * period commits alone, since the exposure's range follows it.
	 */
	struct v4l2_ctrl *apply[2];
	struct v4l2_ctrl *flips[2];
	/* A passive node's owner controls, one cluster; NULL where it has none. */
	struct v4l2_ctrl *owner[NXS_OWNER_CTRLS];
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;

	/* The endpoint. */
	u8 lanes;
	u64 *link_freqs;
	unsigned int nr_link_freqs;
	/* The endpoint is a deserializer's output, which every mode rides. */
	bool deserializer;
	/* The sensor carries no colour filter array. */
	bool monochrome;

	struct nxs_csi2_core core;
	/* The door's row of id LINE_TIME_NS, -1 when it serves none. */
	int line_row;
	/* The line in effect in ns, the unit of EXPOSURE and VBLANK. */
	u32 line_ns;
	/* A passive node: the modes are the tree's, and the bus is another host's. */
	bool passive;
	/* The tree's modes: a passive node's, or a door's declared mode while no pod answers. */
	struct nxs_csi2_mode dt_modes[NXS_MODE_TABLE_MAX];
	unsigned int nr_dt_modes;
	/* A passive node's aliensense,rows, NXS_DT_ROW_CELLS cells per row of a mode. */
	u32 *dt_rows;
	unsigned int nr_dt_rows;

	/* The modes the endpoint carries, as formats. */
	struct nxs_fmt fmts[NXS_MODE_TABLE_MAX];
	unsigned int nr_fmts;
	const struct nxs_fmt *cur;

	/* LIVE or a range written into the controls: no commit. */
	bool syncing;
	bool streaming;
};

static const struct v4l2_ctrl_ops nxs_csi2_ctrl_ops;

static inline struct nxs_csi2 *to_nxs_csi2(struct v4l2_subdev *sd)
{
	return container_of(sd, struct nxs_csi2, sd);
}

/* ---- the modes and the formats --------------------------------------- */

/* The flips the controls hold, or the flips a set brings. */
static u8 nxs_flips(const struct nxs_csi2 *priv, bool cur)
{
	u8 flips = 0;

	if (priv->hflip && (cur ? priv->hflip->cur.val : priv->hflip->val))
		flips |= NXS_FLIP_X;
	if (priv->vflip && (cur ? priv->vflip->cur.val : priv->vflip->val))
		flips |= NXS_FLIP_Y;

	return flips;
}

/* The media bus code of a mode under the flips; a monochrome sensor's has no Bayer order. */
static u32 nxs_code(const struct nxs_csi2 *priv, const struct nxs_csi2_mode *mode, u8 flips)
{
	struct nxs_csi2_mode mono;

	if (!priv->monochrome)
		return nxs_csi2_core_mbus_code(mode, flips);
	mono = *mode;
	mono.phase = NXS_PHASE_MONO;

	return nxs_csi2_core_mbus_code(&mono, 0);
}

/* The DT link frequency a lane rate rides: the D-PHY data rate is twice the link frequency. */
static int nxs_link_freq_index(const struct nxs_csi2 *priv, u16 rate_mbps)
{
	u64 freq = (u64)rate_mbps * 1000000 / 2;
	unsigned int i;

	for (i = 0; i < priv->nr_link_freqs; i++)
		if (priv->link_freqs[i] == freq)
			return i;

	return -1;
}

/*
 * One format per mode the endpoint carries; a mode it cannot carry is logged.
 * Behind a deserializer the mode's lanes and rate are the sensor's, on the
 * serial link: the receiver sees the deserializer's output, so every mode
 * rides the endpoint's lanes at its one link frequency. A table with no mode
 * the endpoint carries leaves the formats as they were.
 */
static int nxs_add_fmts(struct nxs_csi2 *priv, const struct nxs_csi2_mode *modes, unsigned int n)
{
	struct device *dev = &priv->client->dev;
	unsigned int count = 0;
	unsigned int i;

	for (i = 0; i < n; i++) {
		const struct nxs_csi2_mode *mode = &modes[i];
		int freq;

		if (!mode->width || !mode->height || !mode->lanes || !mode->rate_mbps ||
		    !nxs_code(priv, mode, 0)) {
			dev_warn(dev, "mode %u: %ux%u at %u bits, pixel phase %u, %u lanes at %u Mbps, not offered\n",
				 mode->value, mode->width, mode->height, mode->bit_depth,
				 mode->phase, mode->lanes, mode->rate_mbps);
			continue;
		}
		if (priv->deserializer) {
			priv->fmts[count].mode = mode;
			priv->fmts[count].link_freq = 0;
			count++;
			continue;
		}
		if (mode->lanes != priv->lanes) {
			dev_warn(dev, "mode %u: %u lanes, the endpoint has %u, not offered\n",
				 mode->value, mode->lanes, priv->lanes);
			continue;
		}
		freq = nxs_link_freq_index(priv, mode->rate_mbps);
		if (freq < 0) {
			dev_warn(dev, "mode %u: %u Mbps per lane needs link-frequencies %llu, not offered\n",
				 mode->value, mode->rate_mbps, (u64)mode->rate_mbps * 1000000 / 2);
			continue;
		}
		priv->fmts[count].mode = mode;
		priv->fmts[count].link_freq = freq;
		count++;
	}
	if (!count) {
		dev_err(dev, "no mode the endpoint carries\n");
		return -ENODEV;
	}
	priv->nr_fmts = count;

	return 0;
}

/* The offered format a MODE_VALUE selects, NULL for none. */
static const struct nxs_fmt *nxs_find_fmt(const struct nxs_csi2 *priv, u8 value)
{
	unsigned int i;

	for (i = 0; i < priv->nr_fmts; i++)
		if (priv->fmts[i].mode->value == value)
			return &priv->fmts[i];

	return NULL;
}

/* The first offered format of @mbus's size and code under the flips in effect, NULL for none. */
static const struct nxs_fmt *nxs_match_fmt(const struct nxs_csi2 *priv,
					   const struct v4l2_mbus_framefmt *mbus)
{
	unsigned int i;

	for (i = 0; i < priv->nr_fmts; i++) {
		const struct nxs_csi2_mode *mode = priv->fmts[i].mode;

		if (mode->width == mbus->width && mode->height == mbus->height &&
		    nxs_code(priv, mode, nxs_flips(priv, true)) == mbus->code)
			return &priv->fmts[i];
	}

	return NULL;
}

/* The offered format whose mode the table marks default, else the first. */
static const struct nxs_fmt *nxs_default_fmt(const struct nxs_csi2 *priv)
{
	unsigned int i;

	for (i = 0; i < priv->nr_fmts; i++)
		if (priv->fmts[i].mode->flags & NXS_MODE_FLAG_DEFAULT)
			return &priv->fmts[i];

	return &priv->fmts[0];
}

/*
 * The mode's pixels per second: its lane rate on its lanes over the bits of
 * one pixel on the wire. An offered mode's data type is a RAW code, which
 * names its pixel bits.
 */
static u64 nxs_pixel_rate(const struct nxs_csi2_mode *mode)
{
	return div_u64((u64)mode->rate_mbps * 1000000 * mode->lanes,
		       nxs_csi2_data_type_pixel_bits(mode->data_type));
}

/* The mode's shortest line in ns: its active pixels at its pixel rate, rounded up. */
static u64 nxs_shortest_ns(const struct nxs_csi2 *priv)
{
	const struct nxs_csi2_mode *mode = priv->cur->mode;

	return DIV64_U64_ROUND_UP((u64)mode->width * NSEC_PER_SEC, nxs_pixel_rate(mode));
}

/*
 * The line in effect in ns: the door's LINE_TIME_NS row as the last refresh
 * read it, its declared default while the map is INERT or the row has no
 * reading yet. A door that serves none, a passive node and a door no pod
 * answers at run the mode's active pixels at its pixel rate, and no line is
 * shorter than that.
 */
static u32 nxs_line_ns(const struct nxs_csi2 *priv)
{
	u64 shortest = nxs_shortest_ns(priv);
	u32 line = 0;

	if (priv->line_row >= 0) {
		const struct nxs_csi2_row *row = &priv->core.rows[priv->line_row];

		line = row->live && !nxs_csi2_row_unread(row) ? row->live : row->def;
	}

	return max_t(u64, line, min_t(u64, shortest, U32_MAX));
}

/*
 * The line in effect into priv->line_ns. A passive node's line row is a view
 * of it: the row's minimum, maximum and default become that line, and so
 * does its value where the mode in effect lists the row. A row the mode does
 * not list keeps no reading, so its control stays inactive. A door's line
 * row is the device's and is left as the refresh read it.
 */
static void nxs_take_line(struct nxs_csi2 *priv)
{
	struct nxs_csi2_row *row;

	priv->line_ns = nxs_line_ns(priv);
	if (!priv->passive || priv->line_row < 0)
		return;
	row = &priv->core.rows[priv->line_row];
	row->range[NXS_CSI2_MIN] = priv->line_ns;
	row->range[NXS_CSI2_MAX] = priv->line_ns;
	row->def = priv->line_ns;
	if (!nxs_csi2_row_unread(row))
		row->live = priv->line_ns;
}

/*
 * The horizontal blanking that makes up the line in effect at the mode's
 * pixel rate, at most S32_MAX: HBLANK is a 32-bit control.
 */
static s64 nxs_hblank(const struct nxs_csi2 *priv)
{
	const struct nxs_csi2_mode *mode = priv->cur->mode;
	u64 pixels = div64_u64((u64)priv->line_ns * nxs_pixel_rate(mode) + NSEC_PER_SEC / 2,
			       NSEC_PER_SEC);

	return pixels > mode->width ? min_t(u64, pixels - mode->width, S32_MAX) : 0;
}

/*
 * The pixel array the modes read out of: the widest and the tallest mode the
 * endpoint carries, since the map serves no array of its own.
 */
static struct v4l2_rect nxs_pixel_array(const struct nxs_csi2 *priv)
{
	struct v4l2_rect array = { };
	unsigned int i;

	for (i = 0; i < priv->nr_fmts; i++) {
		array.width = max_t(u32, array.width, priv->fmts[i].mode->width);
		array.height = max_t(u32, array.height, priv->fmts[i].mode->height);
	}

	return array;
}

static void nxs_fill_fmt(const struct nxs_csi2 *priv, const struct nxs_fmt *fmt,
			 struct v4l2_mbus_framefmt *mbus)
{
	mbus->width = fmt->mode->width;
	mbus->height = fmt->mode->height;
	mbus->code = nxs_code(priv, fmt->mode, nxs_flips(priv, true));
	mbus->field = V4L2_FIELD_NONE;
	mbus->colorspace = V4L2_COLORSPACE_RAW;
	mbus->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	mbus->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	mbus->xfer_func = V4L2_XFER_FUNC_NONE;
}

/* The RAW data type of a sample depth, 0 for a depth no RAW code carries. */
static u8 nxs_raw_data_type(u32 depth)
{
	switch (depth) {
	case 8:
		return NXS_CSI2_DATA_TYPE_RAW8;
	case 10:
		return NXS_CSI2_DATA_TYPE_RAW10;
	case 12:
		return NXS_CSI2_DATA_TYPE_RAW12;
	case 14:
		return NXS_CSI2_DATA_TYPE_RAW14;
	case 16:
		return NXS_CSI2_DATA_TYPE_RAW16;
	default:
		return 0;
	}
}

/*
 * The modes of aliensense,modes: six cells per mode, the width, the height,
 * the bit depth, the pixel phase (0xff for monochrome), the lane rate in
 * Mbps and the lanes. A mode's data type is the RAW code of its depth, as a
 * door's mode states it. The first mode is the default, and a mode's value
 * is its index.
 */
static int nxs_parse_dt_modes(struct nxs_csi2 *priv)
{
	struct device *dev = &priv->client->dev;
	u32 cells[NXS_MODE_TABLE_MAX * NXS_DT_CELLS];
	unsigned int i;
	int n;
	int err;

	n = device_property_count_u32(dev, "aliensense,modes");
	if (n <= 0 || n % NXS_DT_CELLS || n > ARRAY_SIZE(cells)) {
		dev_err(dev, "aliensense,modes: %d cells, needs %u per mode and %u modes at most\n",
			n, NXS_DT_CELLS, NXS_MODE_TABLE_MAX);
		return -EINVAL;
	}
	err = device_property_read_u32_array(dev, "aliensense,modes", cells, n);
	if (err)
		return err;
	priv->nr_dt_modes = n / NXS_DT_CELLS;
	for (i = 0; i < priv->nr_dt_modes; i++) {
		struct nxs_csi2_mode *mode = &priv->dt_modes[i];
		const u32 *c = cells + i * NXS_DT_CELLS;

		mode->width = c[NXS_DT_WIDTH];
		mode->height = c[NXS_DT_HEIGHT];
		mode->bit_depth = c[NXS_DT_BIT_DEPTH];
		mode->data_type = nxs_raw_data_type(c[NXS_DT_BIT_DEPTH]);
		mode->phase = c[NXS_DT_PIXEL_PHASE];
		mode->rate_mbps = c[NXS_DT_RATE_MBPS];
		mode->lanes = c[NXS_DT_LANES];
		mode->flags = i ? 0 : NXS_MODE_FLAG_DEFAULT;
		mode->value = i;
		mode->fps_max_milli = 0;
	}

	return 0;
}

/*
 * The standard id a row of aliensense,rows is served as, 0 for a row the
 * tree does not carry. The tree carries the range rows a door serves: the
 * exposure, the rate, the gain and the black level, which take a value, and
 * the line time, which is read-only.
 */
static u16 nxs_dt_row_std(u32 id, u32 step)
{
	switch (id) {
	case NXS_PARAM_ID_EXPOSURE_US:
	case NXS_PARAM_ID_FRAME_RATE_MFPS:
	case NXS_PARAM_ID_GAIN_MDB:
	case NXS_PARAM_ID_BLACK_LEVEL_DN:
		return step ? id : 0;
	case NXS_PARAM_ID_LINE_TIME_NS:
		return step ? 0 : id;
	default:
		return 0;
	}
}

/*
 * The rows of aliensense,rows: six cells per row of a mode, the mode's index
 * in aliensense,modes, the row's id, then its MIN, MAX, STEP and default in
 * the row's unit, a STEP of 0 for a read-only row. The core's rows become one
 * per id the tuples name, as a door's description gives them. A row's core
 * stays NULL, so the core's controls on it transfer nothing. A node without
 * the property has no rows.
 */
static int nxs_parse_dt_rows(struct nxs_csi2 *priv)
{
	struct device *dev = &priv->client->dev;
	struct nxs_csi2_core *core = &priv->core;
	unsigned int i;
	u16 n = 0;
	int count;
	int err;

	memset(core->row_of, -1, sizeof(core->row_of));
	if (!device_property_present(dev, "aliensense,rows"))
		return 0;
	count = device_property_count_u32(dev, "aliensense,rows");
	if (count <= 0 || count % NXS_DT_ROW_CELLS || count > NXS_DT_ROWS_MAX * NXS_DT_ROW_CELLS) {
		dev_err(dev, "aliensense,rows: %d cells, needs %u per row and %u rows at most\n",
			count, NXS_DT_ROW_CELLS, NXS_DT_ROWS_MAX);
		return -EINVAL;
	}
	priv->dt_rows = devm_kcalloc(dev, count, sizeof(*priv->dt_rows), GFP_KERNEL);
	core->rows = devm_kcalloc(dev, NXS_CSI2_ID_COUNT, sizeof(*core->rows), GFP_KERNEL);
	if (!priv->dt_rows || !core->rows)
		return -ENOMEM;
	err = device_property_read_u32_array(dev, "aliensense,rows", priv->dt_rows, count);
	if (err)
		return err;
	priv->nr_dt_rows = count / NXS_DT_ROW_CELLS;
	for (i = 0; i < priv->nr_dt_rows; i++) {
		const u32 *c = priv->dt_rows + i * NXS_DT_ROW_CELLS;
		u16 std = nxs_dt_row_std(c[NXS_DT_ROW_ID], c[NXS_DT_ROW_STEP]);
		struct nxs_csi2_row *row;

		if (c[NXS_DT_ROW_MODE] >= priv->nr_dt_modes || !std) {
			dev_warn(dev, "aliensense,rows: row %u, mode %u, id %u, not offered\n", i,
				 c[NXS_DT_ROW_MODE], c[NXS_DT_ROW_ID]);
			continue;
		}
		if (nxs_csi2_core_row(core, std) >= 0)
			continue;
		row = &core->rows[n];
		snprintf(row->name, sizeof(row->name), "row %u", n);
		row->index = n;
		row->id = std;
		row->std = std;
		row->type = NXS_ROW_TYPE_RANGE;
		row->kind = NXS_PARAM_KIND_APPLY;
		row->flags = c[NXS_DT_ROW_STEP] ? 0 : NXS_ROW_FLAG_READ_ONLY;
		row->count = 3;
		core->row_of[nxs_csi2_id_slot(std)] = n++;
	}
	core->client = priv->client;
	core->param_count = n;
	core->rows_cap = n;

	return 0;
}

/*
 * A passive node's rows for the mode in effect: a row takes the range and
 * the default of its tuple for that mode, and its value in effect is the
 * default. A row the mode does not list takes no value, its STEP 0, and a
 * read-only one has no reading, so its control reads inactive.
 */
static void nxs_take_dt_rows(struct nxs_csi2 *priv)
{
	struct nxs_csi2_core *core = &priv->core;
	unsigned int i;

	for (i = 0; i < core->param_count; i++) {
		struct nxs_csi2_row *row = &core->rows[i];

		memset(row->range, 0, sizeof(row->range));
		row->def = 0;
		row->live = NXS_ROW_UNREAD;
	}
	for (i = 0; i < priv->nr_dt_rows; i++) {
		const u32 *c = priv->dt_rows + i * NXS_DT_ROW_CELLS;
		int index = nxs_csi2_core_row(core, nxs_dt_row_std(c[NXS_DT_ROW_ID],
								   c[NXS_DT_ROW_STEP]));
		struct nxs_csi2_row *row;

		if (index < 0 || c[NXS_DT_ROW_MODE] != priv->cur->mode->value)
			continue;
		row = &core->rows[index];
		row->range[NXS_CSI2_MIN] = c[NXS_DT_ROW_MIN];
		row->range[NXS_CSI2_MAX] = c[NXS_DT_ROW_MAX];
		row->range[NXS_CSI2_STEP] = c[NXS_DT_ROW_STEP];
		row->def = c[NXS_DT_ROW_DEFAULT];
		row->live = row->def;
	}
}

/* ---- the rows and their controls ------------------------------------- */

static struct nxs_csi2_row *nxs_std_row(struct nxs_csi2 *priv, unsigned int id)
{
	int i = nxs_csi2_core_row(&priv->core, id);

	return i >= 0 ? &priv->core.rows[i] : NULL;
}

/* The value a set brings to a control. */
static s64 nxs_ctrl_val(const struct v4l2_ctrl *ctrl)
{
	return ctrl->type == V4L2_CTRL_TYPE_INTEGER64 ? *ctrl->p_new.p_s64 : ctrl->val;
}

/* The frame period in ns of a rate in milli-frames per second, nearest; 0 for 0. */
static u64 nxs_period_ns(u32 mfps)
{
	return mfps ? DIV_ROUND_CLOSEST_ULL(1000000000000ULL, mfps) : 0;
}

/* The rate in milli-frames per second of a frame of @lines lines in effect, nearest. */
static u32 nxs_rate_mfps(const struct nxs_csi2 *priv, s64 lines)
{
	u64 period = lines > 0 ? (u64)lines * priv->line_ns : 0;

	return period ? min_t(u64, div64_u64(1000000000000ULL + period / 2, period), U32_MAX) :
			U32_MAX;
}

/* Lines in effect as microseconds, nearest. */
static u32 nxs_lines_us(const struct nxs_csi2 *priv, s64 lines)
{
	return lines > 0 ? min_t(u64, div_u64((u64)lines * priv->line_ns + 500, 1000), U32_MAX) : 0;
}

/* Microseconds as lines in effect, nearest. */
static s64 nxs_us_lines(const struct nxs_csi2 *priv, u32 us)
{
	return div_u64((u64)us * 1000 + priv->line_ns / 2, priv->line_ns);
}

/*
 * The value a row takes from a set of its control, in the row's unit: the
 * exposure's lines in microseconds, the rate of the frame the height plus
 * the blanking make in lines, the flips as their bits, a range control's
 * value as it is.
 */
static u32 nxs_row_value(const struct nxs_csi2 *priv, const struct nxs_csi2_row *row)
{
	s64 val = nxs_ctrl_val(row->ctrl);

	switch (row->std) {
	case NXS_PARAM_ID_EXPOSURE_US:
		return nxs_lines_us(priv, val);
	case NXS_PARAM_ID_FRAME_RATE_MFPS:
		return nxs_rate_mfps(priv, priv->cur->mode->height + val);
	case NXS_PARAM_ID_FLIPS:
		return nxs_flips(priv, false);
	default:
		return (u32)val;
	}
}

/* The control value a row's value in effect reads as. */
static s64 nxs_live_to_ctrl(const struct nxs_csi2 *priv, const struct nxs_csi2_row *row, u32 live)
{
	s64 lines;

	switch (row->std) {
	case NXS_PARAM_ID_EXPOSURE_US:
		return nxs_us_lines(priv, live);
	case NXS_PARAM_ID_FRAME_RATE_MFPS:
		lines = div64_u64(nxs_period_ns(live) + priv->line_ns / 2, priv->line_ns);
		return lines > priv->cur->mode->height ? lines - priv->cur->mode->height : 0;
	default:
		return nxs_csi2_row_s64(row, live);
	}
}

/*
 * The exposure in lines: the row's range in microseconds, its shortest
 * rounded up and its longest rounded down, so the row takes every line count
 * the control offers. EXPOSURE is a 32-bit control: no bound passes S32_MAX.
 */
static void nxs_exposure_range(const struct nxs_csi2 *priv, const struct nxs_csi2_row *row,
			       s64 *min, s64 *max, u64 *step, s64 *def)
{
	s64 shortest, longest;

	nxs_csi2_core_ctrl_range(row, true, &shortest, &longest, step, def);
	*min = min_t(u64, DIV64_U64_ROUND_UP((u64)max_t(s64, shortest, 0) * 1000, priv->line_ns),
		     S32_MAX);
	*max = clamp_t(s64, div_u64((u64)max_t(s64, longest, 0) * 1000, priv->line_ns), *min,
		       S32_MAX);
	*def = clamp_t(s64, nxs_us_lines(priv, (u32)max_t(s64, *def, 0)), *min, *max);
	*step = 1;
}

/*
 * The vertical blanking: the frames the rate row's range names in lines, less
 * the mode's height. The slowest rate is the longest frame, so the row's MIN
 * bounds the blanking above and its MAX below. The shortest frame rounds up
 * and the longest down, so the row takes every blanking the control offers.
 * VBLANK is a 32-bit control: no bound passes S32_MAX.
 */
static void nxs_vblank_range(const struct nxs_csi2 *priv, const struct nxs_csi2_row *row, s64 *min,
			     s64 *max, u64 *step, s64 *def)
{
	s64 height = priv->cur->mode->height;
	s64 slowest, fastest;
	s64 lines;

	nxs_csi2_core_ctrl_range(row, true, &slowest, &fastest, step, def);
	lines = DIV64_U64_ROUND_UP(nxs_period_ns((u32)fastest), priv->line_ns);
	*min = clamp_t(s64, lines - height, 0, S32_MAX);
	lines = div64_u64(nxs_period_ns((u32)slowest), priv->line_ns);
	*max = clamp_t(s64, lines - height, *min, S32_MAX);
	lines = div64_u64(nxs_period_ns((u32)*def) + priv->line_ns / 2, priv->line_ns);
	*def = clamp_t(s64, lines - height, *min, *max);
	*step = 1;
}

/* The range of a row this adapter maps, as its control's. */
static void nxs_std_range(const struct nxs_csi2 *priv, const struct nxs_csi2_row *row, s64 *min,
			  s64 *max, u64 *step, s64 *def)
{
	switch (row->std) {
	case NXS_PARAM_ID_EXPOSURE_US:
		nxs_exposure_range(priv, row, min, max, step, def);
		break;
	case NXS_PARAM_ID_FRAME_RATE_MFPS:
		nxs_vblank_range(priv, row, min, max, step, def);
		break;
	default:
		nxs_csi2_core_ctrl_range(row, true, min, max, step, def);
		break;
	}
}

/* A control takes a value without a commit; a grabbed one keeps its own. */
static void nxs_sync_ctrl(struct v4l2_ctrl *ctrl, s64 val)
{
	if (ctrl->flags & V4L2_CTRL_FLAG_GRABBED)
		return;
	__v4l2_ctrl_s_ctrl(ctrl, clamp_t(s64, val, S32_MIN, S32_MAX));
}

/* The rows this adapter maps to the standard controls: exposure, vertical blanking, gain. */
static bool nxs_range_row(const struct nxs_csi2_row *row)
{
	return row->std == NXS_PARAM_ID_EXPOSURE_US || row->std == NXS_PARAM_ID_FRAME_RATE_MFPS ||
	       row->std == NXS_PARAM_ID_GAIN_MDB;
}

/* What an owner control reads: the line in effect in ns, the exposure row's MIN or MAX in us. */
static s32 nxs_owner_value(struct nxs_csi2 *priv, unsigned int which)
{
	const struct nxs_csi2_row *exposure = nxs_std_row(priv, NXS_PARAM_ID_EXPOSURE_US);

	switch (which) {
	case NXS_OWNER_EXPOSURE_MIN:
		return min_t(u32, exposure->range[NXS_CSI2_MIN], S32_MAX);
	case NXS_OWNER_EXPOSURE_MAX:
		return min_t(u32, exposure->range[NXS_CSI2_MAX], S32_MAX);
	default:
		return min_t(u32, priv->line_ns, S32_MAX);
	}
}

/*
 * Owner Line Time's minimum, the mode's shortest line: a paint under it is
 * clamped to it. Only a format set moves the mode, so inside the owner's
 * own paint the minimum stands and the range is not touched. A minimum
 * that rises above the control's value would clamp it to a line never in
 * effect, so the line in effect is written first and the range moves
 * under it: each subscriber hears that line once, and the later sync
 * writes nothing new.
 */
static void nxs_owner_line_range(struct nxs_csi2 *priv)
{
	struct v4l2_ctrl *line = priv->owner[NXS_OWNER_LINE];
	s64 shortest;

	if (!line)
		return;
	shortest = min_t(u64, nxs_shortest_ns(priv), line->maximum);
	if (line->minimum == shortest)
		return;
	if (line->cur.val < shortest)
		__v4l2_ctrl_s_ctrl(line, nxs_owner_value(priv, NXS_OWNER_LINE));
	__v4l2_ctrl_modify_range(line, shortest, line->maximum, 1,
				 clamp_t(s64, line->default_value, shortest, line->maximum));
}

/*
 * The line in effect, the standard controls' ranges from the rows' in its
 * lines, and the mode's pixel rate, horizontal blanking and link frequency.
 * Never called inside a set of a control it repaints: a range change sets
 * the control.
 */
static void nxs_apply_ranges(struct nxs_csi2 *priv)
{
	s64 hblank;
	unsigned int i;

	nxs_take_line(priv);
	hblank = nxs_hblank(priv);
	priv->syncing = true;
	for (i = 0; i < priv->core.param_count; i++) {
		struct nxs_csi2_row *row = &priv->core.rows[i];
		s64 min, max, def;
		u64 step;

		if (!row->ctrl || !nxs_range_row(row))
			continue;
		nxs_std_range(priv, row, &min, &max, &step, &def);
		__v4l2_ctrl_modify_range(row->ctrl, min, max, step, def);
	}
	__v4l2_ctrl_s_ctrl_int64(priv->pixel_rate, nxs_pixel_rate(priv->cur->mode));
	__v4l2_ctrl_modify_range(priv->hblank, hblank, hblank, 1, hblank);
	__v4l2_ctrl_s_ctrl(priv->link_freq, priv->cur->link_freq);
	nxs_owner_line_range(priv);
	priv->syncing = false;
}

/*
 * The exposure's range after an apply moved the frame: the device repaints
 * MIN and MAX under the new frame, and the control's range follows, as a
 * sensor driver's does when its vertical blanking changes. The exposure is
 * in no cluster with the rate, so no set of it is in flight here,
 * and a current value past the new range is clamped without a commit.
 */
static int nxs_sync_exposure_range(struct nxs_csi2 *priv)
{
	struct nxs_csi2_row *row = nxs_std_row(priv, NXS_PARAM_ID_EXPOSURE_US);
	s64 min, max, def;
	u64 step;
	int err;

	if (!row || !row->ctrl)
		return 0;
	err = nxs_csi2_core_read_range(&priv->core, row->index);
	if (err)
		return err;
	nxs_exposure_range(priv, row, &min, &max, &step, &def);
	priv->syncing = true;
	err = __v4l2_ctrl_modify_range(row->ctrl, min, max, step, def);
	priv->syncing = false;

	return err;
}

/*
 * The values in effect into the standard controls, after the core read
 * them, without a commit. Never called inside a set. A held row keeps the
 * value staged while the map was INERT. The flips set the Bayer order of
 * the format the pipeline negotiated, so the controls own them: flips LIVE
 * does not match are staged for stream on instead.
 */
static void nxs_sync_live(struct nxs_csi2 *priv)
{
	struct nxs_csi2_core *core = &priv->core;
	unsigned int i;

	priv->syncing = true;
	for (i = 0; i < core->param_count; i++) {
		struct nxs_csi2_row *row = &core->rows[i];

		if (!row->ctrl || nxs_csi2_rows_has(&core->held, i))
			continue;
		if (row->std == NXS_PARAM_ID_FLIPS) {
			if (row->live != nxs_flips(priv, true)) {
				nxs_csi2_core_stage(core, i, nxs_flips(priv, true));
				nxs_csi2_core_hold(core, i);
			}
		} else if (nxs_range_row(row)) {
			nxs_sync_ctrl(row->ctrl, nxs_live_to_ctrl(priv, row, row->live));
		}
	}
	priv->syncing = false;
}

/* After a run the map repaints every range and LIVE: the controls follow. */
static int nxs_refresh(struct nxs_csi2 *priv)
{
	int err = nxs_csi2_core_refresh(&priv->core);

	if (err)
		return err;
	nxs_apply_ranges(priv);
	nxs_sync_live(priv);

	return 0;
}

/*
 * The owner controls read the line and the exposure's bounds the rows hold
 * now, without a paint. Never called inside a set of them.
 */
static int nxs_sync_owner(struct nxs_csi2 *priv)
{
	unsigned int i;
	int err = 0;

	if (!priv->owner[NXS_OWNER_LINE])
		return 0;
	priv->syncing = true;
	for (i = 0; !err && i < NXS_OWNER_CTRLS; i++)
		err = __v4l2_ctrl_s_ctrl(priv->owner[i], nxs_owner_value(priv, i));
	priv->syncing = false;

	return err;
}

/*
 * The owner's paint: the line as the line in effect and the
 * bounds as the exposure row's range, then the ranges in the new line's
 * lines and the core's controls. The framework clamps a value the new
 * ranges leave outside and raises the events. The repaint sets no control
 * of the owner's cluster, so it takes no value of this set. The try has
 * refused a minimum above the maximum. The line becomes the line row's
 * default, and its value only where the mode lists the row: a paint
 * activates no row the mode does not list.
 */
static int nxs_owner_paint(struct nxs_csi2 *priv)
{
	struct nxs_csi2_row *exposure = nxs_std_row(priv, NXS_PARAM_ID_EXPOSURE_US);
	struct nxs_csi2_row *line = &priv->core.rows[priv->line_row];

	line->def = priv->owner[NXS_OWNER_LINE]->val;
	if (!nxs_csi2_row_unread(line))
		line->live = line->def;
	exposure->range[NXS_CSI2_MIN] = priv->owner[NXS_OWNER_EXPOSURE_MIN]->val;
	exposure->range[NXS_CSI2_MAX] = priv->owner[NXS_OWNER_EXPOSURE_MAX]->val;
	nxs_apply_ranges(priv);
	nxs_csi2_core_sync_ranges(&priv->core);

	return 0;
}

/* ---- the controls ---------------------------------------------------- */

/*
 * The rows a set names, into @rows: a control's own row, or in a cluster
 * the rows of the controls whose value changes. Both flips name the FLIPS
 * row.
 */
static void nxs_ctrl_rows(struct v4l2_ctrl *ctrl, struct nxs_csi2_rows *rows)
{
	const struct nxs_csi2_row *row;
	unsigned int i;

	if (ctrl->ncontrols == 1) {
		row = ctrl->priv;
		if (row)
			nxs_csi2_rows_add(rows, row->index);
		return;
	}
	for (i = 0; i < ctrl->ncontrols; i++) {
		struct v4l2_ctrl *member = ctrl->cluster[i];

		if (!member || !member->has_changed || !member->priv)
			continue;
		row = member->priv;
		nxs_csi2_rows_add(rows, row->index);
	}
}

/* The active format's code follows the flips a set brings. */
static void nxs_update_code(struct nxs_csi2 *priv)
{
	struct v4l2_subdev_state *state = v4l2_subdev_get_locked_active_state(&priv->sd);

	if (state)
		v4l2_subdev_state_get_format(state, 0)->code =
			nxs_code(priv, priv->cur->mode, nxs_flips(priv, false));
}

static int nxs_csi2_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct nxs_csi2 *priv = container_of(ctrl->handler, struct nxs_csi2, ctrls);
	struct nxs_csi2_core *core = &priv->core;
	struct nxs_csi2_row *frame = nxs_std_row(priv, NXS_PARAM_ID_FRAME_RATE_MFPS);
	struct nxs_csi2_rows rows = { };
	unsigned int i;
	int err;

	/*
	 * The handler's lock is priv->lock, held by the framework. A passive
	 * node's bus is another host's: the framework keeps the value and
	 * raises its event.
	 */
	if (ctrl == priv->owner[NXS_OWNER_LINE])
		return priv->syncing ? 0 : nxs_owner_paint(priv);
	nxs_ctrl_rows(ctrl, &rows);
	if (priv->syncing || priv->passive || nxs_csi2_rows_empty(&rows))
		return 0;
	for (i = 0; i < core->param_count; i++)
		if (nxs_csi2_rows_has(&rows, i))
			nxs_csi2_core_stage(core, i, nxs_row_value(priv, &core->rows[i]));
	err = nxs_csi2_core_commit(core, &rows, ctrl->name);
	if (err == NXS_CSI2_INERT) {
		/* The map takes a commit once its sensor runs: the values wait for stream on. */
		return 0;
	}
	if (err)
		return err;
	if (frame && nxs_csi2_rows_has(&rows, frame->index))
		err = nxs_sync_exposure_range(priv);
	if (!err && ctrl == priv->flips[0])
		nxs_update_code(priv);

	return err;
}

/*
 * A try and a set of the owner's cluster alike refuse a minimum above the
 * maximum. The driver's own sync writes the bounds one at a time and may
 * pass through such a pair.
 */
static int nxs_csi2_try_ctrl(struct v4l2_ctrl *ctrl)
{
	struct nxs_csi2 *priv = container_of(ctrl->handler, struct nxs_csi2, ctrls);
	const struct v4l2_ctrl *min = priv->owner[NXS_OWNER_EXPOSURE_MIN];
	const struct v4l2_ctrl *max = priv->owner[NXS_OWNER_EXPOSURE_MAX];

	if (ctrl != priv->owner[NXS_OWNER_LINE] || priv->syncing)
		return 0;

	return min->val > max->val ? -EINVAL : 0;
}

static const struct v4l2_ctrl_ops nxs_csi2_ctrl_ops = {
	.try_ctrl = nxs_csi2_try_ctrl,
	.s_ctrl = nxs_csi2_s_ctrl,
};

/* A standard control at a range row's range, backed by the row. */
static void nxs_new_std(struct nxs_csi2 *priv, struct nxs_csi2_row *row, u32 id)
{
	s64 min, max, def;
	u64 step;

	nxs_std_range(priv, row, &min, &max, &step, &def);
	row->ctrl = v4l2_ctrl_new_std(&priv->ctrls, &nxs_csi2_ctrl_ops, id, min, max, step, def);
	if (row->ctrl)
		row->ctrl->priv = row;
}

static struct v4l2_ctrl *nxs_new_flip(struct nxs_csi2 *priv, struct nxs_csi2_row *row, u32 id,
				      bool def)
{
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std(&priv->ctrls, &nxs_csi2_ctrl_ops, id, 0, 1, 1, def);
	if (ctrl) {
		/* A flip changes the Bayer order. */
		if (!priv->monochrome)
			ctrl->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
		ctrl->priv = row;
	}

	return ctrl;
}

/*
 * HFLIP and VFLIP for the FLIPS bits the row lists. The row backs both: the
 * first as its control and VFLIP beside HFLIP as its pair, so a re-read of
 * the door takes both off the row and puts both back.
 */
static void nxs_new_flips(struct nxs_csi2 *priv, struct nxs_csi2_row *row)
{
	u32 bits = 0;
	unsigned int i;

	for (i = 0; i < row->count; i++)
		bits |= nxs_csi2_row_value(row, i);
	if (bits & NXS_FLIP_X)
		priv->hflip = nxs_new_flip(priv, row, V4L2_CID_HFLIP, row->def & NXS_FLIP_X);
	if (bits & NXS_FLIP_Y)
		priv->vflip = nxs_new_flip(priv, row, V4L2_CID_VFLIP, row->def & NXS_FLIP_Y);
	row->ctrl = priv->hflip ? priv->hflip : priv->vflip;
	row->pair = priv->hflip ? priv->vflip : NULL;
}

static struct v4l2_ctrl *nxs_std_ctrl(struct nxs_csi2 *priv, unsigned int id)
{
	struct nxs_csi2_row *row = nxs_std_row(priv, id);

	return row ? row->ctrl : NULL;
}

/*
 * A passive node's owner controls, where its tree names the exposure and the
 * line rows: each reads what the rows hold for the mode in effect. The line
 * runs to one second and the bounds to S32_MAX.
 */
static void nxs_new_owner(struct nxs_csi2 *priv)
{
	static const char * const names[NXS_OWNER_CTRLS] = {
		"Owner Line Time", "Owner Exposure Min", "Owner Exposure Max",
	};
	static const u32 ids[NXS_OWNER_CTRLS] = {
		NXS_CSI2_CID_OWNER_LINE_NS(NXS_CSI2_CID_BASE),
		NXS_CSI2_CID_OWNER_EXPOSURE_MIN_US(NXS_CSI2_CID_BASE),
		NXS_CSI2_CID_OWNER_EXPOSURE_MAX_US(NXS_CSI2_CID_BASE),
	};
	struct v4l2_ctrl_config cfg = {
		.ops = &nxs_csi2_ctrl_ops,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.step = 1,
	};
	unsigned int i;

	if (!priv->passive || priv->line_row < 0 || !nxs_std_row(priv, NXS_PARAM_ID_EXPOSURE_US))
		return;
	for (i = 0; i < NXS_OWNER_CTRLS; i++) {
		cfg.id = ids[i];
		cfg.name = names[i];
		cfg.max = i == NXS_OWNER_LINE ? NSEC_PER_SEC : S32_MAX;
		cfg.min = i == NXS_OWNER_LINE ? min_t(u64, nxs_shortest_ns(priv), cfg.max) : 0;
		cfg.def = clamp_t(s64, nxs_owner_value(priv, i), cfg.min, cfg.max);
		priv->owner[i] = v4l2_ctrl_new_custom(&priv->ctrls, &cfg, NULL);
	}
}

/* Packs the controls that exist into one cluster, the first the master. */
static void nxs_cluster(struct v4l2_ctrl **cluster, struct v4l2_ctrl **ctrls, unsigned int n)
{
	unsigned int count = 0;
	unsigned int i;

	for (i = 0; i < n; i++)
		if (ctrls[i])
			cluster[count++] = ctrls[i];
	if (count > 1)
		v4l2_ctrl_cluster(count, cluster);
}

/*
 * The standard controls on the rows this adapter maps, in lines of the line
 * in effect, the core's controls on the rows it offers, and HBLANK,
 * PIXEL_RATE and LINK_FREQ for the line and the bus. STREAM and MODE are no
 * controls: s_stream commits the stream, and the mode is the declared
 * parameter the format reports. A passive node's rows are its tree's, and
 * its owner controls stand beside them. A door no pod answered at when it
 * probed, and a passive node without rows, carry the line's and the bus's
 * controls alone.
 */
static int nxs_init_controls(struct nxs_csi2 *priv)
{
	struct v4l2_ctrl_handler *hdl = &priv->ctrls;
	struct device *dev = &priv->client->dev;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *apply[2];
	struct v4l2_ctrl *flips[2];
	unsigned int i;
	s64 hblank;
	int err;

	err = v4l2_ctrl_handler_init(hdl, priv->core.param_count + 6 + NXS_OWNER_CTRLS);
	if (err)
		return err;
	hdl->lock = &priv->lock;
	nxs_take_line(priv);
	hblank = nxs_hblank(priv);

	if (priv->core.param_count) {
		for (i = 0; i < priv->core.param_count; i++) {
			struct nxs_csi2_row *row = &priv->core.rows[i];

			switch (row->std) {
			case NXS_PARAM_ID_EXPOSURE_US:
				nxs_new_std(priv, row, V4L2_CID_EXPOSURE);
				break;
			case NXS_PARAM_ID_FRAME_RATE_MFPS:
				nxs_new_std(priv, row, V4L2_CID_VBLANK);
				break;
			case NXS_PARAM_ID_GAIN_MDB:
				nxs_new_std(priv, row, V4L2_CID_ANALOGUE_GAIN);
				break;
			case NXS_PARAM_ID_FLIPS:
				nxs_new_flips(priv, row);
				break;
			default:
				break;
			}
		}
		err = nxs_csi2_core_init_controls(&priv->core, hdl, NXS_CSI2_CID_BASE);
		if (err)
			goto free;
		nxs_new_owner(priv);
	}
	priv->hblank = v4l2_ctrl_new_std(hdl, &nxs_csi2_ctrl_ops, V4L2_CID_HBLANK, hblank, hblank,
					 1, hblank);
	if (priv->hblank)
		priv->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	priv->pixel_rate = v4l2_ctrl_new_std(hdl, &nxs_csi2_ctrl_ops, V4L2_CID_PIXEL_RATE, 1,
					     S64_MAX, 1, nxs_pixel_rate(priv->cur->mode));
	priv->link_freq = v4l2_ctrl_new_int_menu(hdl, &nxs_csi2_ctrl_ops, V4L2_CID_LINK_FREQ,
						 priv->nr_link_freqs - 1, priv->cur->link_freq,
						 priv->link_freqs);
	if (priv->link_freq)
		priv->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	if (hdl->error) {
		err = hdl->error;
		goto free;
	}

	apply[0] = nxs_std_ctrl(priv, NXS_PARAM_ID_EXPOSURE_US);
	apply[1] = nxs_std_ctrl(priv, NXS_PARAM_ID_GAIN_MDB);
	nxs_cluster(priv->apply, apply, ARRAY_SIZE(apply));
	flips[0] = priv->hflip;
	flips[1] = priv->vflip;
	nxs_cluster(priv->flips, flips, ARRAY_SIZE(flips));
	if (priv->owner[NXS_OWNER_LINE])
		v4l2_ctrl_cluster(NXS_OWNER_CTRLS, priv->owner);

	err = v4l2_fwnode_device_parse(dev, &props);
	if (err)
		goto free;
	err = v4l2_ctrl_new_fwnode_properties(hdl, &nxs_csi2_ctrl_ops, &props);
	if (err)
		goto free;
	priv->sd.ctrl_handler = hdl;

	return 0;

free:
	v4l2_ctrl_handler_free(hdl);

	return err;
}

/* ---- the mode and the stream ----------------------------------------- */

/*
 * No pod answers at the door, or its map serves no mode yet: a pod answers
 * once its link is up and its personality runs.
 */
static bool nxs_empty(int err)
{
	return nxs_csi2_core_unanswered(err) || err == -ENOENT;
}

/*
 * The map, read by the core: nothing is written. The mode in effect is the
 * one the MODE row's LIVE names, else the default row's, since an INERT map
 * runs none. A map that does not read stands for none, and the core's
 * error says why: nxs_empty() tells a door no pod answers at yet.
 *
 * The sub-device's session on the link stands from here to remove, so a
 * control set commits whenever the map takes a commit, streaming or not. A
 * set that meets the map INERT waits for stream on, and a set that meets a
 * commit of the port's owner in flight fails with EBUSY.
 */
/* The formats, the format in effect and the line row of the map the core holds. */
static int nxs_attach_fmts(struct nxs_csi2 *priv)
{
	struct nxs_csi2_core *core = &priv->core;
	struct nxs_csi2_status status;
	int err;

	err = nxs_csi2_core_read_status(core, &status);
	if (!err)
		err = nxs_add_fmts(priv, core->modes, core->mode_count);
	if (err)
		return err;
	priv->cur = nxs_find_fmt(priv, status.current_mode);
	if (!priv->cur)
		priv->cur = nxs_default_fmt(priv);
	priv->line_row = nxs_csi2_core_row(core, NXS_PARAM_ID_LINE_TIME_NS);
	nxs_csi2_core_session(core, true);

	return 0;
}

/*
 * @locked is a caller that holds priv->lock, the controls' handler's: a
 * format ask or a stream start, never the probe.
 */
static int nxs_attach(struct nxs_csi2 *priv, bool locked)
{
	struct nxs_csi2_core *core = &priv->core;
	int err;

	err = locked ? nxs_csi2_core_init_locked(core, priv->client) :
		       nxs_csi2_core_init(core, priv->client);
	if (!err)
		err = nxs_attach_fmts(priv);
	if (err) {
		core->read = false;
		core->param_count = 0;
		core->mode_count = 0;
		priv->line_row = -1;
	}

	return err;
}

/*
 * The map of a door whose pod answered after the probe, read at the first
 * format ask or stream start that finds it: the formats become the map's,
 * and the line's and the bus's controls follow its mode. The row controls
 * stand on the rows read at probe, so such a door offers none until it
 * binds again. NXS_DOOR_EMPTY while no pod answers.
 */
static int nxs_ensure_map(struct nxs_csi2 *priv)
{
	int err;

	if (priv->passive || priv->core.read)
		return 0;
	err = nxs_attach(priv, true);
	if (nxs_empty(err))
		return NXS_DOOR_EMPTY;
	if (err)
		return err;
	nxs_apply_ranges(priv);

	return 0;
}

/*
 * The map read again after another personality was uploaded into the pod
 * behind the door (nxs_csi2_core_changed), the row controls standing on
 * rows of their ids and shapes. NXS_DOOR_EMPTY while the pod answers no map,
 * mid-upload.
 */
static int nxs_reread_map(struct nxs_csi2 *priv)
{
	struct nxs_csi2_core *core = &priv->core;
	int err;

	err = nxs_csi2_core_reread_locked(core);
	if (!err)
		err = nxs_attach_fmts(priv);
	if (err) {
		core->read = false;
		core->param_count = 0;
		core->mode_count = 0;
		priv->line_row = -1;
	}
	if (nxs_empty(err))
		return NXS_DOOR_EMPTY;
	if (err)
		return err;
	nxs_apply_ranges(priv);

	return 0;
}

/*
 * The map read again where it changed since the core read it: `status` is
 * the header just read and is read again after. NXS_DOOR_EMPTY while the
 * pod answers no map.
 */
static int nxs_follow_map(struct nxs_csi2 *priv, struct nxs_csi2_status *status)
{
	bool changed;
	int err;

	err = nxs_csi2_core_changed(&priv->core, status, &changed);
	if (err || !changed)
		return err;
	err = nxs_reread_map(priv);
	if (err)
		return err;

	return nxs_csi2_core_read_status(&priv->core, status);
}

/* A flip changes the Bayer order, so none changes while the stream runs. */
static void nxs_grab_flips(struct nxs_csi2 *priv, bool grab)
{
	if (priv->hflip)
		__v4l2_ctrl_grab(priv->hflip, grab);
	if (priv->vflip)
		__v4l2_ctrl_grab(priv->vflip, grab);
}

static int nxs_stream_err(struct nxs_csi2 *priv, int err)
{
	if (err != NXS_CSI2_INERT)
		return err;
	dev_err(&priv->client->dev,
		"the NXS Pod has not configured the sensor: bring the port up with nxs\n");

	return -ENODEV;
}

/*
 * Stream on: the mode of the active format, LIVE as the controls' values,
 * the controls staged while the map was INERT, then a GATE commit of
 * STREAM 1. A door no pod answers at streams nothing.
 */
static int nxs_stream_on(struct nxs_csi2 *priv)
{
	struct nxs_csi2_core *core = &priv->core;
	struct device *dev = &priv->client->dev;
	struct nxs_csi2_status status;
	int err;

	err = nxs_ensure_map(priv);
	if (err == NXS_DOOR_EMPTY) {
		dev_err(dev, "no NXS Pod answers at 0x%02x: the link has no camera to stream\n",
			priv->client->addr);
		return -ENODEV;
	}
	if (err)
		return err;
	err = nxs_csi2_core_read_status(core, &status);
	if (err)
		return err;
	err = nxs_follow_map(priv, &status);
	if (err == NXS_DOOR_EMPTY) {
		dev_err(dev, "the door at 0x%02x serves no camera description yet\n", priv->client->addr);
		return -ENODEV;
	}
	if (err)
		return err;
	if (status.state == NXS_MAP_STATE_INERT)
		return nxs_stream_err(priv, NXS_CSI2_INERT);
	if (status.current_mode != priv->cur->mode->value) {
		const struct nxs_fmt *fmt = nxs_find_fmt(priv, status.current_mode);

		if (!fmt) {
			dev_err(dev, "the NXS Pod runs mode %u, which the endpoint does not offer\n",
				status.current_mode);
			return -EINVAL;
		}
		priv->cur = fmt;
		nxs_apply_ranges(priv);
	}
	err = nxs_refresh(priv);
	if (!err && nxs_csi2_core_holds(core)) {
		err = nxs_csi2_core_commit_held(core);
		if (!err)
			err = nxs_refresh(priv);
	}
	if (err)
		return nxs_stream_err(priv, err);
	nxs_grab_flips(priv, true);
	err = nxs_csi2_core_gate(core, true);
	if (err) {
		nxs_grab_flips(priv, false);
		return nxs_stream_err(priv, err);
	}
	priv->streaming = true;

	return 0;
}

static int nxs_stream_off(struct nxs_csi2 *priv)
{
	int err = priv->core.read ? nxs_csi2_core_gate(&priv->core, false) : 0;

	nxs_grab_flips(priv, false);
	priv->streaming = false;

	return err == NXS_CSI2_INERT ? 0 : err;
}

/*
 * The mode is the camera's declared parameter, committed through the door by
 * the port's owner. The sub-device adopts the mode the map runs whenever a
 * format is asked, so its one format follows the door, never the reverse. A
 * passive node has no map to ask, a door no pod answers at keeps its
 * declared mode, and an INERT map runs no mode yet: the format stays the one
 * chosen before. A mode or a line the port's owner moved since the last
 * refresh repaints the controls.
 */
static int nxs_adopt_mode(struct nxs_csi2 *priv)
{
	struct nxs_csi2_status status;
	const struct nxs_fmt *fmt;
	bool moved;
	u32 line;
	int err;

	err = nxs_ensure_map(priv);
	if (err)
		return err == NXS_DOOR_EMPTY ? 0 : err;
	if (priv->passive || nxs_csi2_core_row(&priv->core, NXS_PARAM_ID_MODE) < 0)
		return 0;
	err = nxs_csi2_core_read_status(&priv->core, &status);
	if (err)
		return err;
	err = nxs_follow_map(priv, &status);
	if (err)
		return err == NXS_DOOR_EMPTY ? 0 : err;
	if (status.state == NXS_MAP_STATE_INERT)
		return 0;
	fmt = nxs_find_fmt(priv, status.current_mode);
	if (!fmt) {
		dev_err(&priv->client->dev, "the NXS Pod runs mode %u, which the endpoint does not offer\n",
			status.current_mode);
		return -EINVAL;
	}
	moved = fmt != priv->cur;
	if (priv->line_row >= 0) {
		line = priv->core.rows[priv->line_row].live;
		err = nxs_csi2_core_read_live(&priv->core, priv->line_row);
		if (err)
			return err;
		moved |= priv->core.rows[priv->line_row].live != line;
	}
	if (!moved)
		return 0;
	priv->cur = fmt;

	return nxs_refresh(priv);
}

/* ---- the pad and video ops ------------------------------------------- */

/* The state lock is priv->lock: the pad ops below run with it held. */
static int nxs_csi2_init_state(struct v4l2_subdev *sd, struct v4l2_subdev_state *state)
{
	struct nxs_csi2 *priv = to_nxs_csi2(sd);

	nxs_fill_fmt(priv, priv->cur, v4l2_subdev_state_get_format(state, 0));

	return 0;
}

static int nxs_csi2_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	struct nxs_csi2 *priv = to_nxs_csi2(sd);
	int err;

	if (code->index)
		return -EINVAL;
	err = nxs_adopt_mode(priv);
	if (err)
		return err;
	code->code = nxs_code(priv, priv->cur->mode, nxs_flips(priv, true));

	return 0;
}

static int nxs_csi2_enum_frame_size(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	struct nxs_csi2 *priv = to_nxs_csi2(sd);
	const struct nxs_csi2_mode *mode;
	int err;

	err = nxs_adopt_mode(priv);
	if (err)
		return err;
	mode = priv->cur->mode;
	if (fse->index || fse->code != nxs_code(priv, mode, nxs_flips(priv, true)))
		return -EINVAL;
	fse->min_width = mode->width;
	fse->max_width = mode->width;
	fse->min_height = mode->height;
	fse->max_height = mode->height;

	return 0;
}

/*
 * A door's mode is the camera's declared parameter, committed through the
 * door by the port's owner: the sub-device offers that one format, and a
 * format set, try or active, reports it. A host's stack scales or crops from
 * it and never changes the sensor's mode. A passive node's mode is the one
 * its host's stack sets among the tree's.
 */
/* The one format, as the door runs it now: a changed map is read again first. */
static int nxs_csi2_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
			    struct v4l2_subdev_format *format)
{
	struct nxs_csi2 *priv = to_nxs_csi2(sd);
	int err;

	err = nxs_adopt_mode(priv);
	if (err)
		return err;
	nxs_fill_fmt(priv, priv->cur, &format->format);
	*v4l2_subdev_state_get_format(state, 0) = format->format;

	return 0;
}

/*
 * A passive node's active format set: the first mode of the asked size and
 * code becomes the mode in effect, that mode's rows with it, and the
 * controls' ranges and activation follow, the core's too, and the owner
 * controls, the owner's paint gone. A size and code no mode has, the mode
 * in effect and a try keep the mode and the paint. Another mode while the
 * node streams is EBUSY.
 */
static int nxs_select_mode(struct nxs_csi2 *priv, const struct v4l2_subdev_format *format)
{
	const struct nxs_fmt *fmt;

	if (format->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return 0;
	fmt = nxs_match_fmt(priv, &format->format);
	if (!fmt || fmt == priv->cur)
		return 0;
	if (priv->streaming)
		return -EBUSY;
	priv->cur = fmt;
	nxs_take_dt_rows(priv);
	nxs_apply_ranges(priv);
	nxs_csi2_core_sync_ranges(&priv->core);

	return nxs_sync_owner(priv);
}

static int nxs_csi2_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
			    struct v4l2_subdev_format *format)
{
	struct nxs_csi2 *priv = to_nxs_csi2(sd);
	int err;

	err = nxs_adopt_mode(priv);
	if (!err && priv->passive)
		err = nxs_select_mode(priv, format);
	if (err)
		return err;
	nxs_fill_fmt(priv, priv->cur, &format->format);
	*v4l2_subdev_state_get_format(state, 0) = format->format;

	return 0;
}

/*
 * The selection rectangles from the modes the door serves. The array is the
 * largest mode's size, every pixel of it active. The map serves no crop
 * offset, so a format's crop is its size centred in the array, on an even
 * pixel so the Bayer order holds.
 */
static int nxs_csi2_get_selection(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				  struct v4l2_subdev_selection *sel)
{
	struct nxs_csi2 *priv = to_nxs_csi2(sd);
	struct v4l2_rect array = nxs_pixel_array(priv);
	const struct v4l2_mbus_framefmt *fmt;

	if (sel->pad)
		return -EINVAL;
	switch (sel->target) {
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r = array;
		return 0;
	case V4L2_SEL_TGT_CROP:
		fmt = v4l2_subdev_state_get_format(state, 0);
		sel->r.width = min(fmt->width, array.width);
		sel->r.height = min(fmt->height, array.height);
		sel->r.left = round_down((array.width - sel->r.width) / 2, 2);
		sel->r.top = round_down((array.height - sel->r.height) / 2, 2);
		return 0;
	default:
		return -EINVAL;
	}
}

static int nxs_csi2_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct nxs_csi2 *priv = to_nxs_csi2(sd);
	int err = 0;

	mutex_lock(&priv->lock);
	if (priv->passive)
		priv->streaming = enable;
	else
		err = enable ? nxs_stream_on(priv) : nxs_stream_off(priv);
	mutex_unlock(&priv->lock);

	return err;
}

/*
 * Before the receiver starts its lanes: a sensor left streaming parks. A door
 * no pod answers at has no sensor to park.
 */
static int nxs_csi2_pre_streamon(struct v4l2_subdev *sd, u32 flags)
{
	struct nxs_csi2 *priv = to_nxs_csi2(sd);
	int err;

	mutex_lock(&priv->lock);
	err = nxs_adopt_mode(priv);
	if (!err && priv->core.read && !priv->streaming && !priv->passive)
		err = nxs_csi2_core_park(&priv->core);
	mutex_unlock(&priv->lock);

	return err == NXS_DOOR_EMPTY ? 0 : err;
}

static const struct v4l2_subdev_video_ops nxs_csi2_video_ops = {
	.s_stream = nxs_csi2_s_stream,
	.pre_streamon = nxs_csi2_pre_streamon,
};

static const struct v4l2_subdev_pad_ops nxs_csi2_pad_ops = {
	.enum_mbus_code = nxs_csi2_enum_mbus_code,
	.enum_frame_size = nxs_csi2_enum_frame_size,
	.get_fmt = nxs_csi2_get_fmt,
	.set_fmt = nxs_csi2_set_fmt,
	.get_selection = nxs_csi2_get_selection,
};

static const struct v4l2_subdev_ops nxs_csi2_subdev_ops = {
	.video = &nxs_csi2_video_ops,
	.pad = &nxs_csi2_pad_ops,
};

static const struct v4l2_subdev_internal_ops nxs_csi2_internal_ops = {
	.init_state = nxs_csi2_init_state,
};

/* ---- the endpoint and the map ---------------------------------------- */

static int nxs_parse_endpoint(struct nxs_csi2 *priv)
{
	struct device *dev = &priv->client->dev;
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct fwnode_handle *endpoint;
	int err;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "no endpoint\n");
		return -EINVAL;
	}
	err = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep);
	fwnode_handle_put(endpoint);
	if (err) {
		dev_err(dev, "endpoint: %d\n", err);
		return err;
	}
	if (!ep.nr_of_link_frequencies) {
		dev_err(dev, "endpoint: link-frequencies missing\n");
		err = -EINVAL;
		goto free;
	}
	if (priv->deserializer && ep.nr_of_link_frequencies != 1) {
		dev_err(dev, "endpoint: a deserializer's output runs at one link frequency, %u named\n",
			ep.nr_of_link_frequencies);
		err = -EINVAL;
		goto free;
	}
	priv->lanes = ep.bus.mipi_csi2.num_data_lanes;
	priv->nr_link_freqs = ep.nr_of_link_frequencies;
	priv->link_freqs = devm_kmemdup(dev, ep.link_frequencies,
					ep.nr_of_link_frequencies * sizeof(*ep.link_frequencies),
					GFP_KERNEL);
	if (!priv->link_freqs)
		err = -ENOMEM;
free:
	v4l2_fwnode_endpoint_free(&ep);

	return err;
}

/* The modes of the tree, the first one the format, and a passive node's rows for it. */
static int nxs_read_tree(struct nxs_csi2 *priv)
{
	int err;

	err = nxs_parse_dt_modes(priv);
	if (err)
		return err;
	err = nxs_add_fmts(priv, priv->dt_modes, priv->nr_dt_modes);
	if (err)
		return err;
	priv->cur = nxs_default_fmt(priv);
	if (!priv->passive)
		return 0;
	err = nxs_parse_dt_rows(priv);
	if (err)
		return err;
	nxs_take_dt_rows(priv);
	priv->line_row = nxs_csi2_core_row(&priv->core, NXS_PARAM_ID_LINE_TIME_NS);

	return 0;
}

/*
 * A camera door at probe: its map where a pod answers, else the declared
 * modes of aliensense,modes until one does. A door without the property
 * waits for its map, which the deserializer's driver brings up before this
 * probe on a kernel-owned link and the host tool on a tool-owned one: the
 * probe defers.
 */
static int nxs_read_map(struct nxs_csi2 *priv)
{
	struct device *dev = &priv->client->dev;
	u8 addr = priv->client->addr;
	const char *why;
	int err;

	err = nxs_attach(priv, false);
	if (!nxs_empty(err))
		return err;
	why = err == -ENOENT ? "serves no mode" : "does not answer";
	if (!device_property_present(dev, "aliensense,modes"))
		return dev_err_probe(dev, -EPROBE_DEFER, "camera door at 0x%02x %s%s\n", addr, why,
				     err == -ENOENT ? ": bring the port up with nxs" : "");
	err = nxs_read_tree(priv);
	if (err)
		return err;
	dev_info(dev, "camera door at 0x%02x %s: the declared mode %ux%u until an NXS Pod answers\n", addr,
		 why, priv->cur->mode->width, priv->cur->mode->height);

	return 0;
}

/* ---- probe and remove ------------------------------------------------ */

static int nxs_csi2_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct nxs_csi2 *priv;
	int err;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return dev_err_probe(dev, -ENODEV, "the adapter takes no plain I2C transfers\n");
	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->client = client;
	priv->core.adapter_ops = &nxs_csi2_ctrl_ops;
	priv->line_row = -1;
	priv->passive = device_property_read_bool(dev, "aliensense,passive");
	priv->deserializer = device_property_read_bool(dev, "aliensense,deserializer");
	priv->monochrome = device_property_read_bool(dev, "aliensense,monochrome");
	mutex_init(&priv->lock);

	err = nxs_parse_endpoint(priv);
	if (err)
		goto destroy;
	err = priv->passive ? nxs_read_tree(priv) : nxs_read_map(priv);
	if (err)
		goto destroy;
	if (priv->passive)
		dev_info(dev, "passive: %u modes from the device tree, no transfer\n", priv->nr_fmts);
	err = nxs_init_controls(priv);
	if (err)
		goto destroy;

	v4l2_i2c_subdev_init(&priv->sd, client, &nxs_csi2_subdev_ops);
	priv->sd.internal_ops = &nxs_csi2_internal_ops;
	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	priv->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	err = media_entity_pads_init(&priv->sd.entity, 1, &priv->pad);
	if (err)
		goto ctrls;
	priv->sd.state_lock = &priv->lock;
	err = v4l2_subdev_init_finalize(&priv->sd);
	if (err)
		goto entity;
	err = v4l2_async_register_subdev_sensor(&priv->sd);
	if (err)
		goto state;
	return 0;

state:
	v4l2_subdev_cleanup(&priv->sd);
entity:
	media_entity_cleanup(&priv->sd.entity);
ctrls:
	v4l2_ctrl_handler_free(&priv->ctrls);
destroy:
	mutex_destroy(&priv->lock);

	return err;
}

static void nxs_csi2_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct nxs_csi2 *priv = to_nxs_csi2(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&priv->ctrls);
	mutex_destroy(&priv->lock);
}

static const struct of_device_id nxs_csi2_of_match[] = {
	{ .compatible = "aliensense,nxs-csi2" },
	{ }
};
MODULE_DEVICE_TABLE(of, nxs_csi2_of_match);

static struct i2c_driver nxs_csi2_i2c_driver = {
	.driver = {
		.name = "nxs-csi2",
		.of_match_table = nxs_csi2_of_match,
	},
	.probe = nxs_csi2_probe,
	.remove = nxs_csi2_remove,
};
module_i2c_driver(nxs_csi2_i2c_driver);

MODULE_DESCRIPTION("V4L2 driver for a camera door of the NXS Device Interface");
MODULE_AUTHOR("Aliensense");
MODULE_LICENSE("GPL");

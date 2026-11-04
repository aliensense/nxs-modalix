// SPDX-License-Identifier: GPL-2.0-only
/*
 * nxs-csi2-core.c - the NXS map's camera door, for the drivers that serve it
 *
 * Copyright (C) 2026 Aliensense
 *
 * One core for the two adapters: nxs-csi2, the V4L2 sub-device driver, and
 * nxs-csi2-tegra, the tegracam driver. The core knows the map and no sensor.
 * A pod, or a smart camera, serves the map at its camera door: a
 * flat map behind a 16-bit register pointer, high byte first, with the
 * header's capture fields, the value and the range in effect of each
 * parameter row, the staged cells, the commit block, the mode table and the
 * rows. The core reads it, commits the rows in their units as one write
 * each, and the device converts the values to its sensor's registers. The
 * constants are nxs-csi2-map.h, generated from the map's one source
 * (redbrain constants/nxs_map.yaml). The contract is the NXS Interface
 * Description, chapters 3, 4, 6.4 and 12, and the drivers' sequence is
 * docs/bring-up/nxs-csi2-mainline.md in the redbrain tree.
 *
 * The windows (§6.4). The core reads STATE once under the host bus lock
 * before every COMMIT, since a COMMIT written while BUSY or INERT is
 * ignored, and keeps the lock through the write until it has read STATE
 * past the device's bound. After the write it sends nothing for the bound
 * the map painted for the kind: APPLY_RUN_MS after an apply, MODE_RUN_MS,
 * GATE_RUN_MS or TRIGGER_RUN_MS after a run. Then it reads STATE once, and
 * once more BOUND_LATENESS_MS later while the run still ends; BUSY past
 * that is the device's failure. Every transfer is one try: the device
 * answers outside its bound, so a NAK is a silent address or a fault. The
 * core commits only while the adapter has a capture session open on the
 * link (nxs_csi2_core_session):
 * the host's tool brings the port up and joins its links while no session
 * is open, and a control set meanwhile, tegracam's defaults at probe among
 * them, waits in the held rows for the session's start.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <media/v4l2-ctrls.h>

#include "nxs-csi2-core.h"

/* The longest commit: every staged cell, MASK and COMMIT. */
#define NXS_CORE_COMMIT_MAX	(4 * NXS_MAX_ROWS + NXS_MAP_REG_MASK_LEN + 1)
/* The header's capture fields, read at init. */
#define NXS_CORE_HEADER_LEN	(NXS_MAP_REG_PROTO_VERSION + 1)
/* What a session reads: the header through STREAM. */
#define NXS_CORE_STATUS_LEN	(NXS_MAP_REG_STREAM + 1)
/* A mode-table value the map reports before the first run. */
#define NXS_CORE_NO_MODE	0xff
/*
 * The trigger menu's entries are the TriggerCode values. A code past the
 * registry is named by its number, and the menu holds the codes below 32.
 */
#define NXS_CORE_TRIGGER_MENU_MAX	32

static const char * const nxs_core_trigger_names[] = {
	[NXS_TRIGGER_CODE_FREERUN] = "Free Run",
	[NXS_TRIGGER_CODE_FAST] = "Fast Trigger",
	[NXS_TRIGGER_CODE_SEQUENTIAL] = "Sequential Trigger",
};

static const struct v4l2_ctrl_ops nxs_core_ctrl_ops;

static u32 nxs_le32(const u8 *p)
{
	return p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24);
}

static u16 nxs_le16(const u8 *p)
{
	return p[0] | ((u16)p[1] << 8);
}

static void nxs_put_le32(u8 *p, u32 v)
{
	p[0] = v & 0xff;
	p[1] = (v >> 8) & 0xff;
	p[2] = (v >> 16) & 0xff;
	p[3] = (v >> 24) & 0xff;
}

/* ---- the map ----------------------------------------------------------- */

/* A NAK or a lost arbitration: the map at the address did not answer. */
bool nxs_csi2_core_unanswered(int err)
{
	return err == -ENXIO || err == -EREMOTEIO || err == -EIO || err == -EAGAIN;
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_unanswered);

/*
 * One transfer, one try: the 2-byte register pointer and the bytes written,
 * or the pointer and a read under a repeated start. @locked is a transfer
 * under the caller's bus lock, a commit's window; otherwise the adapter
 * locks. The device answers outside its bound, so a NAK is a silent address
 * or a fault.
 */
static int nxs_core_xfer(struct i2c_client *client, bool locked, u16 reg, const u8 *wbuf,
			 size_t wlen, u8 *rbuf, size_t rlen)
{
	u8 out[2 + NXS_CORE_COMMIT_MAX];
	struct i2c_msg msgs[2] = {
		{ .addr = client->addr, .flags = 0, .len = 2 + wlen, .buf = out },
		{ .addr = client->addr, .flags = I2C_M_RD, .len = rlen, .buf = rbuf },
	};
	int n = rlen ? 2 : 1;
	int err;

	if (wlen > NXS_CORE_COMMIT_MAX)
		return -EINVAL;
	out[0] = reg >> 8;
	out[1] = reg & 0xff;
	if (wlen)
		memcpy(out + 2, wbuf, wlen);
	err = locked ? __i2c_transfer(client->adapter, msgs, n) : i2c_transfer(client->adapter, msgs, n);
	if (err == n)
		return 0;

	return err >= 0 ? -EIO : err;
}

static int nxs_core_read(struct nxs_csi2_core *core, u16 reg, u8 *buf, size_t len)
{
	return nxs_core_xfer(core->client, false, reg, NULL, 0, buf, len);
}

/*
 * STATE under the bus lock, one read. The wire is idle when the core reads,
 * so a value outside the four is the device's fault, never a torn read.
 */
static int nxs_core_state(struct i2c_client *client, u8 *state)
{
	int err = nxs_core_xfer(client, true, NXS_MAP_REG_STATE, NULL, 0, state, 1);

	if (err)
		return err;

	return *state > NXS_MAP_STATE_FAULT ? -EBADMSG : 0;
}

const char *nxs_csi2_core_state_name(u8 state)
{
	switch (state) {
	case NXS_MAP_STATE_INERT:
		return "INERT";
	case NXS_MAP_STATE_READY:
		return "READY";
	case NXS_MAP_STATE_BUSY:
		return "BUSY";
	case NXS_MAP_STATE_FAULT:
		return "FAULT";
	default:
		return "unknown";
	}
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_state_name);

static const char *nxs_core_module_kind_name(u8 kind)
{
	switch (kind) {
	case NXS_MODULE_KIND_POD:
	case NXS_MODULE_KIND_HEADED:
		return "a pod";
	case NXS_MODULE_KIND_SMART_CAMERA:
		return "a smart camera";
	case NXS_MODULE_KIND_SMART_MODULE:
		return "a smart module";
	default:
		return "an unknown module kind";
	}
}

/* ---- the windows ----------------------------------------------------- */

/*
 * A COMMIT written while BUSY or INERT is ignored, so STATE is read once
 * under the host bus lock. The lock is kept once the map reads READY or
 * FAULT, so no host transfer lands between the check and the commit. A map
 * under a run in flight returns -EBUSY and one that reads INERT
 * NXS_CSI2_INERT, both with the lock released.
 */
static int nxs_core_idle(struct i2c_client *client)
{
	u8 state;
	int err;

	i2c_lock_bus(client->adapter, I2C_LOCK_SEGMENT);
	err = nxs_core_state(client, &state);
	if (!err && state == NXS_MAP_STATE_BUSY)
		err = -EBUSY;
	else if (!err && state == NXS_MAP_STATE_INERT)
		err = NXS_CSI2_INERT;
	if (err)
		i2c_unlock_bus(client->adapter, I2C_LOCK_SEGMENT);

	return err;
}

/*
 * The window after a COMMIT, the bus held. The host waits the bound the
 * map painted for the kind and sends nothing, then reads STATE once. A run
 * ends within BOUND_LATENESS_MS of its deadline, so a BUSY read is read
 * once more after that; BUSY past it stands.
 */
static int nxs_core_settle(struct i2c_client *client, u16 wait_ms, u8 *state)
{
	int err;

	if (wait_ms >= 20)
		msleep(wait_ms);
	else if (wait_ms)
		usleep_range(wait_ms * 1000, wait_ms * 1000 + 200);
	err = nxs_core_state(client, state);
	if (!err && *state == NXS_MAP_STATE_BUSY) {
		usleep_range(NXS_BOUND_LATENESS_MS * 1000, NXS_BOUND_LATENESS_MS * 1000 + 200);
		err = nxs_core_state(client, state);
	}

	return err;
}

/* The cell that holds the bound of a commit of a kind. */
static u16 nxs_core_wait_cell(u8 kind)
{
	switch (kind) {
	case NXS_PARAM_KIND_APPLY:
		return NXS_MAP_REG_APPLY_RUN_MS;
	case NXS_PARAM_KIND_RUN:
		return NXS_MAP_REG_MODE_RUN_MS;
	case NXS_PARAM_KIND_GATE:
		return NXS_MAP_REG_GATE_RUN_MS;
	default:
		return NXS_MAP_REG_TRIGGER_RUN_MS;
	}
}

/*
 * A FAULT's ERROR cell in the kernel's numbering. The device numbers its
 * errnos as its own C library does (nxs-csi2-map.h, from §12), so the
 * translation is by name; a value the table lacks reads EIO.
 */
static int nxs_core_errno(u8 error)
{
	switch (error) {
	case NXS_DEV_ENOENT:
		return -ENOENT;
	case NXS_DEV_EIO:
		return -EIO;
	case NXS_DEV_ENXIO:
		return -ENXIO;
	case NXS_DEV_ENOEXEC:
		return -ENOEXEC;
	case NXS_DEV_EBADF:
		return -EBADF;
	case NXS_DEV_EAGAIN:
		return -EAGAIN;
	case NXS_DEV_ETIME:
		return -ETIME;
	case NXS_DEV_EFAULT:
		return -EFAULT;
	case NXS_DEV_EBUSY:
		return -EBUSY;
	case NXS_DEV_EEXIST:
		return -EEXIST;
	case NXS_DEV_ENODEV:
		return -ENODEV;
	case NXS_DEV_EINVAL:
		return -EINVAL;
	case NXS_DEV_EFBIG:
		return -EFBIG;
	case NXS_DEV_ENOSPC:
		return -ENOSPC;
	case NXS_DEV_ERANGE:
		return -ERANGE;
	case NXS_DEV_ENODATA:
		return -ENODATA;
	case NXS_DEV_EPROTO:
		return -EPROTO;
	case NXS_DEV_EBADMSG:
		return -EBADMSG;
	case NXS_DEV_EOPNOTSUPP:
	case NXS_DEV_ENOTSUP:
		return -EOPNOTSUPP;
	case NXS_DEV_ETIMEDOUT:
		return -ETIMEDOUT;
	case NXS_DEV_ESTALE:
		return -ESTALE;
	case NXS_DEV_EILSEQ:
		return -EILSEQ;
	case NXS_DEV_ECANCELED:
		return -ECANCELED;
	default:
		return -EIO;
	}
}

/*
 * One commit: the staged cells from the highest row of @mask down to row 0,
 * MASK and COMMIT in one write, so a transfer the bus cuts never applies.
 * Every row of @mask shares @kind. The host bus stays held from the STATE
 * check before the write until the read past the device's bound. The bound
 * the map painted for the kind is read before the write. Returns
 * NXS_CSI2_INERT when the map has not configured its sensor and nothing
 * was written, -EBUSY under a run in flight.
 */
static int nxs_core_commit_one(struct nxs_csi2_core *core, u64 mask, u8 kind, const char *what)
{
	struct i2c_client *client = core->client;
	struct device *dev = &client->dev;
	u8 buf[NXS_CORE_COMMIT_MAX];
	u8 ms[NXS_MAP_REG_APPLY_RUN_MS_LEN] = { 0, 0 };
	u8 state = NXS_MAP_STATE_BUSY;
	u8 error = 0;
	u16 wait_ms;
	unsigned int hi;
	unsigned int len;
	unsigned int i;
	int err;

	if (!mask)
		return 0;
	hi = fls64(mask) - 1;
	for (i = 0; i <= hi; i++)
		nxs_put_le32(buf + 4 * (hi - i), (mask & BIT_ULL(i)) ? core->staged[i] : 0);
	len = 4 * (hi + 1);
	for (i = 0; i < NXS_MAP_REG_MASK_LEN; i++)
		buf[len + i] = (mask >> (8 * i)) & 0xff;
	buf[len + NXS_MAP_REG_MASK_LEN] = 1;
	len += NXS_MAP_REG_MASK_LEN + 1;

	err = nxs_core_idle(client);
	if (err)
		return err;
	wait_ms = 0;
	if (!nxs_core_xfer(client, true, nxs_core_wait_cell(kind), NULL, 0, ms, sizeof(ms)))
		wait_ms = nxs_le16(ms);
	err = nxs_core_xfer(client, true, NXS_MAP_REG_MASK - 4 * (hi + 1), buf, len, NULL, 0);
	if (!err)
		err = nxs_core_settle(client, wait_ms, &state);
	if (!err && state == NXS_MAP_STATE_FAULT)
		err = nxs_core_xfer(client, true, NXS_MAP_REG_ERROR, NULL, 0, &error, 1);
	i2c_unlock_bus(client->adapter, I2C_LOCK_SEGMENT);
	if (err) {
		dev_warn_ratelimited(dev, "%s: bus error %d\n", what, err);
		return err;
	}
	switch (state) {
	case NXS_MAP_STATE_READY:
		return 0;
	case NXS_MAP_STATE_INERT:
		return NXS_CSI2_INERT;
	case NXS_MAP_STATE_FAULT:
		dev_warn_ratelimited(dev, "%s: the camera map refused the commit (error %u)\n",
				     what, error);
		return nxs_core_errno(error);
	default:
		dev_warn_ratelimited(dev, "%s: the camera map stayed busy past its painted bound\n", what);
		return -EBUSY;
	}
}

/* ---- the rows -------------------------------------------------------- */

/* The index of @value among an enum row's values, -1 for a value off the list. */
static int nxs_core_row_find(const struct nxs_csi2_row *row, u32 value)
{
	unsigned int i;

	for (i = 0; i < row->count; i++)
		if (row->values[i] == value)
			return i;

	return -1;
}

/* A row the drivers offer: read-only or of a kind a commit runs, and an enum with values. */
static bool nxs_core_row_offered(const struct nxs_csi2_row *row)
{
	if (!(row->type & NXS_ROW_TYPE_RANGE) && !row->count)
		return false;
	if (nxs_csi2_row_read_only(row))
		return true;

	return row->kind <= NXS_PARAM_KIND_TRIGGER;
}

/*
 * The rows the core offers as controls: the trigger, the test pattern, the
 * black level, the temperature and every custom row. The others are the
 * adapter's, or the stream gate's and the format's.
 */
static bool nxs_core_owns(const struct nxs_csi2_row *row)
{
	switch (row->std) {
	case NXS_PARAM_ID_CUSTOM:
	case NXS_PARAM_ID_BLACK_LEVEL_DN:
	case NXS_PARAM_ID_TEST_PATTERN:
	case NXS_PARAM_ID_TRIGGER:
	case NXS_PARAM_ID_TEMPERATURE_DC:
		return true;
	default:
		return false;
	}
}

static void nxs_core_parse_row(struct nxs_csi2_row *row, unsigned int index, const u8 *buf)
{
	bool sign = buf[NXS_ROW_FIELD_TYPE] & NXS_ROW_TYPE_SIGNED;
	unsigned int i;

	memcpy(row->name, buf + NXS_ROW_FIELD_NAME, NXS_ROW_FIELD_NAME_LEN);
	row->name[NXS_ROW_FIELD_NAME_LEN] = '\0';
	if (!row->name[0])
		snprintf(row->name, sizeof(row->name), "row %u", index);
	row->index = index;
	row->id = nxs_le16(buf + NXS_ROW_FIELD_ID);
	row->type = buf[NXS_ROW_FIELD_TYPE];
	row->kind = buf[NXS_ROW_FIELD_KIND];
	row->flags = buf[NXS_ROW_FIELD_FLAGS];
	row->def = nxs_le32(buf + NXS_ROW_FIELD_DEFAULT);
	row->std = 0;
	row->ctrl = NULL;
	if (row->type & NXS_ROW_TYPE_RANGE) {
		row->count = 3;
		for (i = 0; i < 3; i++)
			row->values[i] = nxs_le32(buf + NXS_ROW_FIELD_VALUES + 4 * i);
	} else if (row->type & NXS_ROW_TYPE_WIDE) {
		row->count = min_t(u8, buf[NXS_ROW_FIELD_COUNT], NXS_ROW_WIDE_VALUES_MAX);
		for (i = 0; i < row->count; i++)
			row->values[i] = nxs_le32(buf + NXS_ROW_FIELD_VALUES + 4 * i);
	} else {
		row->count = min_t(u8, buf[NXS_ROW_FIELD_COUNT], NXS_ROW_VALUES_MAX);
		/* A signed row's cells hold 32-bit two's complement: so do its values. */
		for (i = 0; i < row->count; i++) {
			u16 v = nxs_le16(buf + NXS_ROW_FIELD_VALUES + 2 * i);

			row->values[i] = sign ? (u32)(s32)(s16)v : v;
		}
	}
}

/*
 * The standard id a row is served as. The first row of an id the drivers
 * know becomes that id's control when its type fits and the host may write
 * it. Any other row is custom: id 0, an id past the registry, a second row
 * of an id, or a standard row the drivers cannot drive.
 */
static u8 nxs_core_std(const struct nxs_csi2_core *core, const struct nxs_csi2_row *row)
{
	bool range = row->type & NXS_ROW_TYPE_RANGE;
	bool writable = !nxs_csi2_row_read_only(row);

	if (!row->id || row->id >= NXS_CSI2_ID_COUNT || core->row_of[row->id] >= 0)
		return 0;
	switch (row->id) {
	case NXS_PARAM_ID_EXPOSURE_US:
	case NXS_PARAM_ID_FRAME_RATE_MFPS:
	case NXS_PARAM_ID_GAIN_MDB:
		return range && writable ? row->id : 0;
	case NXS_PARAM_ID_FLIPS:
	case NXS_PARAM_ID_TEST_PATTERN:
	case NXS_PARAM_ID_TRIGGER:
	case NXS_PARAM_ID_STREAM:
	case NXS_PARAM_ID_MODE:
		return !range && writable ? row->id : 0;
	default:
		return row->id;
	}
}

/* ---- the modes ------------------------------------------------------- */

/*
 * The media bus code of a mode under the flips: PIXEL_PHASE XOR the FLIPS
 * bits. A mode whose capture table names no Bayer order is monochrome. 0
 * for a depth or a phase the core does not know.
 */
u32 nxs_csi2_core_mbus_code(const struct nxs_csi2_mode *mode, u8 flips)
{
	static const u32 bayer[3][4] = {
		{ MEDIA_BUS_FMT_SRGGB8_1X8, MEDIA_BUS_FMT_SGRBG8_1X8,
		  MEDIA_BUS_FMT_SGBRG8_1X8, MEDIA_BUS_FMT_SBGGR8_1X8 },
		{ MEDIA_BUS_FMT_SRGGB10_1X10, MEDIA_BUS_FMT_SGRBG10_1X10,
		  MEDIA_BUS_FMT_SGBRG10_1X10, MEDIA_BUS_FMT_SBGGR10_1X10 },
		{ MEDIA_BUS_FMT_SRGGB12_1X12, MEDIA_BUS_FMT_SGRBG12_1X12,
		  MEDIA_BUS_FMT_SGBRG12_1X12, MEDIA_BUS_FMT_SBGGR12_1X12 },
	};
	static const u32 mono[3] = {
		MEDIA_BUS_FMT_Y8_1X8, MEDIA_BUS_FMT_Y10_1X10, MEDIA_BUS_FMT_Y12_1X12,
	};
	unsigned int depth;

	switch (mode->bit_depth) {
	case 8:
		depth = 0;
		break;
	case 10:
		depth = 1;
		break;
	case 12:
		depth = 2;
		break;
	default:
		return 0;
	}
	if (mode->phase == 0xff)
		return mono[depth];
	if (mode->phase > 3)
		return 0;

	return bayer[depth][mode->phase ^ (flips & (NXS_FLIP_X | NXS_FLIP_Y))];
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_mbus_code);

/* The mode table, MODE_COUNT rows, as the map serves it. */
static int nxs_core_read_modes(struct nxs_csi2_core *core)
{
	u8 *table = core->table;
	unsigned int i;
	int err;

	err = nxs_core_read(core, NXS_MAP_REG_MODE_TABLE, table,
			    core->mode_count * NXS_MODE_ENTRY_SIZE);
	if (err)
		return err;
	for (i = 0; i < core->mode_count; i++) {
		const u8 *row = table + i * NXS_MODE_ENTRY_SIZE;
		struct nxs_csi2_mode *mode = &core->modes[i];

		mode->width = nxs_le16(row + NXS_MODE_ENTRY_WIDTH);
		mode->height = nxs_le16(row + NXS_MODE_ENTRY_HEIGHT);
		mode->bit_depth = row[NXS_MODE_ENTRY_BIT_DEPTH];
		mode->lanes = row[NXS_MODE_ENTRY_LANES];
		mode->rate_mbps = nxs_le16(row + NXS_MODE_ENTRY_RATE_MBPS);
		mode->flags = row[NXS_MODE_ENTRY_FLAGS];
		mode->fps_max_milli = nxs_le32(row + NXS_MODE_ENTRY_FPS_MAX_MILLI);
		mode->value = row[NXS_MODE_ENTRY_MODE_VALUE];
		mode->phase = row[NXS_MODE_ENTRY_PIXEL_PHASE];
	}

	return 0;
}

/* The mode a MODE_VALUE selects, NULL for none. */
const struct nxs_csi2_mode *nxs_csi2_core_find_mode(const struct nxs_csi2_core *core, u8 value)
{
	unsigned int i;

	for (i = 0; i < core->mode_count; i++)
		if (core->modes[i].value == value)
			return &core->modes[i];

	return NULL;
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_find_mode);

/* The first mode of a geometry, at @bit_depth when it is not 0, NULL for none. */
const struct nxs_csi2_mode *nxs_csi2_core_find_geometry(const struct nxs_csi2_core *core,
							u32 width, u32 height, u8 bit_depth)
{
	unsigned int i;

	for (i = 0; i < core->mode_count; i++) {
		const struct nxs_csi2_mode *mode = &core->modes[i];

		if (mode->width == width && mode->height == height &&
		    (!bit_depth || mode->bit_depth == bit_depth))
			return mode;
	}

	return NULL;
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_find_geometry);

/* ---- the ranges and the values in effect ----------------------------- */

/* The cells of row @index in the three range arrays, then the declared range when unpainted. */
static void nxs_core_default_range(struct nxs_csi2_row *row)
{
	/* A range the device has not painted is the row's declared one. */
	if ((row->type & NXS_ROW_TYPE_RANGE) && !row->range[NXS_CSI2_STEP])
		memcpy(row->range, row->values, sizeof(row->range));
}

/* MIN, MAX and STEP of every row: the ranges in effect. */
static int nxs_core_read_ranges(struct nxs_csi2_core *core)
{
	static const u16 cells[] = { NXS_MAP_REG_MIN, NXS_MAP_REG_MAX, NXS_MAP_REG_STEP };
	u8 buf[4 * NXS_MAX_ROWS];
	unsigned int a;
	unsigned int i;
	int err;

	if (!core->param_count)
		return 0;
	for (a = 0; a < ARRAY_SIZE(cells); a++) {
		err = nxs_core_read(core, cells[a], buf, 4 * core->param_count);
		if (err)
			return err;
		for (i = 0; i < core->param_count; i++)
			core->rows[i].range[a] = nxs_le32(buf + 4 * i);
	}
	for (i = 0; i < core->param_count; i++)
		nxs_core_default_range(&core->rows[i]);

	return 0;
}

static void nxs_core_take_live(struct nxs_csi2_core *core, struct nxs_csi2_row *row, u32 live)
{
	row->live = live;
	if (row->std == NXS_PARAM_ID_TRIGGER)
		core->trigger_live = live;
}

/* LIVE of every row: the values in effect. */
static int nxs_core_read_lives(struct nxs_csi2_core *core)
{
	u8 buf[4 * NXS_MAX_ROWS];
	unsigned int i;
	int err;

	if (!core->param_count)
		return 0;
	err = nxs_core_read(core, NXS_MAP_REG_LIVE, buf, 4 * core->param_count);
	if (err)
		return err;
	for (i = 0; i < core->param_count; i++)
		nxs_core_take_live(core, &core->rows[i], nxs_le32(buf + 4 * i));

	return 0;
}

/* The range in effect of one row, into its range[]. */
int nxs_csi2_core_read_range(struct nxs_csi2_core *core, unsigned int index)
{
	static const u16 cells[] = { NXS_MAP_REG_MIN, NXS_MAP_REG_MAX, NXS_MAP_REG_STEP };
	struct nxs_csi2_row *row;
	u8 buf[4];
	unsigned int a;
	int err;

	if (index >= core->param_count)
		return -EINVAL;
	row = &core->rows[index];
	for (a = 0; a < ARRAY_SIZE(cells); a++) {
		err = nxs_core_read(core, cells[a] + 4 * index, buf, sizeof(buf));
		if (err)
			return err;
		row->range[a] = nxs_le32(buf);
	}
	nxs_core_default_range(row);

	return 0;
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_read_range);

/* The value in effect of one row, into its live field. */
int nxs_csi2_core_read_live(struct nxs_csi2_core *core, unsigned int index)
{
	u8 buf[4];
	int err;

	if (index >= core->param_count)
		return -EINVAL;
	err = nxs_core_read(core, NXS_MAP_REG_LIVE + 4 * index, buf, sizeof(buf));
	if (err)
		return err;
	nxs_core_take_live(core, &core->rows[index], nxs_le32(buf));

	return 0;
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_read_live);

/*
 * A range row's range as a control's: signed by the row's TYPE, an INTEGER
 * control's bounds inside s32, the default inside the range.
 */
void nxs_csi2_core_ctrl_range(const struct nxs_csi2_row *row, bool s32_only, s64 *min,
			      s64 *max, u64 *step, s64 *def)
{
	*min = nxs_csi2_row_s64(row, row->range[NXS_CSI2_MIN]);
	*max = nxs_csi2_row_s64(row, row->range[NXS_CSI2_MAX]);
	if (s32_only) {
		*min = clamp_t(s64, *min, S32_MIN, S32_MAX);
		*max = clamp_t(s64, *max, S32_MIN, S32_MAX);
	}
	if (*max < *min)
		*max = *min;
	*step = row->range[NXS_CSI2_STEP] ? row->range[NXS_CSI2_STEP] : 1;
	*def = clamp_t(s64, nxs_csi2_row_s64(row, row->def), *min, *max);
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_ctrl_range);

/*
 * The control value a core row's LIVE cell reads as. False for a value the
 * control cannot show: an enum value off the row's list.
 */
static bool nxs_core_live_to_ctrl(const struct nxs_csi2_row *row, u32 live, s64 *val)
{
	int i;

	if (row->std == NXS_PARAM_ID_TRIGGER) {
		*val = live;
		return live < NXS_CORE_TRIGGER_MENU_MAX && nxs_core_row_find(row, live) >= 0;
	}
	if (row->type & NXS_ROW_TYPE_RANGE) {
		*val = nxs_csi2_row_s64(row, live);
		return true;
	}
	i = nxs_core_row_find(row, live);
	*val = i;

	return i >= 0;
}

/* A control takes a value without a commit; a volatile or grabbed one keeps its own. */
static void nxs_core_sync_ctrl(struct v4l2_ctrl *ctrl, s64 val)
{
	if (ctrl->flags & (V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_GRABBED))
		return;
	if (ctrl->type == V4L2_CTRL_TYPE_INTEGER64)
		__v4l2_ctrl_s_ctrl_int64(ctrl, val);
	else
		__v4l2_ctrl_s_ctrl(ctrl, clamp_t(s64, val, S32_MIN, S32_MAX));
}

/*
 * The core's controls follow the rows: a range control's range and, for a
 * row not held, its value in effect. Never called inside a set: a value
 * written into a control there would replace the value the same request
 * brings it.
 */
static void nxs_core_sync_controls(struct nxs_csi2_core *core)
{
	unsigned int i;

	core->syncing = true;
	for (i = 0; i < core->param_count; i++) {
		struct nxs_csi2_row *row = &core->rows[i];
		struct v4l2_ctrl *ctrl = row->ctrl;
		s64 min, max, def;
		u64 step;
		s64 val;

		if (!ctrl || !nxs_core_owns(row))
			continue;
		if (row->type & NXS_ROW_TYPE_RANGE) {
			nxs_csi2_core_ctrl_range(row, ctrl->type != V4L2_CTRL_TYPE_INTEGER64, &min,
						 &max, &step, &def);
			__v4l2_ctrl_modify_range(ctrl, min, max, step, def);
		}
		if ((core->held & BIT_ULL(i)) || !nxs_core_live_to_ctrl(row, row->live, &val))
			continue;
		nxs_core_sync_ctrl(ctrl, val);
	}
	core->syncing = false;
}

/*
 * After a run the map repaints every range and LIVE: the rows follow, and
 * the core's controls with them. The adapter then reads its own rows'
 * range and live fields into its controls.
 */
int nxs_csi2_core_refresh(struct nxs_csi2_core *core)
{
	int err;

	err = nxs_core_read_ranges(core);
	if (err)
		return err;
	err = nxs_core_read_lives(core);
	if (err)
		return err;
	nxs_core_sync_controls(core);

	return 0;
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_refresh);

/* ---- the commits ----------------------------------------------------- */

/*
 * The rows a trigger pulse governs under @trigger: the rate under any
 * conversion other than free run, and under the fast trigger an exposure row
 * that says so, since the pulse width is the exposure there. A sequential
 * trigger exposes at the row's value.
 */
static u64 nxs_core_pulse_rows(const struct nxs_csi2_core *core, u32 trigger)
{
	u64 mask = 0;
	int i;

	if (trigger == NXS_TRIGGER_CODE_FREERUN)
		return 0;
	i = core->row_of[NXS_PARAM_ID_FRAME_RATE_MFPS];
	if (i >= 0)
		mask |= BIT_ULL(i);
	i = core->row_of[NXS_PARAM_ID_EXPOSURE_US];
	if (trigger == NXS_TRIGGER_CODE_FAST && i >= 0 &&
	    (core->rows[i].flags & NXS_ROW_FLAG_SET_BY_TRIGGER))
		mask |= BIT_ULL(i);

	return mask;
}

/*
 * The rows the pulse governs leave @mask under the conversion the trigger
 * row's LIVE names, since the host tool converts the trigger too.
 */
static int nxs_core_drop_pulse_rows(struct nxs_csi2_core *core, u64 *mask)
{
	int i = core->row_of[NXS_PARAM_ID_TRIGGER];
	int err;

	if (i < 0 || !(*mask & nxs_core_pulse_rows(core, NXS_TRIGGER_CODE_FAST)))
		return 0;
	err = nxs_csi2_core_read_live(core, i);
	if (err)
		return err;
	*mask &= ~nxs_core_pulse_rows(core, core->trigger_live);

	return 0;
}

/* Row @index's value for its next commit. */
void nxs_csi2_core_stage(struct nxs_csi2_core *core, unsigned int index, u32 value)
{
	if (index < NXS_MAX_ROWS)
		core->staged[index] = value;
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_stage);

/*
 * The rows of @mask committed at their staged values, one commit per kind,
 * the rows a trigger pulse governs left out. A map that reads INERT takes
 * no commit: the rows wait for nxs_csi2_core_commit_held() and the call
 * returns NXS_CSI2_INERT.
 */
int nxs_csi2_core_commit(struct nxs_csi2_core *core, u64 mask, const char *what)
{
	int trigger = core->row_of[NXS_PARAM_ID_TRIGGER];
	u64 all = mask;
	int err;

	if (!mask)
		return 0;
	if (!core->session) {
		core->held |= mask;
		return 0;
	}
	err = nxs_core_drop_pulse_rows(core, &mask);
	if (err)
		return err;
	while (mask) {
		u8 kind = core->rows[__ffs64(mask)].kind;
		u64 sub = 0;
		unsigned int i;

		for (i = 0; i < core->param_count; i++)
			if ((mask & BIT_ULL(i)) && core->rows[i].kind == kind)
				sub |= BIT_ULL(i);
		err = nxs_core_commit_one(core, sub, kind, what);
		if (err == NXS_CSI2_INERT) {
			core->held |= mask;
			return NXS_CSI2_INERT;
		}
		if (err)
			return err;
		mask &= ~sub;
		if (trigger >= 0 && (sub & BIT_ULL(trigger)))
			core->trigger_live = core->staged[trigger];
	}
	core->held &= ~all;

	return 0;
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_commit);

/*
 * The rows staged while the map was INERT. A trigger conversion commits
 * first, since the rows the pulse governs leave the commit after it.
 */
int nxs_csi2_core_commit_held(struct nxs_csi2_core *core)
{
	int i = core->row_of[NXS_PARAM_ID_TRIGGER];
	int err;

	if (!core->held)
		return 0;
	if (i >= 0 && (core->held & BIT_ULL(i))) {
		err = nxs_csi2_core_commit(core, BIT_ULL(i), "trigger mode");
		if (err)
			return err;
	}

	return nxs_csi2_core_commit(core, core->held, "held controls");
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_commit_held);

/* The header's capture fields a session reads first. */
int nxs_csi2_core_read_status(struct nxs_csi2_core *core, struct nxs_csi2_status *status)
{
	u8 boot[NXS_CORE_STATUS_LEN];
	int err;

	err = nxs_core_read(core, NXS_MAP_REG_MAP_ID, boot, sizeof(boot));
	if (err)
		return err;
	status->state = boot[NXS_MAP_REG_STATE];
	status->stream = boot[NXS_MAP_REG_STREAM];
	status->current_mode = boot[NXS_MAP_REG_CURRENT_MODE];
	status->param_count = boot[NXS_MAP_REG_PARAM_COUNT];
	status->mode_count = boot[NXS_MAP_REG_MODE_COUNT];

	return 0;
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_read_status);

/*
 * Whether the map differs from the one the core read: another personality
 * uploaded into the pod behind the door changes its row count, its mode
 * count or its mode table's bytes. `status` is the header just read; the
 * table is read again only where the counts agree.
 */
int nxs_csi2_core_changed(struct nxs_csi2_core *core, const struct nxs_csi2_status *status,
			  bool *changed)
{
	u8 table[NXS_MODE_TABLE_MAX * NXS_MODE_ENTRY_SIZE];
	size_t len = core->mode_count * NXS_MODE_ENTRY_SIZE;
	int err;

	*changed = status->param_count != core->param_count ||
		   status->mode_count != core->mode_count;
	if (*changed || !len)
		return 0;
	err = nxs_core_read(core, NXS_MAP_REG_MODE_TABLE, table, len);
	if (err)
		return err;
	*changed = memcmp(table, core->table, len) != 0;

	return 0;
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_changed);

/*
 * The stream gate: a GATE commit of the STREAM row, which starts or parks
 * the sensor. 0 when the map serves no such row, NXS_CSI2_INERT when it
 * has not configured its sensor; the gate is never held.
 */
int nxs_csi2_core_gate(struct nxs_csi2_core *core, bool on)
{
	int i = core->row_of[NXS_PARAM_ID_STREAM];

	if (i < 0)
		return 0;
	core->staged[i] = on;

	return nxs_core_commit_one(core, BIT_ULL(i), core->rows[i].kind,
				   on ? "stream on" : "stream off");
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_gate);

/* A sensor the host tool left streaming parks, so a receiver finds its lanes stopped. */
int nxs_csi2_core_park(struct nxs_csi2_core *core)
{
	struct nxs_csi2_status status;
	int err;

	err = nxs_csi2_core_read_status(core, &status);
	if (err)
		return err;
	if (status.state == NXS_MAP_STATE_INERT || !status.stream)
		return 0;
	err = nxs_csi2_core_gate(core, false);

	return err == NXS_CSI2_INERT ? 0 : err;
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_park);

/*
 * A RUN commit of the MODE row at @mode, in standby: a streaming sensor
 * parks first. The run is the sensor's configure, which starts its stream,
 * so with @park the sensor returns to standby after it. The map repaints
 * every range and LIVE: the caller refreshes. An INERT map takes no run and
 * the call returns NXS_CSI2_INERT.
 */
int nxs_csi2_core_run_mode(struct nxs_csi2_core *core, const struct nxs_csi2_mode *mode,
			   bool park)
{
	int i = core->row_of[NXS_PARAM_ID_MODE];
	struct nxs_csi2_status status;
	int err;

	if (i < 0)
		return -ENODEV;
	err = nxs_csi2_core_read_status(core, &status);
	if (err)
		return err;
	if (status.state == NXS_MAP_STATE_INERT)
		return NXS_CSI2_INERT;
	if (status.stream) {
		err = nxs_csi2_core_gate(core, false);
		if (err)
			return err;
	}
	core->staged[i] = mode->value;
	err = nxs_core_commit_one(core, BIT_ULL(i), core->rows[i].kind, "mode");
	if (err)
		return err;
	if (park)
		return nxs_csi2_core_gate(core, false);

	return 0;
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_run_mode);

/* ---- the core's controls --------------------------------------------- */

/*
 * The value a core row takes from a set of its control, in the row's unit:
 * a menu index as the row's value at that index, the trigger menu's index as
 * the trigger code, a range control's value as it is.
 */
static u32 nxs_core_row_value(const struct nxs_csi2_row *row)
{
	const struct v4l2_ctrl *ctrl = row->ctrl;
	s64 val = ctrl->type == V4L2_CTRL_TYPE_INTEGER64 ? *ctrl->p_new.p_s64 : ctrl->val;

	if (row->std == NXS_PARAM_ID_TRIGGER || (row->type & NXS_ROW_TYPE_RANGE))
		return (u32)val;

	return val >= 0 && val < row->count ? row->values[val] : 0;
}

static int nxs_core_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct nxs_csi2_row *row = ctrl->priv;
	struct nxs_csi2_core *core = row->core;
	int err;

	/* The handler's lock is the adapter's, held by the framework. A control whose row left the map sets nothing. */
	if (!core || core->syncing)
		return 0;
	nxs_csi2_core_stage(core, row->index, nxs_core_row_value(row));
	err = nxs_csi2_core_commit(core, BIT_ULL(row->index), ctrl->name);

	/* The map takes a commit once its sensor runs: the value waits. */
	return err == NXS_CSI2_INERT ? 0 : err;
}

/* A read-only row, and the trigger the host tool may convert, read LIVE. */
static int nxs_core_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct nxs_csi2_row *row = ctrl->priv;
	s64 val;
	int err;

	if (!row->core)
		return 0;
	err = nxs_csi2_core_read_live(row->core, row->index);
	if (err)
		return err;
	if (!nxs_core_live_to_ctrl(row, row->live, &val))
		return 0;
	if (ctrl->type == V4L2_CTRL_TYPE_INTEGER64)
		*ctrl->p_new.p_s64 = val;
	else
		ctrl->val = clamp_t(s64, val, S32_MIN, S32_MAX);

	return 0;
}

static const struct v4l2_ctrl_ops nxs_core_ctrl_ops = {
	.g_volatile_ctrl = nxs_core_g_volatile_ctrl,
	.s_ctrl = nxs_core_s_ctrl,
};

/* The test pattern menu: the row's codes in the row's order, code 0 named Disabled. */
static int nxs_core_new_test_pattern(struct nxs_csi2_core *core, struct v4l2_ctrl_handler *hdl,
				     struct nxs_csi2_row *row)
{
	struct device *dev = &core->client->dev;
	const char **menu;
	unsigned int i;

	menu = devm_kcalloc(dev, row->count, sizeof(*menu), GFP_KERNEL);
	if (!menu)
		return -ENOMEM;
	for (i = 0; i < row->count; i++) {
		menu[i] = row->values[i] ?
			  devm_kasprintf(dev, GFP_KERNEL, "Pattern %u", row->values[i]) :
			  "Disabled";
		if (!menu[i])
			return -ENOMEM;
	}
	row->ctrl = v4l2_ctrl_new_std_menu_items(hdl, &nxs_core_ctrl_ops, V4L2_CID_TEST_PATTERN,
						 row->count - 1, 0,
						 max_t(int, nxs_core_row_find(row, row->def), 0),
						 menu);
	if (row->ctrl)
		row->ctrl->priv = row;

	return 0;
}

/*
 * The trigger menu: one entry per TriggerCode, an entry the row lacks
 * skipped. It reads LIVE back, so a conversion the host tool made shows.
 * Its writes execute every time, as a volatile control's value never
 * counts as changed.
 */
static int nxs_core_new_trigger(struct nxs_csi2_core *core, struct v4l2_ctrl_handler *hdl,
				struct nxs_csi2_row *row)
{
	struct device *dev = &core->client->dev;
	struct v4l2_ctrl_config cfg = {
		.ops = &nxs_core_ctrl_ops,
		.id = NXS_CSI2_CID_TRIGGER_MODE(core->cid_base),
		.name = "Trigger Mode",
		.type = V4L2_CTRL_TYPE_MENU,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	};
	const char **menu;
	u64 codes = 0;
	unsigned int i;

	for (i = 0; i < row->count; i++) {
		if (row->values[i] < NXS_CORE_TRIGGER_MENU_MAX)
			codes |= BIT_ULL(row->values[i]);
		else
			dev_warn(dev, "trigger code %u: past the menu, not offered\n",
				 row->values[i]);
	}
	if (!codes)
		return 0;
	cfg.min = __ffs64(codes);
	cfg.max = fls64(codes) - 1;
	cfg.menu_skip_mask = ~codes & GENMASK_ULL(cfg.max, 0);
	cfg.def = row->def < NXS_CORE_TRIGGER_MENU_MAX && (codes & BIT_ULL(row->def)) ?
		  row->def : cfg.min;
	menu = devm_kcalloc(dev, cfg.max + 1, sizeof(*menu), GFP_KERNEL);
	if (!menu)
		return -ENOMEM;
	for (i = 0; i <= cfg.max; i++) {
		if (!(codes & BIT_ULL(i)))
			menu[i] = "";
		else if (i < ARRAY_SIZE(nxs_core_trigger_names))
			menu[i] = nxs_core_trigger_names[i];
		else
			menu[i] = devm_kasprintf(dev, GFP_KERNEL, "Trigger %u", i);
		if (!menu[i])
			return -ENOMEM;
	}
	cfg.qmenu = menu;
	row->ctrl = v4l2_ctrl_new_custom(hdl, &cfg, row);

	return 0;
}

/*
 * A control named by the row: an integer at the row's range and step, or an
 * integer menu of its values. An unsigned range past s32 is a 64-bit
 * integer. A read-only row is a volatile control that reads LIVE. The black
 * level and the temperature carry their own ids and names.
 */
static int nxs_core_new_custom(struct nxs_csi2_core *core, struct v4l2_ctrl_handler *hdl,
			       struct nxs_csi2_row *row)
{
	struct device *dev = &core->client->dev;
	struct v4l2_ctrl_config cfg = {
		.ops = &nxs_core_ctrl_ops,
		.id = NXS_CSI2_CID_ROW(core->cid_base, row->index),
		.name = row->name,
		.type = V4L2_CTRL_TYPE_INTEGER,
	};
	unsigned int i;
	s64 *menu;

	if (row->std == NXS_PARAM_ID_BLACK_LEVEL_DN) {
		cfg.id = NXS_CSI2_CID_BLACK_LEVEL(core->cid_base);
		cfg.name = "Black Level";
	} else if (row->std == NXS_PARAM_ID_TEMPERATURE_DC) {
		cfg.id = NXS_CSI2_CID_TEMPERATURE(core->cid_base);
		cfg.name = "Temperature";
	}
	if (nxs_csi2_row_read_only(row))
		cfg.flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE;
	if (row->type & NXS_ROW_TYPE_RANGE) {
		if (!(row->type & NXS_ROW_TYPE_SIGNED) &&
		    max(row->range[NXS_CSI2_MAX], row->values[NXS_CSI2_MAX]) > S32_MAX)
			cfg.type = V4L2_CTRL_TYPE_INTEGER64;
		nxs_csi2_core_ctrl_range(row, cfg.type == V4L2_CTRL_TYPE_INTEGER, &cfg.min,
					 &cfg.max, &cfg.step, &cfg.def);
	} else {
		menu = devm_kcalloc(dev, row->count, sizeof(*menu), GFP_KERNEL);
		if (!menu)
			return -ENOMEM;
		for (i = 0; i < row->count; i++)
			menu[i] = nxs_csi2_row_s64(row, row->values[i]);
		cfg.type = V4L2_CTRL_TYPE_INTEGER_MENU;
		cfg.max = row->count - 1;
		cfg.def = max_t(int, nxs_core_row_find(row, row->def), 0);
		cfg.qmenu_int = menu;
	}
	row->ctrl = v4l2_ctrl_new_custom(hdl, &cfg, row);

	return 0;
}

/*
 * The core's controls on the adapter's handler, one per row the core
 * offers: the trigger menu, the test pattern menu, the black level, the
 * temperature and a control named by each custom row. A row no driver
 * offers is logged. The adapter creates its own controls for the rows it
 * maps to its framework's.
 */
int nxs_csi2_core_init_controls(struct nxs_csi2_core *core, struct v4l2_ctrl_handler *hdl,
				u32 cid_base)
{
	struct device *dev = &core->client->dev;
	unsigned int i;
	int err = 0;

	core->cid_base = cid_base;
	for (i = 0; i < core->param_count; i++) {
		struct nxs_csi2_row *row = &core->rows[i];

		if (!nxs_core_row_offered(row)) {
			dev_warn(dev, "row %u (%s): id %u, kind %u, not offered\n", i, row->name,
				 row->id, row->kind);
			continue;
		}
		if (!nxs_core_owns(row))
			continue;
		switch (row->std) {
		case NXS_PARAM_ID_TEST_PATTERN:
			err = nxs_core_new_test_pattern(core, hdl, row);
			break;
		case NXS_PARAM_ID_TRIGGER:
			err = nxs_core_new_trigger(core, hdl, row);
			break;
		default:
			err = nxs_core_new_custom(core, hdl, row);
			break;
		}
		if (err)
			return err;
	}

	return hdl->error;
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_init_controls);

/* ---- the map at init ------------------------------------------------- */

/*
 * The header's capture fields, the mode table, the rows, their ranges and
 * their values in effect. Nothing is written. A map that does not answer
 * returns the bus error, which nxs_csi2_core_unanswered() tells; a map that is
 * not a camera door of the layout this core knows returns -ENODEV; a map
 * that serves no mode yet, a pod whose camera personality has not loaded,
 * returns -ENOENT. The modes stand as the map serves them: the adapter
 * offers the ones its receiver carries.
 */
int nxs_csi2_core_init(struct nxs_csi2_core *core, struct i2c_client *client)
{
	struct device *dev = &client->dev;
	u8 boot[NXS_CORE_HEADER_LEN];
	u8 buf[NXS_ROW_SIZE];
	unsigned int i;
	int err;

	core->client = client;
	core->read = false;
	err = nxs_core_read(core, NXS_MAP_REG_MAP_ID, boot, sizeof(boot));
	if (err)
		return err;
	if (boot[NXS_MAP_REG_MAP_ID] != NXS_MAP_ID_CAMERA) {
		dev_err(dev, "no camera map at 0x%02x: id 0x%02x%s\n", client->addr,
			boot[NXS_MAP_REG_MAP_ID],
			boot[NXS_MAP_REG_MAP_ID] == NXS_MAP_ID_POD ? ", the pod door" : "");
		return -ENODEV;
	}
	if (boot[NXS_MAP_REG_MAP_VERSION] != NXS_MAP_VERSION_VALUE) {
		dev_err(dev, "camera map at 0x%02x: layout version %u, this driver knows %u\n",
			client->addr, boot[NXS_MAP_REG_MAP_VERSION], NXS_MAP_VERSION_VALUE);
		return -ENODEV;
	}
	core->param_count = boot[NXS_MAP_REG_PARAM_COUNT];
	core->mode_count = boot[NXS_MAP_REG_MODE_COUNT];
	core->module_kind = boot[NXS_MAP_REG_MODULE_KIND];
	core->fw_major = boot[NXS_MAP_REG_FW_VERSION_MAJOR];
	core->fw_minor = boot[NXS_MAP_REG_FW_VERSION_MINOR];
	if (core->param_count > NXS_MAX_ROWS || core->mode_count > NXS_MODE_TABLE_MAX) {
		dev_err(dev, "camera map at 0x%02x: %u rows, %u modes\n", client->addr,
			core->param_count, core->mode_count);
		return -ENODEV;
	}
	if (!core->mode_count) {
		dev_dbg(dev, "camera map at 0x%02x serves no mode yet\n", client->addr);
		return -ENOENT;
	}
	dev_info(dev, "camera map v%u at 0x%02x: %s, interface %u, firmware %u.%u, %u rows, %u modes, %s\n",
		 boot[NXS_MAP_REG_MAP_VERSION], client->addr,
		 nxs_core_module_kind_name(core->module_kind), boot[NXS_MAP_REG_PROTO_VERSION],
		 core->fw_major, core->fw_minor, core->param_count, core->mode_count,
		 nxs_csi2_core_state_name(boot[NXS_MAP_REG_STATE]));
	err = nxs_core_read_modes(core);
	if (err)
		return err;

	memset(core->row_of, -1, sizeof(core->row_of));
	core->held = 0;
	core->trigger_live = NXS_TRIGGER_CODE_FREERUN;
	if (!core->rows) {
		core->rows = devm_kcalloc(dev, NXS_MAX_ROWS, sizeof(*core->rows), GFP_KERNEL);
		if (!core->rows)
			return -ENOMEM;
	}
	for (i = 0; i < core->param_count; i++) {
		struct nxs_csi2_row *row = &core->rows[i];

		err = nxs_core_read(core, NXS_MAP_REG_ROWS + i * NXS_ROW_SIZE, buf, sizeof(buf));
		if (err)
			return err;
		nxs_core_parse_row(row, i, buf);
		row->core = core;
		if (!nxs_core_row_offered(row))
			continue;
		row->std = nxs_core_std(core, row);
		if (row->std)
			core->row_of[row->std] = i;
	}
	err = nxs_core_read_ranges(core);
	if (err)
		return err;
	err = nxs_core_read_lives(core);
	if (err)
		return err;
	core->read = true;

	return 0;
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_init);

MODULE_DESCRIPTION("The NXS camera map, for the drivers that serve it");
MODULE_AUTHOR("Aliensense");
MODULE_LICENSE("GPL");

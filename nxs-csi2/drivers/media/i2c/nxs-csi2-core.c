// SPDX-License-Identifier: GPL-2.0-only
/*
 * nxs-csi2-core.c - a camera door of the NXS Device Interface, for the drivers that serve it
 *
 * Copyright (C) 2026 Aliensense
 *
 * One core for the two adapters: nxs-csi2, the V4L2 sub-device driver, and
 * nxs-csi2-tegra, the tegracam driver. The core knows the interface and no
 * sensor. A pod, an NXS Camera or any device of the interface serves a
 * camera door, the driver door of its camera personality: a 64 KB map
 * behind a 16-bit register pointer, high byte first. The core reads the
 * header's cells in one latched read, finds the CELLS and DESCRIPTION
 * regions in its region table, and reads the description through the paged
 * window: the camera personality's PARAMS rows, their ROW_VALUES lists and
 * the mode table of its CAMERA_MAP record. The header names no payload: the
 * core reads the stream and the mode in the LIVE cells of the STREAM and
 * MODE rows, and a row's id is a family byte above an id within the family. It commits
 * the rows in their units, and the device converts the values to its
 * sensor's registers. The constants are nxs-csi2-map.h, generated from the
 * interface's one source (redbrain constants/nxs_map.yaml). The contract
 * is the NXS Device Interface manual, and the drivers' sequence is
 * docs/bring-up/nxs-csi2-mainline.md in the redbrain tree.
 *
 * The commit (§8 of the manual). The core reads STATE and DESCRIPTOR_EPOCH
 * in one header read under the host bus lock, since a COMMIT written while
 * BUSY or INERT is ignored and a description that moved renumbers the
 * rows. It keeps the lock until it has read STATE past the device's bound.
 * It reads the rows' STEP cells and leaves out a row whose STEP reads 0,
 * which takes no value in the state in effect (§7). It writes the STAGED
 * cells of the rest, adjacent rows in one write, and the device marks each
 * row written and paints RUN_MS for the marked set. The core reads RUN_MS,
 * writes COMMIT 1 and sends nothing for RUN_MS. Then it reads STATE once;
 * BUSY then is the device's failure. A RUN_MS of 0 for a marked set is the
 * device's failure too. That, and a staging write the bus cut, end with
 * COMMIT 2, which discards the marked set.
 * Every transfer is one try: the device answers outside its bound, so a
 * NAK is a silent address or a fault. The core commits only while the
 * adapter has a capture session open on the link (nxs_csi2_core_session):
 * the host's tool brings the port up and joins its links while no session
 * is open, and a control set meanwhile, tegracam's defaults at probe among
 * them, waits in the held rows for the session's start.
 */

#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/build_bug.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/lockdep.h>
#include <linux/media-bus-format.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/stringify.h>

#include <media/v4l2-ctrls.h>

#include "nxs-csi2-core.h"

/*
 * The interface layout the core reads, MAJOR.MINOR: it takes a door of this
 * MAJOR and a MINOR at or below this one. The module version names it with
 * the release the core was built from.
 */
#define NXS_CORE_LAYOUT_MAJOR	1
#define NXS_CORE_LAYOUT_MINOR	3
/* One part of a RELEASE value: MAJOR, MINOR and PATCH from the low byte. */
#define NXS_RELEASE_PART(v, i)	(((v) >> (8 * (i))) & 0xff)
static_assert(NXS_RELEASE_PART(NXS_RELEASE_VALUE, 0) == NXS_CORE_LAYOUT_MAJOR &&
	      NXS_RELEASE_PART(NXS_RELEASE_VALUE, 1) == NXS_CORE_LAYOUT_MINOR,
	      "nxs-csi2-map.h names another layout: read its changes into the core");

/* The longest write: 2 pointer bytes and 256 data bytes, 64 staged cells. */
#define NXS_CORE_WRITE_MAX	256
#define NXS_CORE_CELLS_MAX	(NXS_CORE_WRITE_MAX / 4)
/* What a session reads: the header through DESCRIPTOR_EPOCH. */
#define NXS_CORE_STATUS_LEN	(NXS_MAP_REG_DESCRIPTOR_EPOCH + 1)
/* The longest test pattern menu: its maximum index is a u8. */
#define NXS_CORE_MENU_MAX	256
/* A commit's first read, STATE through DESCRIPTOR_EPOCH, and its verdict, STATE through FAULT_COUNT. */
#define NXS_CORE_IDLE_LEN	(NXS_MAP_REG_DESCRIPTOR_EPOCH + 1 - NXS_MAP_REG_STATE)
#define NXS_CORE_VERDICT_LEN	(NXS_MAP_REG_FAULT_COUNT + 1 - NXS_MAP_REG_STATE)
#define NXS_CORE_AT(reg)	((reg) - NXS_MAP_REG_STATE)
/* The description reads it whole, at most this many pages. */
#define NXS_CORE_DESC_MAX	(16 * NXS_DESC_PAGE_SIZE)
/* Reads of a description that moved under them before the core gives up. */
#define NXS_CORE_DESC_TRIES	3
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

/* The arrays of the CELLS region, in the region's order. */
enum nxs_core_array {
	NXS_CORE_LIVE = NXS_CELLS_ARRAY_LIVE,
	NXS_CORE_MIN = NXS_CELLS_ARRAY_MIN,
	NXS_CORE_MAX = NXS_CELLS_ARRAY_MAX,
	NXS_CORE_STEP = NXS_CELLS_ARRAY_STEP,
	NXS_CORE_STAGED = NXS_CELLS_ARRAY_STAGED,
};

/* The address of row @index's cell in @array: the array at 4 C each, the cell at 4 i. */
static u16 nxs_core_cell(const struct nxs_csi2_core *core, enum nxs_core_array array,
			 unsigned int index)
{
	return core->cells + 4 * (array * core->capacity + index);
}

/* ---- the door ---------------------------------------------------------- */

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
	u8 out[2 + NXS_CORE_WRITE_MAX];
	struct i2c_msg msgs[2] = {
		{ .addr = client->addr, .flags = 0, .len = 2 + wlen, .buf = out },
		{ .addr = client->addr, .flags = I2C_M_RD, .len = rlen, .buf = rbuf },
	};
	int n = rlen ? 2 : 1;
	int err;

	if (wlen > NXS_CORE_WRITE_MAX || rlen > U16_MAX)
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

/* The longest read the adapter takes after its write of the pointer. */
static size_t nxs_core_read_max(const struct i2c_adapter *adap)
{
	const struct i2c_adapter_quirks *q = adap->quirks;
	size_t max = U16_MAX;

	if (q && q->max_read_len)
		max = min_t(size_t, max, q->max_read_len);
	if (q && (q->flags & I2C_AQ_COMB) && q->max_comb_2nd_msg_len)
		max = min_t(size_t, max, q->max_comb_2nd_msg_len);

	return max;
}

/*
 * A span outside the header, read in the longest pieces the adapter takes:
 * every region outside the header serves in place, so the pieces read as
 * one. The header itself is one latched read of NXS_MAP_LATCH_SIZE bytes.
 */
static int nxs_core_read_span(struct nxs_csi2_core *core, bool locked, u16 reg, u8 *buf,
			      size_t len)
{
	size_t max = nxs_core_read_max(core->client->adapter);
	size_t done = 0;
	int err;

	while (done < len) {
		size_t n = min(len - done, max);

		err = nxs_core_xfer(core->client, locked, reg + done, NULL, 0, buf + done, n);
		if (err)
			return err;
		done += n;
	}

	return 0;
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
		return "an NXS Pod";
	case NXS_MODULE_KIND_FIXED:
		return "an NXS Camera";
	case NXS_MODULE_KIND_MODULE:
		return "an NXS Module";
	default:
		return "a device of another class";
	}
}

/* ---- the windows ----------------------------------------------------- */

/*
 * A commit's first read: STATE and DESCRIPTOR_EPOCH in one header read
 * under the host bus lock, into @idle from STATE on. A COMMIT written while
 * BUSY or INERT is ignored, and an epoch that moved since the description
 * was read renumbers the rows. The lock is kept once the door reads READY
 * or FAULT under the epoch the rows were read at, so no host transfer lands
 * between the check and the commit. A door under a run in flight returns
 * -EBUSY, one that reads INERT NXS_CSI2_INERT and one whose description
 * moved -ESTALE, each with the lock released. The wire is idle when the
 * core reads, so a STATE outside the four is the device's fault, never a
 * torn read.
 */
static int nxs_core_idle(struct nxs_csi2_core *core, u8 *idle)
{
	struct i2c_client *client = core->client;
	int err;

	i2c_lock_bus(client->adapter, I2C_LOCK_SEGMENT);
	err = nxs_core_xfer(client, true, NXS_MAP_REG_STATE, NULL, 0, idle, NXS_CORE_IDLE_LEN);
	if (err)
		goto unlock;
	if (idle[0] > NXS_MAP_STATE_FAULT)
		err = -EBADMSG;
	else if (idle[0] == NXS_MAP_STATE_BUSY)
		err = -EBUSY;
	else if (idle[0] == NXS_MAP_STATE_INERT)
		err = NXS_CSI2_INERT;
	else if (idle[NXS_CORE_AT(NXS_MAP_REG_DESCRIPTOR_EPOCH)] != core->epoch)
		err = -ESTALE;
	if (!err)
		return 0;
unlock:
	i2c_unlock_bus(client->adapter, I2C_LOCK_SEGMENT);

	return err;
}

/* COMMIT 2 under the held bus: the marked set is cleared and nothing else changes. */
static void nxs_core_discard(struct nxs_csi2_core *core, const char *what)
{
	const u8 discard = NXS_COMMIT_DISCARD;
	int err;

	err = nxs_core_xfer(core->client, true, NXS_MAP_REG_COMMIT, &discard, 1, NULL, 0);
	if (err)
		dev_warn_ratelimited(&core->client->dev, "%s: discarding the staged rows: bus error %d\n",
				     what, err);
}

/*
 * The window after a COMMIT, the bus held: the host sends nothing for the
 * RUN_MS the door painted for the marked set, then reads STATE once, with
 * ERROR and the counts in the same read. The painted time carries the
 * device's own lateness, so BUSY then stands.
 */
static int nxs_core_settle(struct i2c_client *client, u16 wait_ms, u8 *verdict)
{
	if (wait_ms >= 20)
		msleep(wait_ms);
	else if (wait_ms)
		usleep_range(wait_ms * 1000, wait_ms * 1000 + 200);

	return nxs_core_xfer(client, true, NXS_MAP_REG_STATE, NULL, 0, verdict,
			     NXS_CORE_VERDICT_LEN);
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
 * The STAGED cells of @rows, adjacent rows in one write of at most 64
 * cells. A write marks every row whose cell it covers, so no write runs
 * through a row outside the set.
 */
static int nxs_core_stage_rows(struct nxs_csi2_core *core, const struct nxs_csi2_rows *rows)
{
	u8 buf[NXS_CORE_WRITE_MAX];
	unsigned int first = find_first_bit(rows->bits, core->param_count);
	int err;

	while (first < core->param_count) {
		unsigned int end = find_next_zero_bit(rows->bits, core->param_count, first);
		unsigned int n = min_t(unsigned int, end - first, NXS_CORE_CELLS_MAX);
		unsigned int i;

		for (i = 0; i < n; i++)
			nxs_put_le32(buf + 4 * i, core->rows[first + i].staged);
		err = nxs_core_xfer(core->client, true, nxs_core_cell(core, NXS_CORE_STAGED, first),
				    buf, 4 * n, NULL, 0);
		if (err)
			return err;
		first = find_next_bit(rows->bits, core->param_count, first + n);
	}

	return 0;
}

/*
 * The rows of @rows that take a value now, into @set. The STEP cells of
 * the rows are read under the held bus, adjacent rows in one read, into
 * each row's range. A row whose STEP reads 0 takes no value in the state
 * in effect (§7), and a commit of it ends FAULT, so it leaves the set.
 */
static int nxs_core_take_steps(struct nxs_csi2_core *core, const struct nxs_csi2_rows *rows,
			       struct nxs_csi2_rows *set)
{
	u8 buf[4 * NXS_CORE_CELLS_MAX];
	unsigned int first = find_first_bit(rows->bits, core->param_count);
	int err;

	*set = *rows;
	while (first < core->param_count) {
		unsigned int end = find_next_zero_bit(rows->bits, core->param_count, first);
		unsigned int n = min_t(unsigned int, end - first, NXS_CORE_CELLS_MAX);
		unsigned int i;

		err = nxs_core_read_span(core, true, nxs_core_cell(core, NXS_CORE_STEP, first), buf,
					 4 * n);
		if (err)
			return err;
		for (i = 0; i < n; i++) {
			struct nxs_csi2_row *row = &core->rows[first + i];

			row->range[NXS_CSI2_STEP] = nxs_le32(buf + 4 * i);
			if (nxs_csi2_row_takes_value(row))
				continue;
			__clear_bit(first + i, set->bits);
			dev_dbg(&core->client->dev, "row %u (%s) takes no value now: left out\n",
				first + i, row->name);
		}
		first = find_next_bit(rows->bits, core->param_count, first + n);
	}

	return 0;
}

/*
 * One commit of @rows, rows of one kind which the device judges: the
 * header read of STATE and the epoch, the rows' STEP cells, the STAGED
 * cells of the rows that take a value, RUN_MS read, COMMIT 1 written
 * alone, and STATE read once after the painted time. A staging write or a
 * RUN_MS read the bus cut ends with COMMIT 2, and so does a RUN_MS of 0. A
 * COMMIT write the bus cut leaves the core unsure whether the device took
 * it: it waits RUN_MS, reads STATE once, and the counts tell. The host bus
 * stays held from the first read to the last. Returns 0 with nothing
 * written when no row of the set takes a value now, NXS_CSI2_INERT when
 * the door has not configured its sensor and nothing was written, -ESTALE
 * when its description moved since the core read it, -EPROTO for a RUN_MS
 * of 0, and -EBUSY under a run in flight or BUSY past the painted time.
 */
static int nxs_core_commit_one(struct nxs_csi2_core *core, const struct nxs_csi2_rows *rows,
			       const char *what)
{
	struct i2c_client *client = core->client;
	struct device *dev = &client->dev;
	struct nxs_csi2_rows set;
	u8 idle[NXS_CORE_IDLE_LEN];
	u8 verdict[NXS_CORE_VERDICT_LEN];
	u8 ms[NXS_MAP_REG_RUN_MS_LEN] = { 0, 0 };
	const u8 apply = NXS_COMMIT_APPLY;
	bool cut;
	int err;

	if (find_first_bit(rows->bits, core->param_count) >= core->param_count)
		return 0;
	err = nxs_core_idle(core, idle);
	if (err == -ESTALE)
		dev_info_ratelimited(dev, "%s: the description moved since it was read: reading it again at the next session\n",
				     what);
	if (err)
		return err;
	err = nxs_core_take_steps(core, rows, &set);
	if (err)
		goto unlock;
	if (find_first_bit(set.bits, core->param_count) >= core->param_count) {
		i2c_unlock_bus(client->adapter, I2C_LOCK_SEGMENT);
		return 0;
	}
	err = nxs_core_stage_rows(core, &set);
	if (!err)
		err = nxs_core_xfer(client, true, NXS_MAP_REG_RUN_MS, NULL, 0, ms, sizeof(ms));
	if (!err && !nxs_le16(ms)) {
		/* A device paints RUN_MS above 0 for any set of rows it serves. */
		nxs_core_discard(core, what);
		i2c_unlock_bus(client->adapter, I2C_LOCK_SEGMENT);
		dev_warn_ratelimited(dev, "%s: the door painted no RUN_MS for the staged rows: discarded\n",
				     what);
		return -EPROTO;
	}
	if (err) {
		nxs_core_discard(core, what);
		goto unlock;
	}
	cut = nxs_core_xfer(client, true, NXS_MAP_REG_COMMIT, &apply, 1, NULL, 0) != 0;
	err = nxs_core_settle(client, nxs_le16(ms), verdict);
	if (!err && cut &&
	    verdict[NXS_CORE_AT(NXS_MAP_REG_APPLY_COUNT)] == idle[NXS_CORE_AT(NXS_MAP_REG_APPLY_COUNT)] &&
	    verdict[NXS_CORE_AT(NXS_MAP_REG_FAULT_COUNT)] == idle[NXS_CORE_AT(NXS_MAP_REG_FAULT_COUNT)]) {
		/* The counts stood still: the device never took the COMMIT. */
		nxs_core_discard(core, what);
		err = -EIO;
	}
unlock:
	i2c_unlock_bus(client->adapter, I2C_LOCK_SEGMENT);
	if (err) {
		dev_warn_ratelimited(dev, "%s: bus error %d\n", what, err);
		return err;
	}
	switch (verdict[0]) {
	case NXS_MAP_STATE_READY:
		return 0;
	case NXS_MAP_STATE_INERT:
		return NXS_CSI2_INERT;
	case NXS_MAP_STATE_FAULT:
		dev_warn_ratelimited(dev, "%s: the door refused the commit (error %u)\n",
				     what, verdict[NXS_CORE_AT(NXS_MAP_REG_ERROR)]);
		return nxs_core_errno(verdict[NXS_CORE_AT(NXS_MAP_REG_ERROR)]);
	case NXS_MAP_STATE_BUSY:
		dev_warn_ratelimited(dev, "%s: the door stayed busy past its painted bound\n", what);
		return -EBUSY;
	default:
		return -EBADMSG;
	}
}

/* ---- the rows -------------------------------------------------------- */

/* The index of @value among an enum row's values, -1 for a value off the list. */
static int nxs_core_row_find(const struct nxs_csi2_row *row, u32 value)
{
	unsigned int i;

	for (i = 0; i < row->count; i++)
		if (nxs_csi2_row_value(row, i) == value)
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

	return row->kind <= NXS_PARAM_KIND_GATE;
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
	case NXS_PARAM_ID_LINE_TIME_NS:
	case NXS_PARAM_ID_BLACK_LEVEL_DN:
	case NXS_PARAM_ID_TEST_PATTERN:
	case NXS_PARAM_ID_TRIGGER:
	case NXS_PARAM_ID_TEMPERATURE_DC:
		return true;
	default:
		return false;
	}
}

/*
 * One 64-byte row of the PARAMS record. An enum row whose values are in a
 * ROW_VALUES record (ROW_TYPE_LIST) takes them when that record is read,
 * and serves none until then.
 */
static void nxs_core_parse_row(struct nxs_csi2_row *row, unsigned int index, const u8 *buf)
{
	unsigned int i;

	memset(row, 0, sizeof(*row));
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
	if (row->type & NXS_ROW_TYPE_RANGE) {
		row->count = 3;
		for (i = 0; i < 3; i++)
			row->values[i] = nxs_le32(buf + NXS_ROW_FIELD_VALUES + 4 * i);
	} else if (row->type & NXS_ROW_TYPE_LIST) {
		row->count = 0;
	} else if (row->type & NXS_ROW_TYPE_WIDE) {
		row->count = min_t(u8, buf[NXS_ROW_FIELD_COUNT], NXS_ROW_WIDE_VALUES_MAX);
		for (i = 0; i < row->count; i++)
			row->values[i] = nxs_le32(buf + NXS_ROW_FIELD_VALUES + 4 * i);
	} else {
		row->count = min_t(u8, buf[NXS_ROW_FIELD_COUNT], NXS_ROW_VALUES_MAX);
		/* Inline 16-bit values are unsigned in a signed row too: a negative value takes the WIDE form. */
		for (i = 0; i < row->count; i++)
			row->values[i] = nxs_le16(buf + NXS_ROW_FIELD_VALUES + 2 * i);
	}
}

/*
 * The standard id a row is served as. The first row of an id the drivers
 * know becomes that id's control when its type fits and the host may write
 * it. Any other row is custom: id 0, an id of a family or past the ids the
 * drivers know, a second row of an id, or a standard row the drivers cannot
 * drive.
 */
static u16 nxs_core_std(const struct nxs_csi2_core *core, const struct nxs_csi2_row *row)
{
	bool range = row->type & NXS_ROW_TYPE_RANGE;
	bool writable = !nxs_csi2_row_read_only(row);

	if (!row->id || nxs_csi2_id_slot(row->id) < 0 || nxs_csi2_core_row(core, row->id) >= 0)
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
	case NXS_PARAM_ID_LINE_TIME_NS:
	case NXS_PARAM_ID_BLACK_LEVEL_DN:
	case NXS_PARAM_ID_TEMPERATURE_DC:
		return row->id;
	default:
		return 0;
	}
}

/* ---- the modes ------------------------------------------------------- */

/* Whether CSI-2 data type @code is a RAW one, a Bayer or a monochrome stream. */
static bool nxs_core_raw(u8 code)
{
	switch (code) {
	case NXS_CSI2_DATA_TYPE_RAW8:
	case NXS_CSI2_DATA_TYPE_RAW10:
	case NXS_CSI2_DATA_TYPE_RAW12:
	case NXS_CSI2_DATA_TYPE_RAW14:
	case NXS_CSI2_DATA_TYPE_RAW16:
		return true;
	default:
		return false;
	}
}

/*
 * The media bus code of a mode under the flips: PIXEL_PHASE XOR the FLIPS
 * bits. A mode whose capture table names no Bayer order is monochrome. 0
 * for a data type that is not RAW, or a depth or a phase the core does not
 * know.
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

	if (!nxs_core_raw(mode->data_type))
		return 0;
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

/*
 * The mode table of the camera personality's CAMERA_MAP record, as many
 * rows as the record's mode count, after the record's head and one law row
 * per mode. A mode's depth is its data type's: a mode whose data type is
 * not a RAW code the map header names has no media bus code, and is logged
 * and left out. -EBADMSG for a record too short for its modes, or one of no
 * mode or more than the table holds.
 */
static int nxs_core_parse_modes(struct nxs_csi2_core *core, const u8 *rec, size_t len)
{
	const u8 *table;
	unsigned int n = 0;
	unsigned int i;
	u8 count;

	if (len < NXS_CAMERA_MAP_HEAD_SIZE)
		return -EBADMSG;
	count = rec[NXS_CAMERA_MAP_FIELD_MODE_COUNT];
	if (!count || count > NXS_MODE_TABLE_MAX)
		return -EBADMSG;
	if (len < NXS_CAMERA_MAP_HEAD_SIZE + count * (NXS_CAMERA_LAW_ROW_SIZE + NXS_MODE_ENTRY_SIZE))
		return -EBADMSG;
	table = rec + NXS_CAMERA_MAP_HEAD_SIZE + count * NXS_CAMERA_LAW_ROW_SIZE;
	for (i = 0; i < count; i++) {
		const u8 *row = table + i * NXS_MODE_ENTRY_SIZE;
		struct nxs_csi2_mode *mode = &core->modes[n];
		u8 code = row[NXS_MODE_ENTRY_DATA_TYPE];

		if (!nxs_core_raw(code)) {
			dev_warn(&core->client->dev, "mode %u: CSI-2 data type 0x%02x, no media bus code for it, not offered\n",
				 row[NXS_MODE_ENTRY_MODE_VALUE], code);
			continue;
		}
		mode->width = nxs_le16(row + NXS_MODE_ENTRY_WIDTH);
		mode->height = nxs_le16(row + NXS_MODE_ENTRY_HEIGHT);
		mode->data_type = code;
		mode->bit_depth = nxs_csi2_data_type_sample_bits(code);
		mode->lanes = row[NXS_MODE_ENTRY_LANES];
		mode->rate_mbps = nxs_le16(row + NXS_MODE_ENTRY_RATE_MBPS);
		mode->flags = row[NXS_MODE_ENTRY_FLAGS];
		mode->fps_max_milli = nxs_le32(row + NXS_MODE_ENTRY_FPS_MAX_MILLI);
		mode->value = row[NXS_MODE_ENTRY_MODE_VALUE];
		mode->phase = row[NXS_MODE_ENTRY_PIXEL_PHASE];
		n++;
	}
	core->mode_count = n;

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

/*
 * Device-tree mode @i against the camera map. The tree carries the port's
 * pool, every camera the port may boot, and the map the one camera behind
 * this door: a mode the map does not serve is another camera's and is
 * logged. A rate past the map's top warns only past
 * NXS_CSI2_FPS_TOLERANCE_MILLI, since the tree rounds the rate it states.
 * Returns the map's mode, NULL for none.
 */
const struct nxs_csi2_mode *nxs_csi2_core_check_tree_mode(const struct nxs_csi2_core *core,
							  unsigned int i, u32 width, u32 height,
							  u8 bit_depth, u64 fps_milli)
{
	struct device *dev = &core->client->dev;
	const struct nxs_csi2_mode *mode;
	u64 top;

	mode = nxs_csi2_core_find_geometry(core, width, height, bit_depth);
	if (!mode) {
		dev_info(dev, "mode%u: %ux%u at %u bits is in the device tree and not in the camera map: another camera of the port's pool\n",
			 i, width, height, bit_depth);
		return NULL;
	}
	top = mode->fps_max_milli;
	if (!top || fps_milli <= top)
		return mode;
	if (fps_milli - top > NXS_CSI2_FPS_TOLERANCE_MILLI)
		dev_warn(dev, "mode%u: %llu.%03llu fps in the device tree, the camera map tops at %u.%03u\n",
			 i, fps_milli / 1000, fps_milli % 1000, mode->fps_max_milli / 1000,
			 mode->fps_max_milli % 1000);
	else
		dev_info(dev, "mode%u: %llu.%03llu fps in the device tree, the camera map tops at %u.%03u\n",
			 i, fps_milli / 1000, fps_milli % 1000, mode->fps_max_milli / 1000,
			 mode->fps_max_milli % 1000);

	return mode;
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_check_tree_mode);

/* The tree's mode count against the map's: the pool's count differs by design, so it is logged. */
void nxs_csi2_core_check_tree_count(const struct nxs_csi2_core *core, unsigned int tree_modes)
{
	if (tree_modes != core->mode_count)
		dev_info(&core->client->dev, "the device tree names %u modes, the camera map serves %u\n",
			 tree_modes, core->mode_count);
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_check_tree_count);

/* ---- the ranges and the values in effect ----------------------------- */

/*
 * The cells of every row in @array, 64 to a read, into each row's u32 at
 * byte @field of its struct: LIVE into live, MIN, MAX and STEP into range.
 */
static int nxs_core_read_array(struct nxs_csi2_core *core, enum nxs_core_array array,
			       size_t field)
{
	u8 buf[4 * NXS_CORE_CELLS_MAX];
	unsigned int first;
	unsigned int i;
	int err;

	for (first = 0; first < core->param_count; first += NXS_CORE_CELLS_MAX) {
		unsigned int n = min_t(unsigned int, core->param_count - first, NXS_CORE_CELLS_MAX);

		err = nxs_core_read_span(core, false, nxs_core_cell(core, array, first), buf, 4 * n);
		if (err)
			return err;
		for (i = 0; i < n; i++)
			*(u32 *)((u8 *)&core->rows[first + i] + field) = nxs_le32(buf + 4 * i);
	}

	return 0;
}

/*
 * MIN, MAX and STEP of every row: the ranges in effect, as the device
 * paints them. A STEP of 0 says the row takes no value now.
 */
static int nxs_core_read_ranges(struct nxs_csi2_core *core)
{
	unsigned int a;
	int err;

	for (a = NXS_CSI2_MIN; a <= NXS_CSI2_STEP; a++) {
		err = nxs_core_read_array(core, NXS_CORE_MIN + a,
					  offsetof(struct nxs_csi2_row, range) + a * sizeof(u32));
		if (err)
			return err;
	}

	return 0;
}

/* LIVE of every row: the values in effect. */
static int nxs_core_read_lives(struct nxs_csi2_core *core)
{
	return nxs_core_read_array(core, NXS_CORE_LIVE, offsetof(struct nxs_csi2_row, live));
}

/* The range in effect of one row, into its range[]. */
int nxs_csi2_core_read_range(struct nxs_csi2_core *core, unsigned int index)
{
	struct nxs_csi2_row *row;
	u8 buf[4];
	unsigned int a;
	int err;

	if (index >= core->param_count)
		return -EINVAL;
	row = &core->rows[index];
	for (a = NXS_CSI2_MIN; a <= NXS_CSI2_STEP; a++) {
		err = nxs_core_read(core, nxs_core_cell(core, NXS_CORE_MIN + a, index), buf,
				    sizeof(buf));
		if (err)
			return err;
		row->range[a] = nxs_le32(buf);
	}

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
	err = nxs_core_read(core, nxs_core_cell(core, NXS_CORE_LIVE, index), buf, sizeof(buf));
	if (err)
		return err;
	core->rows[index].live = nxs_le32(buf);

	return 0;
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_read_live);

/*
 * A range row's range as a control's: signed by the row's TYPE, an INTEGER
 * control's bounds inside s32, the default inside the range. A STEP of 0
 * gives the control a step of 1, and the control reads inactive.
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
 * A row's control is active while the row takes a value now, or, for a
 * read-only row, once the device has a reading: a LIVE of ROW_UNREAD is none.
 */
static bool nxs_core_row_active(const struct nxs_csi2_row *row)
{
	if (nxs_csi2_row_read_only(row))
		return !nxs_csi2_row_unread(row);

	return nxs_csi2_row_takes_value(row);
}

/*
 * Every row's control active or not by its STEP, the adapter's controls
 * on the rows among them, under the handler's lock.
 */
static void nxs_core_activate_rows(struct nxs_csi2_core *core)
{
	unsigned int i;

	for (i = 0; i < core->param_count; i++) {
		const struct nxs_csi2_row *row = &core->rows[i];

		if (row->ctrl)
			v4l2_ctrl_activate(row->ctrl, nxs_core_row_active(row));
		if (row->pair)
			v4l2_ctrl_activate(row->pair, nxs_core_row_active(row));
	}
}

/*
 * The controls follow the rows' ranges, with no transfer: every row's
 * control is active while its row takes a value, and a core control takes
 * its row's range. A value the new range leaves outside is clamped by the
 * framework, without a commit. Called under the handler's lock, inside a
 * set of another cluster too: it writes no value of its own.
 */
void nxs_csi2_core_sync_ranges(struct nxs_csi2_core *core)
{
	unsigned int i;

	nxs_core_activate_rows(core);
	core->syncing = true;
	for (i = 0; i < core->param_count; i++) {
		struct nxs_csi2_row *row = &core->rows[i];
		struct v4l2_ctrl *ctrl = row->ctrl;
		s64 min, max, def;
		u64 step;

		if (!ctrl || !nxs_core_owns(row) || !(row->type & NXS_ROW_TYPE_RANGE))
			continue;
		nxs_csi2_core_ctrl_range(row, ctrl->type != V4L2_CTRL_TYPE_INTEGER64, &min, &max,
					 &step, &def);
		__v4l2_ctrl_modify_range(ctrl, min, max, step, def);
	}
	core->syncing = false;
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_sync_ranges);

/*
 * The controls follow the rows: their ranges and activation, and a core
 * control of a row not held its value in effect. Never called inside a
 * set: a value written into a control there would replace the value the
 * same request brings it.
 */
static void nxs_core_sync_controls(struct nxs_csi2_core *core)
{
	unsigned int i;

	nxs_csi2_core_sync_ranges(core);
	core->syncing = true;
	for (i = 0; i < core->param_count; i++) {
		struct nxs_csi2_row *row = &core->rows[i];
		s64 val;

		if (!row->ctrl || !nxs_core_owns(row) || nxs_csi2_rows_has(&core->held, i) ||
		    !nxs_core_live_to_ctrl(row, row->live, &val))
			continue;
		nxs_core_sync_ctrl(row->ctrl, val);
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

/* Row @index's value for its next commit. */
void nxs_csi2_core_stage(struct nxs_csi2_core *core, unsigned int index, u32 value)
{
	if (index < core->param_count)
		core->rows[index].staged = value;
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_stage);

/*
 * The rows of @rows committed at their staged values, one commit per kind,
 * a row whose STEP reads 0 left out. A door that reads INERT takes no
 * commit: the rows wait for nxs_csi2_core_commit_held() and the call
 * returns NXS_CSI2_INERT.
 */
int nxs_csi2_core_commit(struct nxs_csi2_core *core, const struct nxs_csi2_rows *rows,
			 const char *what)
{
	struct nxs_csi2_rows left = *rows;
	struct nxs_csi2_rows sub;
	unsigned int first;
	unsigned int i;
	int err;

	/* A row past the door's rows names nothing. */
	bitmap_clear(left.bits, core->param_count, NXS_ROW_CAPACITY_MAX - core->param_count);
	if (nxs_csi2_rows_empty(&left))
		return 0;
	if (!core->session) {
		bitmap_or(core->held.bits, core->held.bits, left.bits, NXS_ROW_CAPACITY_MAX);
		return 0;
	}
	while ((first = find_first_bit(left.bits, core->param_count)) < core->param_count) {
		u8 kind = core->rows[first].kind;

		bitmap_zero(sub.bits, NXS_ROW_CAPACITY_MAX);
		for_each_set_bit(i, left.bits, core->param_count)
			if (core->rows[i].kind == kind)
				__set_bit(i, sub.bits);
		err = nxs_core_commit_one(core, &sub, what);
		if (err == NXS_CSI2_INERT) {
			bitmap_or(core->held.bits, core->held.bits, left.bits, NXS_ROW_CAPACITY_MAX);
			return NXS_CSI2_INERT;
		}
		if (err)
			return err;
		bitmap_andnot(left.bits, left.bits, sub.bits, NXS_ROW_CAPACITY_MAX);
	}
	bitmap_andnot(core->held.bits, core->held.bits, rows->bits, NXS_ROW_CAPACITY_MAX);

	return 0;
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_commit);

/*
 * The rows staged while the map was INERT. A trigger conversion commits
 * first, since the device repaints the STEP of the rows the pulse governs
 * under it.
 */
int nxs_csi2_core_commit_held(struct nxs_csi2_core *core)
{
	struct nxs_csi2_rows rows = { };
	int i = nxs_csi2_core_row(core, NXS_PARAM_ID_TRIGGER);
	int err;

	if (!nxs_csi2_core_holds(core))
		return 0;
	if (i >= 0 && nxs_csi2_rows_has(&core->held, i)) {
		nxs_csi2_rows_add(&rows, i);
		err = nxs_csi2_core_commit(core, &rows, "trigger mode");
		if (err)
			return err;
	}
	rows = core->held;

	return nxs_csi2_core_commit(core, &rows, "held controls");
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_commit_held);

static void nxs_core_take_status(struct nxs_csi2_status *status, const u8 *hdr)
{
	status->state = hdr[NXS_MAP_REG_STATE];
	status->stream = 0;
	status->current_mode = NXS_CSI2_NO_MODE;
	status->epoch = hdr[NXS_MAP_REG_DESCRIPTOR_EPOCH];
	status->param_count = nxs_le16(hdr + NXS_MAP_REG_PARAM_COUNT);
}

/*
 * The stream and the mode in effect: the LIVE cells of the STREAM and MODE
 * rows, in one read across both where they lie close. A door that reads
 * INERT runs no mode, and its LIVE cells read 0.
 */
static int nxs_core_read_run(struct nxs_csi2_core *core, struct nxs_csi2_status *status)
{
	int s = nxs_csi2_core_row(core, NXS_PARAM_ID_STREAM);
	int m = nxs_csi2_core_row(core, NXS_PARAM_ID_MODE);
	u8 buf[4 * NXS_CORE_CELLS_MAX];
	int lo;
	int hi;
	int err;

	if (!core->read || (s < 0 && m < 0))
		return 0;
	lo = s < 0 ? m : (m < 0 ? s : min(s, m));
	hi = max(s, m);
	if (hi - lo < NXS_CORE_CELLS_MAX) {
		err = nxs_core_read_span(core, false, nxs_core_cell(core, NXS_CORE_LIVE, lo), buf,
					 4 * (hi - lo + 1));
		if (err)
			return err;
		if (s >= 0)
			core->rows[s].live = nxs_le32(buf + 4 * (s - lo));
		if (m >= 0)
			core->rows[m].live = nxs_le32(buf + 4 * (m - lo));
	} else {
		err = nxs_csi2_core_read_live(core, s);
		if (!err)
			err = nxs_csi2_core_read_live(core, m);
		if (err)
			return err;
	}
	if (s >= 0)
		status->stream = core->rows[s].live != 0;
	if (m >= 0 && status->state != NXS_MAP_STATE_INERT)
		status->current_mode = core->rows[m].live;

	return 0;
}

/*
 * What a session reads first: the header's cells in one latched read, then
 * the stream and the mode in the rows' LIVE cells.
 */
int nxs_csi2_core_read_status(struct nxs_csi2_core *core, struct nxs_csi2_status *status)
{
	u8 hdr[NXS_CORE_STATUS_LEN];
	int err;

	err = nxs_core_read(core, NXS_MAP_REG_MAP_ID, hdr, sizeof(hdr));
	if (err)
		return err;
	nxs_core_take_status(status, hdr);

	return nxs_core_read_run(core, status);
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_read_status);

/*
 * Whether the description differs from the one the core read: another
 * personality uploaded into the pod behind the door moves
 * DESCRIPTOR_EPOCH, and a door that restarted may serve other counts.
 * `status` is the header just read.
 */
int nxs_csi2_core_changed(struct nxs_csi2_core *core, const struct nxs_csi2_status *status,
			  bool *changed)
{
	*changed = status->epoch != core->epoch || status->param_count != core->door_rows;

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
	struct nxs_csi2_rows rows = { };
	int i = nxs_csi2_core_row(core, NXS_PARAM_ID_STREAM);

	if (i < 0)
		return 0;
	core->rows[i].staged = on;
	nxs_csi2_rows_add(&rows, i);

	return nxs_core_commit_one(core, &rows, on ? "stream on" : "stream off");
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
 * A RUN commit of the MODE row at @mode. The device refuses a RUN while the
 * STREAM row reads 1, so a streaming sensor parks first. The run leaves the
 * gate as it finds it: the sensor stays in standby until a GATE 1. The map
 * repaints every range and LIVE: the caller refreshes. An INERT map takes
 * no run and the call returns NXS_CSI2_INERT.
 */
int nxs_csi2_core_run_mode(struct nxs_csi2_core *core, const struct nxs_csi2_mode *mode)
{
	struct nxs_csi2_rows rows = { };
	int i = nxs_csi2_core_row(core, NXS_PARAM_ID_MODE);
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
	core->rows[i].staged = mode->value;
	nxs_csi2_rows_add(&rows, i);

	return nxs_core_commit_one(core, &rows, "mode");
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

	return val >= 0 && val < row->count ? nxs_csi2_row_value(row, val) : 0;
}

static int nxs_core_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct nxs_csi2_row *row = ctrl->priv;
	struct nxs_csi2_core *core = row->core;
	struct nxs_csi2_rows rows = { };
	int err;

	/* The handler's lock is the adapter's, held by the framework. A control whose row left the door sets nothing. */
	if (!core || core->syncing)
		return 0;
	nxs_csi2_core_stage(core, row->index, nxs_core_row_value(row));
	nxs_csi2_rows_add(&rows, row->index);
	err = nxs_csi2_core_commit(core, &rows, ctrl->name);

	/* The map takes a commit once its sensor runs: the value waits. */
	return err == NXS_CSI2_INERT ? 0 : err;
}

/*
 * A read-only row, and the trigger the host tool may convert, read LIVE. A
 * read-only row that serves ROW_UNREAD has no reading yet: its control
 * keeps its value and reads inactive until the first reading.
 */
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
	/* The handler's lock is held, as the framework calls this. */
	v4l2_ctrl_activate(ctrl, nxs_core_row_active(row));
	if (nxs_csi2_row_unread(row) || !nxs_core_live_to_ctrl(row, row->live, &val))
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

/*
 * The test pattern menu: the row's codes in the row's order, code 0 named
 * Disabled. The menu's maximum index is a u8, so it offers the first 256
 * codes of a longer list.
 */
static int nxs_core_new_test_pattern(struct nxs_csi2_core *core, struct v4l2_ctrl_handler *hdl,
				     struct nxs_csi2_row *row)
{
	struct device *dev = &core->client->dev;
	unsigned int n = min_t(unsigned int, row->count, NXS_CORE_MENU_MAX);
	int def = nxs_core_row_find(row, row->def);
	const char **menu;
	unsigned int i;

	if (row->count > n)
		dev_warn(dev, "test pattern: %u codes, the menu offers the first %u\n", row->count, n);
	menu = devm_kcalloc(dev, n, sizeof(*menu), GFP_KERNEL);
	if (!menu)
		return -ENOMEM;
	for (i = 0; i < n; i++) {
		u32 code = nxs_csi2_row_value(row, i);

		menu[i] = code ? devm_kasprintf(dev, GFP_KERNEL, "Pattern %u", code) :
			  "Disabled";
		if (!menu[i])
			return -ENOMEM;
	}
	row->ctrl = v4l2_ctrl_new_std_menu_items(hdl, &nxs_core_ctrl_ops, V4L2_CID_TEST_PATTERN,
						 n - 1, 0, def > 0 && def < (int)n ? def : 0, menu);
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
		u32 code = nxs_csi2_row_value(row, i);

		if (code < NXS_CORE_TRIGGER_MENU_MAX)
			codes |= BIT_ULL(code);
		else
			dev_warn(dev, "trigger code %u: past the menu, not offered\n", code);
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
 * integer. A door's read-only row is a volatile control that reads LIVE. A
 * read-only row with no core, a passive node's line row, holds the value in
 * effect, so a range change that moves it raises its event. The black level
 * and the temperature carry their own ids and names.
 */
/*
 * Whether @row's control is a 64-bit integer: an unsigned range whose MAX
 * passes S32_MAX, in the description or in the cells.
 */
static bool nxs_core_row_wide(const struct nxs_csi2_row *row)
{
	return (row->type & NXS_ROW_TYPE_RANGE) && !(row->type & NXS_ROW_TYPE_SIGNED) &&
	       max(row->range[NXS_CSI2_MAX], row->values[NXS_CSI2_MAX]) > S32_MAX;
}

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
		cfg.flags = V4L2_CTRL_FLAG_READ_ONLY | (row->core ? V4L2_CTRL_FLAG_VOLATILE : 0);
	if (row->type & NXS_ROW_TYPE_RANGE) {
		if (nxs_core_row_wide(row))
			cfg.type = V4L2_CTRL_TYPE_INTEGER64;
		nxs_csi2_core_ctrl_range(row, cfg.type == V4L2_CTRL_TYPE_INTEGER, &cfg.min,
					 &cfg.max, &cfg.step, &cfg.def);
	} else {
		menu = devm_kcalloc(dev, row->count, sizeof(*menu), GFP_KERNEL);
		if (!menu)
			return -ENOMEM;
		for (i = 0; i < row->count; i++)
			menu[i] = nxs_csi2_row_s64(row, nxs_csi2_row_value(row, i));
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
	nxs_core_activate_rows(core);

	return hdl->error;
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_init_controls);

/* ---- the door at init ------------------------------------------------ */

/*
 * A control active or not, under its handler's lock. @locked is a caller
 * that holds that lock; otherwise the call takes it.
 */
static void nxs_core_activate(struct v4l2_ctrl *ctrl, bool active, bool locked)
{
	if (locked) {
		lockdep_assert_held(ctrl->handler->lock);
		v4l2_ctrl_activate(ctrl, active);
		return;
	}
	v4l2_ctrl_lock(ctrl);
	v4l2_ctrl_activate(ctrl, active);
	v4l2_ctrl_unlock(ctrl);
}

/* An enum row's values, folded into one word: FNV-1a over the values. */
static u32 nxs_core_digest(const struct nxs_csi2_row *row)
{
	u32 h = 2166136261u;
	unsigned int i, b;

	for (i = 0; i < row->count; i++) {
		u32 v = nxs_csi2_row_value(row, i);

		for (b = 0; b < 4; b++)
			h = (h ^ ((v >> (8 * b)) & 0xff)) * 16777619u;
	}

	return h;
}

/* @ctrl leaves @row: it waits, inactive and on no row, in the next detached record. */
static void nxs_core_detach_ctrl(struct nxs_csi2_core *core, const struct nxs_csi2_row *row,
				 struct v4l2_ctrl *ctrl, bool pair, bool keep, bool locked)
{
	struct nxs_csi2_detached *d;

	ctrl->priv = ctrl->ops == core->adapter_ops ? NULL : &core->orphan;
	nxs_core_activate(ctrl, false, locked);
	if (!keep)
		return;
	d = &core->detached[core->detached_count++];
	d->ctrl = ctrl;
	strscpy(d->name, row->name, sizeof(d->name));
	d->id = row->id;
	d->std = row->std;
	d->type = row->type;
	d->read_only = nxs_csi2_row_read_only(row);
	d->count = row->type & NXS_ROW_TYPE_RANGE ? 0 : row->count;
	d->digest = row->type & NXS_ROW_TYPE_RANGE ? 0 : nxs_core_digest(row);
	d->wide = ctrl->ops == &nxs_core_ctrl_ops && ctrl->type == V4L2_CTRL_TYPE_INTEGER64;
	d->pair = pair;
}

/*
 * The rows' controls taken off their rows before the door is read again:
 * each waits for a row of its id and shape, inactive and on no row, an
 * adapter's own on NULL and a core's on the orphan. Every control leaves
 * its row, a row's second control too, since the read may free the rows.
 * Without the memory to keep the records, the controls stay inactive until
 * the next bind. @locked is a caller that holds the handler's lock.
 */
static void nxs_core_detach(struct nxs_csi2_core *core, bool locked)
{
	struct nxs_csi2_detached *detached;
	unsigned int i;

	if (!core->param_count)
		return;
	detached = kvcalloc(core->detached_count + 2 * core->param_count, sizeof(*detached),
			    GFP_KERNEL);
	if (detached) {
		if (core->detached_count)
			memcpy(detached, core->detached, core->detached_count * sizeof(*detached));
		kvfree(core->detached);
		core->detached = detached;
	} else {
		dev_warn(&core->client->dev, "no memory to keep the controls' rows: they stay inactive until the next bind\n");
	}
	for (i = 0; i < core->param_count; i++) {
		struct nxs_csi2_row *row = &core->rows[i];

		if (row->ctrl)
			nxs_core_detach_ctrl(core, row, row->ctrl, false, detached != NULL, locked);
		if (row->pair)
			nxs_core_detach_ctrl(core, row, row->pair, true, detached != NULL, locked);
		row->ctrl = NULL;
		row->pair = NULL;
	}
}

/*
 * Whether a detached control fits @row of the door just read: a row the
 * drivers offer with a free slot, of the control's id and shape, and of
 * its name where the row is custom. A core's range control fits a row of
 * its own width alone, 32 or 64 bits, as a fresh bind would make it. A
 * row of another shape takes a fresh control at the next bind.
 */
static bool nxs_core_fits(const struct nxs_csi2_detached *d, const struct nxs_csi2_row *row)
{
	const u8 shape = NXS_ROW_TYPE_RANGE | NXS_ROW_TYPE_SIGNED;
	bool range = row->type & NXS_ROW_TYPE_RANGE;

	if (d->pair ? !row->ctrl || row->pair : row->ctrl != NULL)
		return false;
	if (!nxs_core_row_offered(row) || row->id != d->id || row->std != d->std ||
	    (row->type & shape) != (d->type & shape) || nxs_csi2_row_read_only(row) != d->read_only)
		return false;
	if (!range && (row->count != d->count || nxs_core_digest(row) != d->digest))
		return false;
	if (range && d->ctrl->ops == &nxs_core_ctrl_ops && nxs_core_row_wide(row) != d->wide)
		return false;

	return row->std || !strcmp(row->name, d->name);
}

/*
 * The detached controls back on the rows of their ids and shapes in the
 * door just read. One whose row is gone or changed its shape stays
 * detached, and a row without a control waits for the next bind. @locked
 * is a caller that holds the handler's lock.
 */
static void nxs_core_repoint(struct nxs_csi2_core *core, bool locked)
{
	struct device *dev = &core->client->dev;
	unsigned int bare = 0;
	unsigned int left = 0;
	unsigned int i, j;

	if (!core->detached_count)
		return;
	for (j = 0; j < core->detached_count; j++) {
		struct nxs_csi2_detached *d = &core->detached[j];
		struct nxs_csi2_row *row = NULL;

		for (i = 0; i < core->param_count; i++) {
			if (!nxs_core_fits(d, &core->rows[i]))
				continue;
			row = &core->rows[i];
			break;
		}
		if (!row) {
			dev_info(dev, "control %s: no row of its id and shape in the door\n",
				 d->ctrl->name);
			core->detached[left++] = *d;
			continue;
		}
		if (d->pair)
			row->pair = d->ctrl;
		else
			row->ctrl = d->ctrl;
		d->ctrl->priv = row;
		nxs_core_activate(d->ctrl, nxs_core_row_active(row), locked);
	}
	core->detached_count = left;
	for (i = 0; i < core->param_count; i++) {
		const struct nxs_csi2_row *row = &core->rows[i];

		bare += !row->ctrl && nxs_core_row_offered(row) && nxs_core_owns(row);
	}
	if (bare)
		dev_info(dev, "%u rows of the new description get their controls at the next bind\n", bare);
}

/* The core's allocations, released when the device unbinds. */
static void nxs_core_release(void *data)
{
	struct nxs_csi2_core *core = data;

	kvfree(core->desc);
	kvfree(core->detached);
	kvfree(core->rows);
	core->desc = NULL;
	core->detached = NULL;
	core->rows = NULL;
	core->detached_count = 0;
	core->rows_cap = 0;
}

/*
 * The header's identity, release and door, its counts and its region
 * table, into the core. -ENODEV for a door this core does not read: no
 * MAP_ID of the interface, a release of another MAJOR or a later MINOR, a
 * device's main door, or counts and regions past the layout.
 */
static int nxs_core_take_header(struct nxs_csi2_core *core, const u8 *hdr)
{
	struct device *dev = &core->client->dev;
	u8 addr = core->client->addr;
	bool cells = false;
	bool window = false;
	unsigned int count;
	unsigned int i;

	if (hdr[NXS_MAP_REG_MAP_ID] != NXS_MAP_ID_VALUE) {
		dev_err(dev, "no NXS Device Interface at 0x%02x: MAP_ID 0x%02x\n", addr,
			hdr[NXS_MAP_REG_MAP_ID]);
		return -ENODEV;
	}
	for (i = 0; i < ARRAY_SIZE(core->release); i++)
		core->release[i] = hdr[NXS_MAP_REG_RELEASE + i];
	if (core->release[0] != NXS_CORE_LAYOUT_MAJOR || core->release[1] > NXS_CORE_LAYOUT_MINOR) {
		dev_err(dev, "camera door at 0x%02x: release %u.%u.%u, this driver reads layout %u.%u and the minors below it\n",
			addr, core->release[0], core->release[1], core->release[2],
			NXS_CORE_LAYOUT_MAJOR, NXS_CORE_LAYOUT_MINOR);
		return -ENODEV;
	}
	if (hdr[NXS_MAP_REG_DOOR] != NXS_DOOR_DRIVER) {
		dev_err(dev, "camera door at 0x%02x: DOOR %u, not a driver door a host kernel driver owns\n",
			addr, hdr[NXS_MAP_REG_DOOR]);
		return -ENODEV;
	}
	core->module_kind = hdr[NXS_MAP_REG_MODULE_KIND];
	core->fw_major = hdr[NXS_MAP_REG_FW_VERSION_MAJOR];
	core->fw_minor = hdr[NXS_MAP_REG_FW_VERSION_MINOR];
	core->epoch = hdr[NXS_MAP_REG_DESCRIPTOR_EPOCH];
	core->door_rows = nxs_le16(hdr + NXS_MAP_REG_PARAM_COUNT);
	core->capacity = nxs_le16(hdr + NXS_MAP_REG_ROW_CAPACITY);
	core->mode_count = 0;
	core->desc_size = nxs_le32(hdr + NXS_MAP_REG_DESC_SIZE);
	count = hdr[NXS_MAP_REG_REGION_COUNT];
	if (core->capacity > NXS_ROW_CAPACITY_MAX || core->door_rows > core->capacity ||
	    count > NXS_REGION_TABLE_MAX || core->desc_size > NXS_CORE_DESC_MAX) {
		dev_err(dev, "camera door at 0x%02x: %u rows of %u, %u regions, a description of %u bytes\n",
			addr, core->door_rows, core->capacity, count, core->desc_size);
		return -ENODEV;
	}
	for (i = 0; i < count; i++) {
		const u8 *entry = hdr + NXS_MAP_REG_REGION_TABLE + i * NXS_REGION_ENTRY_SIZE;
		u32 base = nxs_le16(entry + NXS_REGION_FIELD_BASE);
		u32 size = nxs_le16(entry + NXS_REGION_FIELD_SIZE);

		switch (entry[NXS_REGION_FIELD_TYPE]) {
		case NXS_REGION_TYPE_CELLS:
			if (base < NXS_MAP_LATCH_SIZE || size < 4 * NXS_CELLS_ARRAYS * core->capacity ||
			    base + 4 * NXS_CELLS_ARRAYS * core->capacity > 0x10000)
				break;
			core->cells = base;
			cells = true;
			break;
		case NXS_REGION_TYPE_DESCRIPTION:
			if (base < NXS_MAP_LATCH_SIZE || base + size > 0x10000 ||
			    size < min_t(u32, core->desc_size, NXS_DESC_PAGE_SIZE) ||
			    (size < NXS_DESC_PAGE_SIZE && core->desc_size > size))
				break;
			core->window = base;
			window = true;
			break;
		default:
			break;
		}
	}
	if (!cells || !window) {
		dev_err(dev, "camera door at 0x%02x: its region table names no %s it can serve\n", addr,
			cells ? "DESCRIPTION window" : "CELLS region");
		return -ENODEV;
	}

	return 0;
}

/* DESC_PAGE written: the page the description window serves. */
static int nxs_core_write_page(struct nxs_csi2_core *core, bool locked, unsigned int p)
{
	const u8 page[NXS_MAP_REG_DESC_PAGE_LEN] = { p & 0xff, (p >> 8) & 0xff };

	return nxs_core_xfer(core->client, locked, NXS_MAP_REG_DESC_PAGE, page, sizeof(page),
			     NULL, 0);
}

/*
 * The description, DESC_SIZE bytes through the window, page by page. The
 * door keeps DESC_PAGE, and another host may have left it at any page: a
 * page other than the one the door serves, @page as the header read it,
 * is written to DESC_PAGE and read under one bus lock. DESC_PAGE ends at
 * 0, after an error as well.
 */
static int nxs_core_read_desc(struct nxs_csi2_core *core, unsigned int page)
{
	struct i2c_adapter *adap = core->client->adapter;
	unsigned int pages = DIV_ROUND_UP(core->desc_size, NXS_DESC_PAGE_SIZE);
	unsigned int p;
	int err = 0;
	int ret;

	for (p = 0; p < pages && !err; p++) {
		size_t len = min_t(size_t, core->desc_size - p * NXS_DESC_PAGE_SIZE, NXS_DESC_PAGE_SIZE);
		u8 *buf = core->desc + p * NXS_DESC_PAGE_SIZE;

		if (p == page) {
			err = nxs_core_read_span(core, false, core->window, buf, len);
			continue;
		}
		page = p;
		i2c_lock_bus(adap, I2C_LOCK_SEGMENT);
		err = nxs_core_write_page(core, true, p);
		if (!err)
			err = nxs_core_read_span(core, true, core->window, buf, len);
		i2c_unlock_bus(adap, I2C_LOCK_SEGMENT);
	}
	if (page || err) {
		ret = nxs_core_write_page(core, false, 0);
		if (!err)
			err = ret;
	}

	return err;
}

/* The camera personality's rows, from its PARAMS record. */
static int nxs_core_take_rows(struct nxs_csi2_core *core, const u8 *rec, size_t len)
{
	struct nxs_csi2_row *rows;
	unsigned int n = len / NXS_ROW_SIZE;
	unsigned int i;

	if (len % NXS_ROW_SIZE || n > core->door_rows)
		return -EBADMSG;
	if (n > core->rows_cap) {
		rows = kvcalloc(n, sizeof(*rows), GFP_KERNEL);
		if (!rows)
			return -ENOMEM;
		kvfree(core->rows);
		core->rows = rows;
		core->rows_cap = n;
	}
	core->param_count = n;
	for (i = 0; i < n; i++) {
		struct nxs_csi2_row *row = &core->rows[i];

		nxs_core_parse_row(row, i, rec + i * NXS_ROW_SIZE);
		row->core = core;
	}

	return 0;
}

/* A ROW_VALUES record: the values of one LIST row of the camera personality. */
static int nxs_core_take_list(struct nxs_csi2_core *core, const u8 *rec, size_t len)
{
	struct nxs_csi2_row *row;
	unsigned int index;
	unsigned int count;

	if (len < 4)
		return -EBADMSG;
	index = nxs_le16(rec);
	count = nxs_le16(rec + 2);
	if (len != 4 + 4 * (size_t)count || index >= core->param_count)
		return -EBADMSG;
	row = &core->rows[index];
	if ((row->type & NXS_ROW_TYPE_RANGE) || !(row->type & NXS_ROW_TYPE_LIST))
		return -EBADMSG;
	row->list = rec + 4;
	row->count = count;

	return 0;
}

/*
 * One trailer from *@pos: a count byte, then records of a type byte, a u16
 * LE length and the bytes. Its PARAMS rows count into @rows. The camera
 * trailer, @camera not NULL, gives the core its PARAMS rows and the mode
 * table of its CAMERA_MAP record. With @lists the pass takes the
 * ROW_VALUES records alone, since a list may come before the PARAMS
 * record of its row.
 */
static int nxs_core_parse_trailer(struct nxs_csi2_core *core, const u8 **pos, const u8 *end,
				  bool lists, u32 *rows, bool *camera)
{
	const u8 *p = *pos;
	unsigned int records;
	unsigned int r;
	int err = 0;

	if (p >= end)
		return -EBADMSG;
	records = *p++;
	for (r = 0; r < records && !err; r++) {
		unsigned int type;
		size_t len;

		if (end - p < 3)
			return -EBADMSG;
		type = p[0];
		len = nxs_le16(p + 1);
		p += 3;
		if ((size_t)(end - p) < len)
			return -EBADMSG;
		if (lists) {
			if (type == NXS_TRAILER_RECORD_ROW_VALUES)
				err = nxs_core_take_list(core, p, len);
		} else if (type == NXS_TRAILER_RECORD_PARAMS) {
			*rows += len / NXS_ROW_SIZE;
			if (camera)
				err = nxs_core_take_rows(core, p, len);
		} else if (camera && type == NXS_TRAILER_RECORD_CAMERA_MAP) {
			err = nxs_core_parse_modes(core, p, len);
			*camera = true;
		}
		p += len;
	}
	*pos = p;

	return err;
}

/*
 * The description's trailers, one per personality. The first trailer is
 * the camera personality's: its PARAMS rows and the mode table of its
 * CAMERA_MAP record, then in a second pass its ROW_VALUES lists, whatever
 * the records' order. The rows of every trailer count against
 * PARAM_COUNT. -EBADMSG for a description that does not decode.
 */
static int nxs_core_parse_desc(struct nxs_csi2_core *core, unsigned int personalities)
{
	const u8 *end = core->desc + core->desc_size;
	const u8 *first = core->desc;
	const u8 *p = core->desc;
	bool camera = false;
	u32 rows = 0;
	unsigned int t;
	int err = 0;

	core->param_count = 0;
	for (t = 0; t < personalities && !err; t++)
		err = nxs_core_parse_trailer(core, &p, end, false, &rows, t ? NULL : &camera);
	if (!err && personalities)
		err = nxs_core_parse_trailer(core, &first, end, true, &rows, NULL);
	if (!err && (!camera || rows != core->door_rows))
		err = -EBADMSG;

	return err;
}

/*
 * The header, the description and the cells. The header is one latched
 * read, the region table names the CELLS and DESCRIPTION regions, and
 * DESCRIPTOR_EPOCH brackets the description: a description that moved
 * while it was read is read again. A door that does not answer returns
 * the bus error, which nxs_csi2_core_unanswered() tells; a door this core
 * does not read returns -ENODEV; a door that is not ready, with no
 * description or no personality yet, a pod whose camera personality has not
 * loaded, returns -ENOENT; a description that does not decode returns
 * -EBADMSG. The modes stand as the door serves them, less a mode of a data
 * type with no media bus code: the adapter offers the ones its receiver
 * carries. @locked is a caller that holds the lock of the controls' handler.
 */
static int nxs_core_init(struct nxs_csi2_core *core, struct i2c_client *client, bool locked)
{
	struct device *dev = &client->dev;
	u8 hdr[NXS_MAP_LATCH_SIZE];
	unsigned int tries;
	unsigned int i;
	u8 epoch;
	int err;

	core->client = client;
	core->read = false;
	core->param_count = 0;
	if (!core->managed) {
		err = devm_add_action_or_reset(dev, nxs_core_release, core);
		if (err)
			return err;
		core->managed = true;
	}
	/* The host reads the header in one latched transaction (§3). */
	if (nxs_core_read_max(client->adapter) < NXS_MAP_LATCH_SIZE) {
		dev_err(dev, "camera door at 0x%02x: the adapter reads at most %zu bytes at once, the header takes %u in one read\n",
			client->addr, nxs_core_read_max(client->adapter), NXS_MAP_LATCH_SIZE);
		return -EOPNOTSUPP;
	}
	for (tries = 0; ; tries++) {
		err = nxs_core_read(core, NXS_MAP_REG_MAP_ID, hdr, sizeof(hdr));
		if (err)
			return err;
		err = nxs_core_take_header(core, hdr);
		if (err)
			return err;
		if (!core->desc_size || !hdr[NXS_MAP_REG_PERSONALITY_COUNT]) {
			dev_dbg(dev, "camera door at 0x%02x serves no personality yet\n", client->addr);
			return -ENOENT;
		}
		kvfree(core->desc);
		core->desc = kvmalloc(core->desc_size, GFP_KERNEL);
		if (!core->desc)
			return -ENOMEM;
		err = nxs_core_read_desc(core, nxs_le16(hdr + NXS_MAP_REG_DESC_PAGE));
		if (!err)
			err = nxs_core_read(core, NXS_MAP_REG_DESCRIPTOR_EPOCH, &epoch, 1);
		if (err)
			return err;
		if (epoch == core->epoch)
			break;
		if (tries + 1 >= NXS_CORE_DESC_TRIES) {
			dev_err(dev, "camera door at 0x%02x: its description moved under %u reads\n",
				client->addr, NXS_CORE_DESC_TRIES);
			return -ESTALE;
		}
	}
	err = nxs_core_parse_desc(core, hdr[NXS_MAP_REG_PERSONALITY_COUNT]);
	if (err) {
		dev_err(dev, "camera door at 0x%02x: its description of %u bytes does not decode: %d\n",
			client->addr, core->desc_size, err);
		core->param_count = 0;
		return err;
	}
	dev_info(dev, "camera door of release %u.%u.%u at 0x%02x: %s, firmware %u.%u, %u rows, %u mode%s, %s\n",
		 core->release[0], core->release[1], core->release[2], client->addr,
		 nxs_core_module_kind_name(core->module_kind), core->fw_major, core->fw_minor,
		 core->param_count, core->mode_count, core->mode_count == 1 ? "" : "s",
		 nxs_csi2_core_state_name(hdr[NXS_MAP_REG_STATE]));

	memset(core->row_of, -1, sizeof(core->row_of));
	bitmap_zero(core->held.bits, NXS_ROW_CAPACITY_MAX);
	for (i = 0; i < core->param_count; i++) {
		struct nxs_csi2_row *row = &core->rows[i];

		if (!nxs_core_row_offered(row))
			continue;
		row->std = nxs_core_std(core, row);
		if (row->std)
			core->row_of[nxs_csi2_id_slot(row->std)] = i;
	}
	err = nxs_core_read_ranges(core);
	if (err)
		return err;
	err = nxs_core_read_lives(core);
	if (err)
		return err;
	nxs_core_repoint(core, locked);
	core->read = true;

	return 0;
}

/* nxs_core_init() for a caller that holds no lock of the controls' handler: the probe. */
int nxs_csi2_core_init(struct nxs_csi2_core *core, struct i2c_client *client)
{
	return nxs_core_init(core, client, false);
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_init);

int nxs_csi2_core_init_locked(struct nxs_csi2_core *core, struct i2c_client *client)
{
	return nxs_core_init(core, client, true);
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_init_locked);

/* The re-read both entries share. @locked is a caller that holds the handler's lock. */
static int nxs_core_reread(struct nxs_csi2_core *core, bool locked)
{
	struct device *dev = &core->client->dev;

	nxs_core_detach(core, locked);
	dev_info(dev, "the description of the camera door at 0x%02x changed: reading it again\n",
		 core->client->addr);
	core->read = false;

	return nxs_core_init(core, core->client, locked);
}

/*
 * The door read again after its description changed behind it
 * (nxs_csi2_core_changed): the rows, the modes and the values in effect as
 * the door serves them now, with the row controls standing, each on a row
 * of its id and shape. Returns nxs_csi2_core_init()'s error, after which
 * the controls wait, inactive, for the read that succeeds. The caller holds
 * no lock of the controls' handler, and the core takes it for each control.
 */
int nxs_csi2_core_reread(struct nxs_csi2_core *core)
{
	return nxs_core_reread(core, false);
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_reread);

int nxs_csi2_core_reread_locked(struct nxs_csi2_core *core)
{
	return nxs_core_reread(core, true);
}
EXPORT_SYMBOL_GPL(nxs_csi2_core_reread_locked);

MODULE_DESCRIPTION("A camera door of the NXS Device Interface, for the drivers that serve it");
MODULE_AUTHOR("Aliensense");
MODULE_LICENSE("GPL");
MODULE_VERSION("layout " __stringify(NXS_CORE_LAYOUT_MAJOR) "." __stringify(NXS_CORE_LAYOUT_MINOR)
	       ", release " NXS_VERSION);

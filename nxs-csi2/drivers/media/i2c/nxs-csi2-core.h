/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * nxs-csi2-core.h - a camera door of the NXS Device Interface, for the drivers that serve it
 *
 * Copyright (C) 2026 Aliensense
 *
 * The core reads a camera door of the NXS Device Interface: the header, the
 * region table and the description, whose camera personality gives the rows
 * and the mode table. It commits the rows with the interface's windows and
 * offers the rows no adapter maps to a standard control as custom controls.
 * An adapter, the V4L2 sub-device driver or the tegracam driver, owns the
 * framework's registration, its standard controls, the format and the
 * stream, and calls the core under one lock of its own: the core takes none.
 * The constants are nxs-csi2-map.h, generated from the interface's one
 * source. The contract is the NXS Device Interface manual.
 */

#ifndef NXS_CSI2_CORE_H
#define NXS_CSI2_CORE_H

#include <linux/bitmap.h>
#include <linux/i2c.h>
#include <linux/types.h>

#include <media/v4l2-ctrls.h>

#include "nxs-csi2-map.h"

/* A commit that met the door INERT: nothing was written and the rows wait. */
#define NXS_CSI2_INERT			1

/*
 * The standard ids the core knows, each at a slot of row_of: the generic
 * family's ids from 0, then the camera family's by their low byte. A row of
 * any other id, another family's among them, is custom.
 */
#define NXS_CSI2_ID_LOW(id)		((id) & ((1u << NXS_PARAM_FAMILY_SHIFT) - 1))
#define NXS_CSI2_GENERIC_IDS		(NXS_PARAM_ID_FAULT + 1)
#define NXS_CSI2_CAMERA_IDS		(NXS_CSI2_ID_LOW(NXS_PARAM_ID_PART) + 1)
#define NXS_CSI2_ID_COUNT		(NXS_CSI2_GENERIC_IDS + NXS_CSI2_CAMERA_IDS)

/* The mode a status names while the door runs none: INERT, or no MODE row. */
#define NXS_CSI2_NO_MODE		0xff

/*
 * A device-tree rate within this many milli-frames per second of the camera
 * map's top is the same rate rounded: it is logged, never warned.
 */
#define NXS_CSI2_FPS_TOLERANCE_MILLI	100

/*
 * The core's controls, at the base the adapter passes: the trigger menu,
 * the black level, the temperature, then a custom row's at the base plus
 * 0x10 plus its row index. Both adapters pass NXS_CSI2_CID_BASE, so a
 * control carries one id on every host; it sits past every driver base
 * v4l2-controls.h allocates. The base plus 3 to 5 are a passive node's
 * owner controls, nxs-csi2's: the line in effect in ns and the exposure's
 * minimum and maximum in us, which the node's owner paints.
 */
#define NXS_CSI2_CID_BASE		(V4L2_CID_USER_BASE + 0x2000)
#define NXS_CSI2_CID_TRIGGER_MODE(base)	((base) + 0)
#define NXS_CSI2_CID_BLACK_LEVEL(base)	((base) + 1)
#define NXS_CSI2_CID_TEMPERATURE(base)	((base) + 2)
#define NXS_CSI2_CID_OWNER_LINE_NS(base)		((base) + 3)
#define NXS_CSI2_CID_OWNER_EXPOSURE_MIN_US(base)	((base) + 4)
#define NXS_CSI2_CID_OWNER_EXPOSURE_MAX_US(base)	((base) + 5)
#define NXS_CSI2_CID_ROW(base, index)	((base) + 0x10 + (index))

/* Indices into a row's range in effect. */
enum nxs_csi2_bound {
	NXS_CSI2_MIN,
	NXS_CSI2_MAX,
	NXS_CSI2_STEP,
};

/* One row of the mode table. */
struct nxs_csi2_mode {
	u16 width;
	u16 height;
	/* The bits of one sample, the data type's. */
	u8 bit_depth;
	u8 lanes;
	u16 rate_mbps;
	/* The CSI-2 data type code of the stream (Csi2DataType). */
	u8 data_type;
	u8 flags;
	u8 value;
	u8 phase;
	u32 fps_max_milli;
};

/* A set of a door's rows by index, one bit per row the layout allows. */
struct nxs_csi2_rows {
	DECLARE_BITMAP(bits, NXS_ROW_CAPACITY_MAX);
};

struct nxs_csi2_core;

/* One parameter row as the description carries it, and the control it became. */
struct nxs_csi2_row {
	struct nxs_csi2_core *core;
	char name[NXS_ROW_NAME_LEN + 1];
	/*
	 * An enum row's inline values, u16 zero-extended or u32 under
	 * ROW_TYPE_WIDE, or a range row's declared min, max and step.
	 */
	u32 values[NXS_ROW_VALUES_MAX];
	/* An enum row's values in its ROW_VALUES record, u32 LE in the core's description; NULL when inline. */
	const u8 *list;
	/* The range in effect: MIN, MAX and STEP, a STEP of 0 while the row takes no value. */
	u32 range[3];
	/* The value in effect at the last refresh. */
	u32 live;
	u32 def;
	/* The value for the row's next commit. */
	u32 staged;
	u16 id;
	u16 index;
	/* An enum row's value count, 3 for a range row. */
	u16 count;
	/*
	 * The standard id the row is served as, 0 for a custom row. The core
	 * creates no control for EXPOSURE_US, FRAME_RATE_MFPS, GAIN_MDB and
	 * FLIPS, which the adapter maps to its framework's controls, nor for
	 * STREAM and MODE, which the stream gate and the format commit.
	 */
	u16 std;
	u8 type;
	u8 kind;
	u8 flags;
	struct v4l2_ctrl *ctrl;
	/* A second control on the row: VFLIP beside HFLIP on the FLIPS row. */
	struct v4l2_ctrl *pair;
};

/*
 * What a session reads: the header's state, epoch and row count, and the
 * stream and the mode in effect, the LIVE cells of the STREAM and MODE rows.
 */
struct nxs_csi2_status {
	u8 state;
	/* 0 standby, 1 streaming; 0 for a door with no STREAM row. */
	u8 stream;
	/* The MODE row's LIVE, NXS_CSI2_NO_MODE while the door is INERT or has no MODE row. */
	u8 current_mode;
	u8 epoch;
	u16 param_count;
};

/*
 * A control a re-read took off its row. It waits for a row of the same id
 * and shape: the same standard id, a range or an enum alike, the same
 * signedness and access, and for an enum the same values. A custom row,
 * id 0, also keeps its name.
 */
struct nxs_csi2_detached {
	struct v4l2_ctrl *ctrl;
	char name[NXS_ROW_NAME_LEN + 1];
	u16 id;
	u16 std;
	u8 type;
	bool read_only;
	/* An enum's value count and a digest of its values, 0 for a range. */
	u16 count;
	u32 digest;
	/* A core's 64-bit range control, which fits a 64-bit row alone. */
	bool wide;
	/* The row's second control. */
	bool pair;
};

/* The door as the core read it, and the commit state. */
struct nxs_csi2_core {
	struct i2c_client *client;
	/* The door was read: the fields below stand. */
	bool read;
	u8 module_kind;
	u8 fw_major;
	u8 fw_minor;
	/* The release the door serves: MAJOR, MINOR and PATCH. */
	u8 release[3];
	/* DESCRIPTOR_EPOCH the description was read at; a commit runs under it. */
	u8 epoch;
	/* PARAM_COUNT: the rows the door serves, every personality's. */
	u16 door_rows;
	/* The camera personality's rows, the description's first: rows 0 to param_count - 1. */
	u16 param_count;
	/* The modes of the camera personality's CAMERA_MAP record. */
	u8 mode_count;
	/* The CELLS region's base and the door's ROW_CAPACITY. */
	u16 cells;
	u16 capacity;
	/* The DESCRIPTION region's base. */
	u16 window;
	struct nxs_csi2_mode modes[NXS_MODE_TABLE_MAX];
	/* The description as read: the ROW_VALUES lists of the rows point into it. */
	u8 *desc;
	u32 desc_size;
	struct nxs_csi2_row *rows;
	/* The rows the rows array holds. */
	u16 rows_cap;
	/* The row of a control whose row a re-read door no longer carries: its core is NULL. */
	struct nxs_csi2_row orphan;
	/* The adapter's own control ops: such a control takes no row as NULL, the core's take the orphan. */
	const struct v4l2_ctrl_ops *adapter_ops;
	/* The controls a re-read took off their rows, each back on a row of its id and shape once a read succeeds. */
	struct nxs_csi2_detached *detached;
	unsigned int detached_count;
	/* The allocations above are released when the device unbinds. */
	bool managed;
	/* A standard id's row index, -1 for none. */
	s16 row_of[NXS_CSI2_ID_COUNT];
	/* The rows staged while the door was INERT or no session was open, committed by nxs_csi2_core_commit_held(). */
	struct nxs_csi2_rows held;
	/*
	 * A capture session is open on the link: the core commits. Otherwise a
	 * control set waits in the held rows until the next session's start, so
	 * the tool's bring-up of the port meets no commit of the driver's.
	 */
	bool session;
	/* LIVE or a range written into the core's controls: no commit. */
	bool syncing;
	u32 cid_base;
};

/* Row @index joins @rows. */
static inline void nxs_csi2_rows_add(struct nxs_csi2_rows *rows, unsigned int index)
{
	if (index < NXS_ROW_CAPACITY_MAX)
		__set_bit(index, rows->bits);
}

/* Whether row @index is in @rows. */
static inline bool nxs_csi2_rows_has(const struct nxs_csi2_rows *rows, unsigned int index)
{
	return index < NXS_ROW_CAPACITY_MAX && test_bit(index, rows->bits);
}

static inline bool nxs_csi2_rows_empty(const struct nxs_csi2_rows *rows)
{
	return bitmap_empty(rows->bits, NXS_ROW_CAPACITY_MAX);
}

/* The adapter's session on the link: open at its stream start, closed at its stream stop. */
static inline void nxs_csi2_core_session(struct nxs_csi2_core *core, bool open)
{
	core->session = open;
}

/* A row's value as a number, signed by the row's TYPE. */
static inline s64 nxs_csi2_row_s64(const struct nxs_csi2_row *row, u32 v)
{
	return (row->type & NXS_ROW_TYPE_SIGNED) ? (s64)(s32)v : (s64)v;
}

/* An enum row's value @i, inline or from its ROW_VALUES record. */
static inline u32 nxs_csi2_row_value(const struct nxs_csi2_row *row, unsigned int i)
{
	const u8 *p;

	if (!row->list)
		return i < NXS_ROW_VALUES_MAX ? row->values[i] : 0;
	p = row->list + 4 * i;

	return p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24);
}

/*
 * Whether the row takes a value in the state in effect, as its STEP cell
 * last read: a STEP of 0 says it takes none, and a commit of it ends FAULT.
 */
static inline bool nxs_csi2_row_takes_value(const struct nxs_csi2_row *row)
{
	return row->range[NXS_CSI2_STEP] != 0;
}

/* The temperature is painted by the device whatever its FLAGS say. */
static inline bool nxs_csi2_row_read_only(const struct nxs_csi2_row *row)
{
	/* The graph's owner sets a graph row from the rig's facts: the host offers it read-only. */
	return (row->flags & (NXS_ROW_FLAG_READ_ONLY | NXS_ROW_FLAG_GRAPH)) ||
	       row->id == NXS_PARAM_ID_TEMPERATURE_DC;
}

/*
 * A read-only row whose LIVE cell last read ROW_UNREAD: the device has no
 * reading yet, and the value is none.
 */
static inline bool nxs_csi2_row_unread(const struct nxs_csi2_row *row)
{
	return nxs_csi2_row_read_only(row) && row->live == NXS_ROW_UNREAD;
}

/* The slot of row_of a standard id takes, -1 for an id the core does not know. */
static inline int nxs_csi2_id_slot(unsigned int id)
{
	unsigned int low = NXS_CSI2_ID_LOW(id);

	switch (id >> NXS_PARAM_FAMILY_SHIFT) {
	case NXS_PARAM_FAMILY_GENERIC:
		return low < NXS_CSI2_GENERIC_IDS ? (int)low : -1;
	case NXS_PARAM_FAMILY_CAMERA:
		return low && low < NXS_CSI2_CAMERA_IDS ? (int)(NXS_CSI2_GENERIC_IDS + low) : -1;
	default:
		return -1;
	}
}

/* The row index of a standard id, -1 when the door serves no such row. */
static inline int nxs_csi2_core_row(const struct nxs_csi2_core *core, unsigned int id)
{
	int slot = nxs_csi2_id_slot(id);

	return slot >= 0 ? core->row_of[slot] : -1;
}

/* Row @index waits for nxs_csi2_core_commit_held(). */
static inline void nxs_csi2_core_hold(struct nxs_csi2_core *core, unsigned int index)
{
	nxs_csi2_rows_add(&core->held, index);
}

/* Whether any row waits for nxs_csi2_core_commit_held(). */
static inline bool nxs_csi2_core_holds(const struct nxs_csi2_core *core)
{
	return !nxs_csi2_rows_empty(&core->held);
}

bool nxs_csi2_core_unanswered(int err);
const char *nxs_csi2_core_state_name(u8 state);
u32 nxs_csi2_core_mbus_code(const struct nxs_csi2_mode *mode, u8 flips);

int nxs_csi2_core_init(struct nxs_csi2_core *core, struct i2c_client *client);
/*
 * nxs_csi2_core_init() for a caller that holds the lock of the controls'
 * handler, as a V4L2 sub-device's pad and stream operations do where the
 * state lock is the handler's.
 */
int nxs_csi2_core_init_locked(struct nxs_csi2_core *core, struct i2c_client *client);
int nxs_csi2_core_init_controls(struct nxs_csi2_core *core, struct v4l2_ctrl_handler *hdl,
				u32 cid_base);
int nxs_csi2_core_read_status(struct nxs_csi2_core *core, struct nxs_csi2_status *status);
int nxs_csi2_core_changed(struct nxs_csi2_core *core, const struct nxs_csi2_status *status,
			  bool *changed);
int nxs_csi2_core_reread(struct nxs_csi2_core *core);
/*
 * The re-read for a caller that holds the lock of the controls' handler, as
 * a V4L2 sub-device's pad and stream operations do where the state lock is
 * the handler's.
 */
int nxs_csi2_core_reread_locked(struct nxs_csi2_core *core);
int nxs_csi2_core_refresh(struct nxs_csi2_core *core);
/* The controls' ranges and activation from the rows as they stand, with no transfer. */
void nxs_csi2_core_sync_ranges(struct nxs_csi2_core *core);
int nxs_csi2_core_read_range(struct nxs_csi2_core *core, unsigned int index);
int nxs_csi2_core_read_live(struct nxs_csi2_core *core, unsigned int index);
void nxs_csi2_core_ctrl_range(const struct nxs_csi2_row *row, bool s32_only, s64 *min,
			      s64 *max, u64 *step, s64 *def);

void nxs_csi2_core_stage(struct nxs_csi2_core *core, unsigned int index, u32 value);
int nxs_csi2_core_commit(struct nxs_csi2_core *core, const struct nxs_csi2_rows *rows,
			 const char *what);
int nxs_csi2_core_commit_held(struct nxs_csi2_core *core);
int nxs_csi2_core_run_mode(struct nxs_csi2_core *core, const struct nxs_csi2_mode *mode);
int nxs_csi2_core_gate(struct nxs_csi2_core *core, bool on);
int nxs_csi2_core_park(struct nxs_csi2_core *core);

const struct nxs_csi2_mode *nxs_csi2_core_find_mode(const struct nxs_csi2_core *core, u8 value);
const struct nxs_csi2_mode *nxs_csi2_core_check_tree_mode(const struct nxs_csi2_core *core,
							  unsigned int i, u32 width, u32 height,
							  u8 bit_depth, u64 fps_milli);
void nxs_csi2_core_check_tree_count(const struct nxs_csi2_core *core, unsigned int tree_modes);
const struct nxs_csi2_mode *nxs_csi2_core_find_geometry(const struct nxs_csi2_core *core,
							u32 width, u32 height, u8 bit_depth);

#endif /* NXS_CSI2_CORE_H */

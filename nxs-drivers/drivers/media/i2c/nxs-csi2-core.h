/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * nxs-csi2-core.h - the NXS map's camera door, for the drivers that serve it
 *
 * Copyright (C) 2026 Aliensense
 *
 * The core reads the map of an NXS device at its camera door, builds the
 * rows and the mode table, commits the rows with the apply contract's
 * windows and offers the rows no adapter maps to a standard control as
 * custom controls. An adapter, the V4L2 sub-device driver or the tegracam
 * driver, owns the framework's registration, its standard controls, the
 * format and the stream, and calls the core under one lock of its own: the
 * core takes none. The constants are nxs-csi2-map.h, generated from the
 * map's one source. The contract is the NXS Interface Description, chapters
 * 3, 4, 6.4 and 12.
 */

#ifndef NXS_CSI2_CORE_H
#define NXS_CSI2_CORE_H

#include <linux/i2c.h>
#include <linux/types.h>

#include <media/v4l2-ctrls.h>

#include "nxs-csi2-map.h"

/* A commit that met the map INERT: nothing was written and the rows wait. */
#define NXS_CSI2_INERT			1

/* The standard ids the core knows; a row of an id past them is custom. */
#define NXS_CSI2_ID_COUNT		(NXS_PARAM_ID_TEMPERATURE_DC + 1)

/*
 * The core's controls, at the base the adapter passes: the trigger menu,
 * the black level, the temperature, then a custom row's at the base plus
 * 0x10 plus its row index. Both adapters pass NXS_CSI2_CID_BASE, so a
 * control carries one id on every host; it sits past every driver base
 * v4l2-controls.h allocates.
 */
#define NXS_CSI2_CID_BASE		(V4L2_CID_USER_BASE + 0x2000)
#define NXS_CSI2_CID_TRIGGER_MODE(base)	((base) + 0)
#define NXS_CSI2_CID_BLACK_LEVEL(base)	((base) + 1)
#define NXS_CSI2_CID_TEMPERATURE(base)	((base) + 2)
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
	u8 bit_depth;
	u8 lanes;
	u16 rate_mbps;
	u8 flags;
	u8 value;
	u8 phase;
	u32 fps_max_milli;
};

struct nxs_csi2_core;

/* One parameter row as the map serves it, and the control it became. */
struct nxs_csi2_row {
	struct nxs_csi2_core *core;
	char name[NXS_ROW_NAME_LEN + 1];
	/* An enum row's values, or a range row's declared min, max and step. */
	u32 values[NXS_ROW_VALUES_MAX];
	/* The range in effect: MIN, MAX and STEP. */
	u32 range[3];
	/* The value in effect at the last refresh. */
	u32 live;
	u32 def;
	u16 id;
	u8 index;
	/*
	 * The standard id the row is served as, 0 for a custom row. The core
	 * creates no control for EXPOSURE_US, FRAME_RATE_MFPS, GAIN_MDB and
	 * FLIPS, which the adapter maps to its framework's controls, nor for
	 * STREAM and MODE, which the stream gate and the format commit.
	 */
	u8 std;
	u8 type;
	u8 kind;
	u8 flags;
	u8 count;
	struct v4l2_ctrl *ctrl;
};

/* The header's capture fields a session reads: the state and the counts. */
struct nxs_csi2_status {
	u8 state;
	u8 stream;
	u8 current_mode;
	u8 param_count;
	u8 mode_count;
};

/* The map as the core read it, and the commit state. */
struct nxs_csi2_core {
	struct i2c_client *client;
	/* The map was read: the fields below stand. */
	bool read;
	u8 module_kind;
	u8 fw_major;
	u8 fw_minor;
	u8 param_count;
	u8 mode_count;
	struct nxs_csi2_mode modes[NXS_MODE_TABLE_MAX];
	/* The mode table's bytes as read: with the counts, the map's identity a re-read compares. */
	u8 table[NXS_MODE_TABLE_MAX * NXS_MODE_ENTRY_SIZE];
	struct nxs_csi2_row *rows;
	/* The row of a control whose row a re-read map no longer carries: its core is NULL. */
	struct nxs_csi2_row orphan;
	/* A standard id's row index, -1 for none. */
	s8 row_of[NXS_CSI2_ID_COUNT];
	/* Row i's value for its next commit. */
	u32 staged[NXS_MAX_ROWS];
	/* The rows staged while the map was INERT or no session was open, committed by nxs_csi2_core_commit_held(). */
	u64 held;
	/*
	 * A capture session is open on the link: the core commits. Otherwise a
	 * control set waits in the held rows until the next session's start, so
	 * the tool's bring-up of the port meets no commit of the driver's
	 * (Interface Description §6.4, the phase rule).
	 */
	bool session;
	/* The trigger conversion in effect, as the trigger row last read. */
	u32 trigger_live;
	/* LIVE or a range written into the core's controls: no commit. */
	bool syncing;
	u32 cid_base;
};

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

/* The temperature is painted by the device whatever its FLAGS say. */
static inline bool nxs_csi2_row_read_only(const struct nxs_csi2_row *row)
{
	/* The graph's owner sets a graph row from the rig's facts: the host offers it read-only. */
	return (row->flags & (NXS_ROW_FLAG_READ_ONLY | NXS_ROW_FLAG_GRAPH)) ||
	       row->id == NXS_PARAM_ID_TEMPERATURE_DC;
}

/* The row index of a standard id, -1 when the map serves no such row. */
static inline int nxs_csi2_core_row(const struct nxs_csi2_core *core, unsigned int id)
{
	return id < NXS_CSI2_ID_COUNT ? core->row_of[id] : -1;
}

/* The rows of @mask wait for nxs_csi2_core_commit_held(). */
static inline void nxs_csi2_core_hold(struct nxs_csi2_core *core, u64 mask)
{
	core->held |= mask;
}

bool nxs_csi2_core_unanswered(int err);
const char *nxs_csi2_core_state_name(u8 state);
u32 nxs_csi2_core_mbus_code(const struct nxs_csi2_mode *mode, u8 flips);

int nxs_csi2_core_init(struct nxs_csi2_core *core, struct i2c_client *client);
int nxs_csi2_core_init_controls(struct nxs_csi2_core *core, struct v4l2_ctrl_handler *hdl,
				u32 cid_base);
int nxs_csi2_core_read_status(struct nxs_csi2_core *core, struct nxs_csi2_status *status);
int nxs_csi2_core_changed(struct nxs_csi2_core *core, const struct nxs_csi2_status *status,
			  bool *changed);
int nxs_csi2_core_refresh(struct nxs_csi2_core *core);
int nxs_csi2_core_read_range(struct nxs_csi2_core *core, unsigned int index);
int nxs_csi2_core_read_live(struct nxs_csi2_core *core, unsigned int index);
void nxs_csi2_core_ctrl_range(const struct nxs_csi2_row *row, bool s32_only, s64 *min,
			      s64 *max, u64 *step, s64 *def);

void nxs_csi2_core_stage(struct nxs_csi2_core *core, unsigned int index, u32 value);
int nxs_csi2_core_commit(struct nxs_csi2_core *core, u64 mask, const char *what);
int nxs_csi2_core_commit_held(struct nxs_csi2_core *core);
int nxs_csi2_core_run_mode(struct nxs_csi2_core *core, const struct nxs_csi2_mode *mode,
			   bool park);
int nxs_csi2_core_gate(struct nxs_csi2_core *core, bool on);
int nxs_csi2_core_park(struct nxs_csi2_core *core);

const struct nxs_csi2_mode *nxs_csi2_core_find_mode(const struct nxs_csi2_core *core, u8 value);
const struct nxs_csi2_mode *nxs_csi2_core_find_geometry(const struct nxs_csi2_core *core,
							u32 width, u32 height, u8 bit_depth);

#endif /* NXS_CSI2_CORE_H */

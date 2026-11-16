/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * fake-door.h - a camera door of the NXS Device Interface, in memory
 *
 * Copyright (C) 2026 Aliensense
 *
 * The door serves the header, the region table, the CELLS region and the
 * paged DESCRIPTION window of the NXS Device Interface manual behind a
 * 16-bit register pointer. It marks the rows a STAGED write covers and
 * paints RUN_MS at the write's stop, applies the marked set at COMMIT 1
 * and discards it at COMMIT 2, ends FAULT for a marked row whose STEP
 * reads 0, and reads BUSY until the clock the host's sleeps advance has
 * passed the run. A RUN while the STREAM row reads 1 ends FAULT, other
 * than the trigger row alone, and a RUN leaves the STREAM row as it finds
 * it. It logs every transfer, so a test reads what the host did, and it
 * cuts a transfer where a test asks.
 */

#ifndef NXS_FAKE_DOOR_H
#define NXS_FAKE_DOOR_H

#include "kstub.h"
#include "nxs-csi2-map.h"

#define FAKE_LOG_MAX	4096
#define FAKE_ROWS_MAX	1024

/* One transfer as the door saw it. */
struct fake_xfer {
	bool read;
	bool locked;
	u16 reg;
	/* Data bytes written, or bytes read. */
	u32 len;
	/* The first data byte written. */
	u8 first;
	/* The clock at the transfer, in microseconds. */
	u64 at_us;
};

struct fake_door {
	u8 map[0x10000];
	/* The description and, once the epoch moves under a read, the one after it. */
	const u8 *desc;
	u32 desc_size;
	const u8 *next_desc;
	u32 next_desc_size;
	u16 desc_page;
	/* Window reads before the epoch moves and the description becomes next_desc; 0 never. */
	unsigned int move_after_reads;
	unsigned int window_reads;

	u16 cells;
	u16 capacity;
	u16 window;
	u16 param_count;
	/* Each row's KIND and read-only flag, as its PARAMS row says. */
	u8 kind[FAKE_ROWS_MAX];
	bool read_only[FAKE_ROWS_MAX];
	/* The STREAM row and the trigger row, -1 for none. */
	int stream_row;
	int trigger_row;
	/* The bound of each kind in ms, RUN_MS without the lateness. */
	u16 bound_ms[NXS_PARAM_KIND_GATE + 1];
	/* How long a run takes, in ms. */
	u16 run_ms;
	/* RUN_MS reads 0 whatever the door marked: the device's failure. */
	bool no_run_ms;
	/* The errno a refused set writes to ERROR, EINVAL when 0. */
	u8 refuse_error;

	bool marked[FAKE_ROWS_MAX];
	u8 state;
	u8 error;
	u8 apply_count;
	u8 fault_count;
	u8 epoch;
	u64 busy_until_us;
	u8 state_after;

	/* The next write to @cut_reg fails on the bus: before the door takes it, or after with @cut_taken. */
	int cut_reg;
	bool cut_taken;
	/* The next read at @cut_read fails on the bus. */
	int cut_read;
	/* The window read of this number, from 1, fails on the bus; 0 none. */
	unsigned int cut_window_read;
	unsigned int window_hits;

	u64 now_us;
	struct fake_xfer log[FAKE_LOG_MAX];
	unsigned int log_count;
	/* DESC_PAGE writes, in order. */
	u16 pages[16];
	unsigned int page_writes;
	/* COMMIT values written, in order. */
	u8 commits[64];
	unsigned int commit_count;

	struct i2c_adapter adapter;
	struct i2c_client client;
};

/*
 * A door of @capacity rows with its CELLS region at @cells, its window at
 * 0x8000 and the description @desc, MAP_ID, DOOR 2 and RELEASE 1.3.0, READY.
 */
void fake_door_init(struct fake_door *door, u16 cells, u16 capacity, const u8 *desc, u32 size);
/* The header's counts: PARAM_COUNT and PERSONALITY_COUNT. */
void fake_door_counts(struct fake_door *door, u16 param_count, u8 personalities);
/* Row @i's cells: LIVE, MIN, MAX and STEP. */
void fake_door_row(struct fake_door *door, unsigned int i, u8 kind, u32 live, u32 min, u32 max,
		   u32 step);
u32 fake_door_cell(const struct fake_door *door, unsigned int array, unsigned int i);
/* The door's clock is the test's: the host's sleeps advance it. */
void fake_door_use(struct fake_door *door);
void fake_door_clear_log(struct fake_door *door);

#endif /* NXS_FAKE_DOOR_H */

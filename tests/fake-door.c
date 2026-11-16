// SPDX-License-Identifier: GPL-2.0-only
/*
 * fake-door.c - a camera door of the NXS Device Interface, in memory
 *
 * Copyright (C) 2026 Aliensense
 */

#include "fake-door.h"

#include "nxs-csi2-map.h"

static struct fake_door *fake_clock;

void msleep(unsigned int ms)
{
	if (fake_clock)
		fake_clock->now_us += (u64)ms * 1000;
}

void usleep_range(unsigned long min_us, unsigned long max_us)
{
	(void)max_us;
	if (fake_clock)
		fake_clock->now_us += min_us;
}

static void put16(u8 *p, u16 v)
{
	p[0] = v & 0xff;
	p[1] = v >> 8;
}

static void put32(u8 *p, u32 v)
{
	put16(p, v & 0xffff);
	put16(p + 2, v >> 16);
}

static u32 get32(const u8 *p)
{
	return p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24);
}

static u16 fake_cell_addr(const struct fake_door *door, unsigned int array, unsigned int i)
{
	return door->cells + 4 * (array * door->capacity + i);
}

u32 fake_door_cell(const struct fake_door *door, unsigned int array, unsigned int i)
{
	return get32(door->map + fake_cell_addr(door, array, i));
}

static u16 fake_run_ms(const struct fake_door *door)
{
	u16 longest = 0;
	bool any = false;
	unsigned int i;

	for (i = 0; i < door->param_count; i++) {
		if (!door->marked[i])
			continue;
		any = true;
		if (door->kind[i] <= NXS_PARAM_KIND_GATE)
			longest = max(longest, door->bound_ms[door->kind[i]]);
	}

	return any && !door->no_run_ms ? longest + NXS_BOUND_LATENESS_MS : 0;
}

/* The header's live cells, as a read serves them. */
static void fake_paint(struct fake_door *door)
{
	u8 *m = door->map;

	m[NXS_MAP_REG_STATE] = door->now_us < door->busy_until_us ? NXS_MAP_STATE_BUSY :
				door->state;
	m[NXS_MAP_REG_ERROR] = door->error;
	m[NXS_MAP_REG_APPLY_COUNT] = door->apply_count;
	m[NXS_MAP_REG_FAULT_COUNT] = door->fault_count;
	m[NXS_MAP_REG_DESCRIPTOR_EPOCH] = door->epoch;
	put16(m + NXS_MAP_REG_RUN_MS, fake_run_ms(door));
	put32(m + NXS_MAP_REG_DESC_SIZE, door->desc_size);
	put16(m + NXS_MAP_REG_DESC_PAGE, door->desc_page);
}

void fake_door_init(struct fake_door *door, u16 cells, u16 capacity, const u8 *desc, u32 size)
{
	u8 *m;

	memset(door, 0, sizeof(*door));
	m = door->map;
	door->cells = cells;
	door->capacity = capacity;
	door->window = 0x8000;
	door->desc = desc;
	door->desc_size = size;
	door->state = NXS_MAP_STATE_READY;
	door->cut_reg = -1;
	door->cut_read = -1;
	door->stream_row = -1;
	door->trigger_row = -1;
	door->bound_ms[NXS_PARAM_KIND_APPLY] = 3;
	door->bound_ms[NXS_PARAM_KIND_RUN] = 400;
	door->bound_ms[NXS_PARAM_KIND_GATE] = 60;
	door->run_ms = 2;
	m[NXS_MAP_REG_MAP_ID] = NXS_MAP_ID_VALUE;
	m[NXS_MAP_REG_DOOR] = NXS_DOOR_DRIVER;
	m[NXS_MAP_REG_FW_VERSION_MAJOR] = 2;
	m[NXS_MAP_REG_FW_VERSION_MINOR] = 0;
	m[NXS_MAP_REG_FW_VERSION_PATCH] = 1;
	m[NXS_MAP_REG_MODULE_KIND] = NXS_MODULE_KIND_FIXED;
	put16(m + NXS_MAP_REG_ROW_CAPACITY, capacity);
	put32(m + NXS_MAP_REG_RELEASE, NXS_RELEASE_VALUE);
	m[NXS_MAP_REG_REGION_COUNT] = 3;
	m[NXS_MAP_REG_REGION_TABLE + 0] = NXS_REGION_TYPE_CELLS;
	m[NXS_MAP_REG_REGION_TABLE + 1] = NXS_REGION_FLAG_WRITABLE;
	put16(m + NXS_MAP_REG_REGION_TABLE + 2, cells);
	put16(m + NXS_MAP_REG_REGION_TABLE + 4, 4 * NXS_CELLS_ARRAYS * capacity);
	m[NXS_MAP_REG_REGION_TABLE + 8] = NXS_REGION_TYPE_DESCRIPTION;
	put16(m + NXS_MAP_REG_REGION_TABLE + 10, door->window);
	put16(m + NXS_MAP_REG_REGION_TABLE + 12, NXS_DESC_PAGE_SIZE);
	m[NXS_MAP_REG_REGION_TABLE + 16] = NXS_REGION_TYPE_DEVICE;
	put16(m + NXS_MAP_REG_REGION_TABLE + 18, 0x6000);
	put16(m + NXS_MAP_REG_REGION_TABLE + 20, 0x0400);
	fake_paint(door);
}

void fake_door_counts(struct fake_door *door, u16 param_count, u8 personalities)
{
	door->param_count = param_count;
	put16(door->map + NXS_MAP_REG_PARAM_COUNT, param_count);
	door->map[NXS_MAP_REG_PERSONALITY_COUNT] = personalities;
}

void fake_door_row(struct fake_door *door, unsigned int i, u8 kind, u32 live, u32 min, u32 max,
		   u32 step)
{
	door->kind[i] = kind;
	put32(door->map + fake_cell_addr(door, 0, i), live);
	put32(door->map + fake_cell_addr(door, 1, i), min);
	put32(door->map + fake_cell_addr(door, 2, i), max);
	put32(door->map + fake_cell_addr(door, 3, i), step);
}

void fake_door_clear_log(struct fake_door *door)
{
	door->log_count = 0;
	door->page_writes = 0;
	door->commit_count = 0;
}

/* Whether the marked set is the trigger row alone. */
static bool fake_trigger_alone(const struct fake_door *door)
{
	unsigned int i;

	for (i = 0; i < door->param_count; i++)
		if (door->marked[i] != ((int)i == door->trigger_row))
			return false;

	return door->trigger_row >= 0;
}

/* COMMIT 1 at a write's stop: the marked set judged, applied and cleared. */
static void fake_apply(struct fake_door *door)
{
	int kind = -1;
	bool bad = false;
	bool any = false;
	unsigned int i;

	if (door->now_us < door->busy_until_us || door->state == NXS_MAP_STATE_INERT)
		return;
	door->error = 0;
	for (i = 0; i < door->param_count; i++) {
		if (!door->marked[i])
			continue;
		any = true;
		/* A row whose STEP reads 0 takes no value now, and a KIND past GATE is none. */
		if (door->read_only[i] || door->kind[i] > NXS_PARAM_KIND_GATE ||
		    (kind >= 0 && kind != door->kind[i]) || !fake_door_cell(door, 3, i))
			bad = true;
		kind = door->kind[i];
	}
	/* A RUN while the STREAM row reads 1, other than the trigger row alone. */
	if (kind == NXS_PARAM_KIND_RUN && door->stream_row >= 0 &&
	    fake_door_cell(door, 0, door->stream_row) && !fake_trigger_alone(door))
		bad = true;
	door->busy_until_us = door->now_us + (u64)door->run_ms * 1000;
	if (bad) {
		door->state = NXS_MAP_STATE_FAULT;
		door->error = door->refuse_error ? door->refuse_error : NXS_DEV_EINVAL;
		door->fault_count++;
	} else {
		door->state = NXS_MAP_STATE_READY;
		for (i = 0; i < door->param_count; i++)
			if (door->marked[i])
				put32(door->map + fake_cell_addr(door, 0, i),
				      get32(door->map + fake_cell_addr(door, 4, i)));
		if (any)
			door->apply_count++;
	}
	memset(door->marked, 0, sizeof(door->marked));
}

static void fake_write(struct fake_door *door, u16 reg, const u8 *data, unsigned int len)
{
	u16 staged = fake_cell_addr(door, 4, 0);
	u16 staged_end = fake_cell_addr(door, 5, 0);
	int commit = -1;
	bool page = false;
	unsigned int k;

	for (k = 0; k < len; k++) {
		u16 a = reg + k;

		if (a == NXS_MAP_REG_DESC_PAGE || a == NXS_MAP_REG_DESC_PAGE + 1) {
			door->map[a] = data[k];
			page = true;
		} else if (a == NXS_MAP_REG_COMMIT) {
			commit = data[k];
		} else if (a >= staged && a < staged_end && (a - staged) / 4 < door->param_count) {
			door->map[a] = data[k];
			door->marked[(a - staged) / 4] = true;
		}
	}
	if (page) {
		door->desc_page = door->map[NXS_MAP_REG_DESC_PAGE] |
				  door->map[NXS_MAP_REG_DESC_PAGE + 1] << 8;
		if (door->page_writes < ARRAY_SIZE(door->pages))
			door->pages[door->page_writes] = door->desc_page;
		door->page_writes++;
	}
	if (commit < 0)
		return;
	if (door->commit_count < ARRAY_SIZE(door->commits))
		door->commits[door->commit_count++] = commit;
	if (commit == NXS_COMMIT_APPLY)
		fake_apply(door);
	else if (commit == NXS_COMMIT_DISCARD)
		memset(door->marked, 0, sizeof(door->marked));
}

static void fake_read(struct fake_door *door, u16 reg, u8 *buf, unsigned int len)
{
	bool window = false;
	unsigned int k;

	fake_paint(door);
	for (k = 0; k < len; k++) {
		u16 a = reg + k;

		if (a >= door->window && (u32)a < (u32)door->window + NXS_DESC_PAGE_SIZE) {
			u32 at = (u32)door->desc_page * NXS_DESC_PAGE_SIZE + (a - door->window);

			buf[k] = at < door->desc_size ? door->desc[at] : 0;
			window = true;
		} else {
			buf[k] = door->map[a];
		}
	}
	if (!window || !door->move_after_reads || ++door->window_reads != door->move_after_reads)
		return;
	door->epoch++;
	door->desc = door->next_desc;
	door->desc_size = door->next_desc_size;
}

static int fake_xfer(void *fake, struct i2c_msg *msgs, int n, bool locked)
{
	struct fake_door *door = fake;
	struct fake_xfer *x = door->log_count < FAKE_LOG_MAX ? &door->log[door->log_count++] : NULL;
	u16 reg;

	if (n < 1 || msgs[0].flags & I2C_M_RD || msgs[0].len < 2)
		return -EINVAL;
	reg = msgs[0].buf[0] << 8 | msgs[0].buf[1];
	if (x) {
		x->read = n == 2;
		x->locked = locked;
		x->reg = reg;
		x->len = n == 2 ? msgs[1].len : msgs[0].len - 2u;
		x->first = msgs[0].len > 2 ? msgs[0].buf[2] : 0;
		x->at_us = door->now_us;
	}
	if (n == 2 && door->cut_read == reg) {
		door->cut_read = -1;
		return -EIO;
	}
	if (n == 1 && door->cut_reg == reg) {
		door->cut_reg = -1;
		if (door->cut_taken)
			fake_write(door, reg, msgs[0].buf + 2, msgs[0].len - 2);
		return -EIO;
	}
	if (n == 2 && reg >= door->window && (u32)reg < (u32)door->window + NXS_DESC_PAGE_SIZE &&
	    ++door->window_hits == door->cut_window_read)
		return -EIO;
	if (n == 1)
		fake_write(door, reg, msgs[0].buf + 2, msgs[0].len - 2);
	else
		fake_read(door, reg, msgs[1].buf, msgs[1].len);

	return n;
}

void fake_door_use(struct fake_door *door)
{
	fake_clock = door;
	door->adapter.fake = door;
	door->adapter.xfer = fake_xfer;
	door->client.addr = 0x31;
	door->client.adapter = &door->adapter;
	door->client.dev.name = "nxs-csi2 fake";
}

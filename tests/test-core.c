// SPDX-License-Identifier: GPL-2.0-only
/*
 * test-core.c - nxs-csi2-core against a camera door of the NXS Device Interface
 *
 * Copyright (C) 2026 Aliensense
 *
 * Each test builds a description, serves it from a fake door and drives
 * the core as an adapter does: the probe through the header, the region
 * table and the paged window, the maskless commit and its windows, the
 * epoch the commit runs under, and the controls by row id and shape. The
 * last tests probe nxs-csi2.c, the sub-device, on the same door.
 */

#include "adapter.h"
#include "fake-door.h"
#include "nxs-csi2-core.h"

extern bool kstub_verbose;

static int failures;
static const char *current;

#define CHECK(cond) do { \
	if (!(cond)) { \
		fprintf(stderr, "FAIL %s: %s:%d: %s\n", current, __FILE__, __LINE__, #cond); \
		failures++; \
	} \
} while (0)

/* ---- descriptions ------------------------------------------------------ */

struct desc {
	u8 buf[3 * NXS_DESC_PAGE_SIZE];
	u32 len;
	/* Where the open trailer's count byte sits. */
	u32 trailer;
};

static void desc_trailer(struct desc *d)
{
	d->trailer = d->len;
	d->buf[d->len++] = 0;
}

static u8 *desc_record(struct desc *d, u8 type, u32 len)
{
	u8 *rec = d->buf + d->len + 3;

	d->buf[d->len] = type;
	d->buf[d->len + 1] = len & 0xff;
	d->buf[d->len + 2] = len >> 8;
	memset(rec, 0, len);
	d->len += 3 + len;
	d->buf[d->trailer]++;

	return rec;
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

/* One PARAMS row: a range of min, max and step, or an enum of @n u16 values. */
static void row_range(u8 *row, const char *name, const char *unit, u16 id, u8 kind, u8 flags,
		      u32 min, u32 max, u32 step, u32 def)
{
	strncpy((char *)row + NXS_ROW_FIELD_NAME, name, NXS_ROW_NAME_LEN - 1);
	strncpy((char *)row + NXS_ROW_FIELD_UNIT, unit, NXS_ROW_UNIT_LEN);
	put16(row + NXS_ROW_FIELD_ID, id);
	row[NXS_ROW_FIELD_TYPE] = NXS_ROW_TYPE_RANGE;
	row[NXS_ROW_FIELD_KIND] = kind;
	row[NXS_ROW_FIELD_FLAGS] = flags;
	row[NXS_ROW_FIELD_COUNT] = 3;
	put32(row + NXS_ROW_FIELD_DEFAULT, def);
	put32(row + NXS_ROW_FIELD_VALUES, min);
	put32(row + NXS_ROW_FIELD_VALUES + 4, max);
	put32(row + NXS_ROW_FIELD_VALUES + 8, step);
}

static void row_enum(u8 *row, const char *name, u16 id, u8 kind, u8 n, u32 def)
{
	unsigned int i;

	strncpy((char *)row + NXS_ROW_FIELD_NAME, name, NXS_ROW_NAME_LEN - 1);
	put16(row + NXS_ROW_FIELD_ID, id);
	row[NXS_ROW_FIELD_KIND] = kind;
	row[NXS_ROW_FIELD_COUNT] = n;
	put32(row + NXS_ROW_FIELD_DEFAULT, def);
	for (i = 0; i < n; i++)
		put16(row + NXS_ROW_FIELD_VALUES + 2 * i, i);
}

/* The rows of the manual's IMX900 head, §14.1: seven rows, the camera's. */
#define IMX900_ROWS	7
/* Where a camera trailer's PARAMS rows start: its count, the identity record and the PARAMS head. */
#define ROWS_AT		(1 + 3 + 64 + 3)

static void imx900_rows(u8 *rows)
{
	row_range(rows + 0 * NXS_ROW_SIZE, "exposure", "us", NXS_PARAM_ID_EXPOSURE_US,
		  NXS_PARAM_KIND_APPLY, 0, 4, 82241492, 1, 4216);
	row_range(rows + 1 * NXS_ROW_SIZE, "fps", "mfps", NXS_PARAM_ID_FRAME_RATE_MFPS,
		  NXS_PARAM_KIND_APPLY, 0, 12, 215843, 1, 215843);
	row_range(rows + 2 * NXS_ROW_SIZE, "gain", "mdB", NXS_PARAM_ID_GAIN_MDB,
		  NXS_PARAM_KIND_APPLY, 0, 0, 48000, 100, 0);
	row_enum(rows + 3 * NXS_ROW_SIZE, "trigger", NXS_PARAM_ID_TRIGGER, NXS_PARAM_KIND_RUN, 2, 0);
	row_enum(rows + 4 * NXS_ROW_SIZE, "stream", NXS_PARAM_ID_STREAM, NXS_PARAM_KIND_GATE, 2, 0);
	row_enum(rows + 5 * NXS_ROW_SIZE, "mode", NXS_PARAM_ID_MODE, NXS_PARAM_KIND_RUN, 4, 0);
	row_range(rows + 6 * NXS_ROW_SIZE, "line_time", "ns", NXS_PARAM_ID_LINE_TIME_NS,
		  NXS_PARAM_KIND_RUN, NXS_ROW_FLAG_GRAPH, 4902, 8215, 1, 4902);
}

/* The modes the door serves: the manual's four, or the first of them alone. */
static unsigned int mode_total = 4;

/* The CSI-2 data type each mode row names: the manual's RAW10 and RAW12 unless a test sets another. */
static u8 mode_type[4] = {
	NXS_CSI2_DATA_TYPE_RAW10, NXS_CSI2_DATA_TYPE_RAW10,
	NXS_CSI2_DATA_TYPE_RAW12, NXS_CSI2_DATA_TYPE_RAW10,
};

/*
 * The first mode_total of the manual's four modes, in a CAMERA_MAP record
 * after its head and law rows. Byte 4 of a row is reserved and reads 0.
 */
static void camera_map(struct desc *d)
{
	static const struct { u16 w, h; u32 fps; u8 phase; } modes[4] = {
		{ 1032, 776, 215843, 2 }, { 1920, 1080, 160488, 0 },
		{ 2064, 1552, 72067, 0 }, { 2064, 1552, 117028, 0 },
	};
	const unsigned int head = NXS_CAMERA_MAP_HEAD_SIZE;
	const unsigned int law = NXS_CAMERA_LAW_ROW_SIZE;
	u8 *rec = desc_record(d, NXS_TRAILER_RECORD_CAMERA_MAP,
			      head + mode_total * (law + NXS_MODE_ENTRY_SIZE) + NXS_CAMERA_MAP_TAIL_SIZE);
	unsigned int i;

	rec[NXS_CAMERA_MAP_FIELD_MODE_COUNT] = mode_total;
	put16(rec + NXS_CAMERA_MAP_FIELD_GAIN_FACTOR, 1);
	for (i = 0; i < mode_total; i++) {
		u8 *m = rec + head + mode_total * law + i * NXS_MODE_ENTRY_SIZE;

		put32(rec + head + i * law + NXS_CAMERA_LAW_FIELD_LINE_TIME_NS, 4902);
		put16(m + NXS_MODE_ENTRY_WIDTH, modes[i].w);
		put16(m + NXS_MODE_ENTRY_HEIGHT, modes[i].h);
		m[NXS_MODE_ENTRY_LANES] = 4;
		put16(m + NXS_MODE_ENTRY_RATE_MBPS, 2376);
		m[NXS_MODE_ENTRY_DATA_TYPE] = mode_type[i];
		m[NXS_MODE_ENTRY_FLAGS] = (i ? 0 : NXS_MODE_FLAG_DEFAULT) | NXS_MODE_FLAG_TRIGGERABLE;
		put32(m + NXS_MODE_ENTRY_FPS_MAX_MILLI, modes[i].fps);
		m[NXS_MODE_ENTRY_MODE_VALUE] = i;
		m[NXS_MODE_ENTRY_PIXEL_PHASE] = modes[i].phase;
	}
}

/* A LIST row after the rows: its id, its value count, and its ROW_VALUES record ahead of the PARAMS. */
struct list_shape {
	u16 id;
	unsigned int n;
	bool first;
};

static const struct list_shape filter_list = { NXS_PARAM_ID_CUSTOM, 20, false };

/* The ROW_VALUES record of row @index: 1000, 1007 and on. */
static void list_record(struct desc *d, unsigned int index, const struct list_shape *list)
{
	u8 *rec = desc_record(d, NXS_TRAILER_RECORD_ROW_VALUES, 4 + 4 * list->n);
	unsigned int i;

	put16(rec, index);
	put16(rec + 2, list->n);
	for (i = 0; i < list->n; i++)
		put32(rec + 4 + 4 * i, 1000 + 7 * i);
}

/*
 * The camera trailer: an identity, @pad bytes of a record no host reads,
 * the seven rows and @extra custom range rows after them, a LIST row of
 * @list's shape when @list is not NULL, then the camera map.
 */
static void camera_trailer(struct desc *d, u32 pad, unsigned int extra,
			   const struct list_shape *list)
{
	unsigned int n = IMX900_ROWS + extra + !!list;
	unsigned int i;
	u8 *rows;

	desc_trailer(d);
	desc_record(d, NXS_TRAILER_RECORD_IDENTITY, 64);
	if (pad)
		desc_record(d, NXS_TRAILER_RECORD_VENDOR_BASE, pad);
	if (list && list->first)
		list_record(d, IMX900_ROWS + extra, list);
	rows = desc_record(d, NXS_TRAILER_RECORD_PARAMS, n * NXS_ROW_SIZE);
	imx900_rows(rows);
	for (i = 0; i < extra; i++) {
		char name[16];

		snprintf(name, sizeof(name), "custom_%u", i);
		row_range(rows + (IMX900_ROWS + i) * NXS_ROW_SIZE, name, "", NXS_PARAM_ID_CUSTOM,
			  NXS_PARAM_KIND_APPLY, 0, 0, 1000, 1, 0);
	}
	if (list) {
		u8 *row = rows + (IMX900_ROWS + extra) * NXS_ROW_SIZE;

		row_enum(row, "filter", list->id, NXS_PARAM_KIND_APPLY, 0, 1000);
		row[NXS_ROW_FIELD_TYPE] = NXS_ROW_TYPE_LIST;
		if (!list->first)
			list_record(d, IMX900_ROWS + extra, list);
	}
	camera_map(d);
}

static struct desc d1, d2;
static struct fake_door door;
static struct nxs_csi2_core core;

/* The door with the manual's head, its CELLS region at @cells for @capacity rows. */
static void serve_list(u16 cells, u16 capacity, u32 pad, unsigned int extra,
		       const struct list_shape *list)
{
	unsigned int n = IMX900_ROWS + extra + !!list;
	unsigned int i;

	memset(&d1, 0, sizeof(d1));
	camera_trailer(&d1, pad, extra, list);
	fake_door_init(&door, cells, capacity, d1.buf, d1.len);
	fake_door_counts(&door, n, 1);
	door.trigger_row = 3;
	door.stream_row = 4;
	fake_door_row(&door, 0, NXS_PARAM_KIND_APPLY, 4216, 4, 19000, 1);
	fake_door_row(&door, 1, NXS_PARAM_KIND_APPLY, 215843, 12, 215843, 1);
	fake_door_row(&door, 2, NXS_PARAM_KIND_APPLY, 0, 0, 48000, 100);
	fake_door_row(&door, 3, NXS_PARAM_KIND_RUN, 0, 0, 1, 1);
	fake_door_row(&door, 4, NXS_PARAM_KIND_GATE, 0, 0, 1, 1);
	fake_door_row(&door, 5, NXS_PARAM_KIND_RUN, 0, 0, 3, 1);
	fake_door_row(&door, 6, NXS_PARAM_KIND_RUN, 4902, 4902, 8215, 1);
	for (i = IMX900_ROWS; i < n; i++)
		fake_door_row(&door, i, NXS_PARAM_KIND_APPLY, 0, 0,
			      i < IMX900_ROWS + extra ? 1000 : list->n - 1, 1);
	fake_door_use(&door);
	kstub_devm_release(&door.client.dev);
	memset(&core, 0, sizeof(core));
}

static void serve(u16 cells, u16 capacity, u32 pad, unsigned int extra, bool list)
{
	serve_list(cells, capacity, pad, extra, list ? &filter_list : NULL);
}

static void begin(const char *name)
{
	current = name;
}

/* The bus lock was taken and released in pairs, and no transfer broke its rule. */
static void check_lock(void)
{
	CHECK(door.adapter.locked == 0);
	CHECK(door.adapter.lock_errors == 0);
	CHECK(door.adapter.quirk_errors == 0);
}

/* ---- the probe --------------------------------------------------------- */

static void test_probe_reads_the_header_and_the_description(void)
{
	const struct nxs_csi2_mode *mode;

	begin("probe reads the header and the description");
	serve(0x1000, 256, 0, 0, false);
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(core.read);
	CHECK(core.param_count == IMX900_ROWS && core.door_rows == IMX900_ROWS);
	CHECK(core.mode_count == 4 && core.capacity == 256 && core.cells == 0x1000);
	CHECK(core.release[0] == 1 && core.release[1] == 3 && core.release[2] == 0);
	CHECK(!strcmp(core.rows[0].name, "exposure") && core.rows[0].id == NXS_PARAM_ID_EXPOSURE_US);
	CHECK(core.rows[0].live == 4216 && core.rows[0].range[NXS_CSI2_MAX] == 19000);
	CHECK(core.rows[2].range[NXS_CSI2_STEP] == 100);
	CHECK(core.rows[6].live == 4902 && core.rows[6].range[NXS_CSI2_MIN] == 4902);
	CHECK(nxs_csi2_core_row(&core, NXS_PARAM_ID_MODE) == 5);
	CHECK(nxs_csi2_core_row(&core, NXS_PARAM_ID_STREAM) == 4);
	mode = nxs_csi2_core_find_mode(&core, 2);
	CHECK(mode && mode->width == 2064 && mode->height == 1552 && mode->bit_depth == 12);
	CHECK(mode && mode->lanes == 4 && mode->rate_mbps == 2376 && mode->phase == 0);
	CHECK(core.modes[0].flags & NXS_MODE_FLAG_DEFAULT);
	CHECK(nxs_csi2_core_mbus_code(&core.modes[0], 0) == MEDIA_BUS_FMT_SGBRG10_1X10);
	check_lock();
}

static void test_probe_writes_nothing_for_one_page(void)
{
	unsigned int i;
	bool wrote = false;

	begin("a description of one page reads with no write");
	serve(0x1000, 256, 0, 0, false);
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	for (i = 0; i < door.log_count; i++)
		wrote |= !door.log[i].read;
	CHECK(!wrote);
	CHECK(door.log[0].read && door.log[0].reg == 0 && door.log[0].len == NXS_MAP_LATCH_SIZE);
}

static void test_probe_gates_the_door(void)
{
	begin("probe takes MAP_ID 0xC4, DOOR 2 and a MINOR at or below its own");
	serve(0x1000, 256, 0, 0, false);
	door.map[NXS_MAP_REG_RELEASE + 1] = 2;
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(core.release[1] == 2);

	serve(0x1000, 256, 0, 0, false);
	door.map[NXS_MAP_REG_RELEASE + 1] = 4;
	CHECK(nxs_csi2_core_init(&core, &door.client) == -ENODEV);

	serve(0x1000, 256, 0, 0, false);
	door.map[NXS_MAP_REG_RELEASE] = 2;
	CHECK(nxs_csi2_core_init(&core, &door.client) == -ENODEV);

	serve(0x1000, 256, 0, 0, false);
	door.map[NXS_MAP_REG_DOOR] = NXS_DOOR_MAIN;
	CHECK(nxs_csi2_core_init(&core, &door.client) == -ENODEV);

	serve(0x1000, 256, 0, 0, false);
	door.map[NXS_MAP_REG_MAP_ID] = 0xAB;
	CHECK(nxs_csi2_core_init(&core, &door.client) == -ENODEV);

	serve(0x1000, 256, 0, 0, false);
	door.map[NXS_MAP_REG_PERSONALITY_COUNT] = 0;
	CHECK(nxs_csi2_core_init(&core, &door.client) == -ENOENT);

	serve(0x1000, 256, 0, 0, false);
	door.map[NXS_MAP_REG_PARAM_COUNT] = IMX900_ROWS + 1;
	CHECK(nxs_csi2_core_init(&core, &door.client) == -EBADMSG);
}

static void test_probe_line_counts_the_modes(void)
{
	begin("the probe line names 1 mode and 4 modes");
	mode_total = 1;
	serve(0x1000, 256, 0, 0, false);
	kstub_log_clear();
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(core.mode_count == 1);
	CHECK(kstub_log_holds(", 7 rows, 1 mode, "));
	mode_total = 4;
	serve(0x1000, 256, 0, 0, false);
	kstub_log_clear();
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(kstub_log_holds(", 7 rows, 4 modes, "));
}

/* The manual's mode rows: a RAW10 row gives depth 10 and a RAW12 row depth 12, byte 4 at 0. */
static void test_modes_take_their_depth_from_the_data_type(void)
{
	const u8 *row;

	begin("a RAW10 and a RAW12 mode row give depths 10 and 12 and their bus codes, byte 4 at 0");
	serve(0x1000, 256, 0, 0, false);
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(core.mode_count == 4);
	row = core.desc + ROWS_AT + IMX900_ROWS * NXS_ROW_SIZE + 3 + NXS_CAMERA_MAP_HEAD_SIZE +
	      4 * NXS_CAMERA_LAW_ROW_SIZE;
	CHECK(row[4] == 0 && row[NXS_MODE_ENTRY_DATA_TYPE] == NXS_CSI2_DATA_TYPE_RAW10);
	CHECK(row[2 * NXS_MODE_ENTRY_SIZE + 4] == 0);
	CHECK(row[2 * NXS_MODE_ENTRY_SIZE + NXS_MODE_ENTRY_DATA_TYPE] == NXS_CSI2_DATA_TYPE_RAW12);
	CHECK(core.modes[0].bit_depth == 10 && core.modes[2].bit_depth == 12);
	CHECK(nxs_csi2_core_mbus_code(&core.modes[0], 0) == MEDIA_BUS_FMT_SGBRG10_1X10);
	CHECK(nxs_csi2_core_mbus_code(&core.modes[1], 0) == MEDIA_BUS_FMT_SRGGB10_1X10);
	CHECK(nxs_csi2_core_mbus_code(&core.modes[2], 0) == MEDIA_BUS_FMT_SRGGB12_1X12);
	CHECK(nxs_csi2_core_mbus_code(&core.modes[2], NXS_FLIP_X | NXS_FLIP_Y) ==
	      MEDIA_BUS_FMT_SBGGR12_1X12);
	CHECK(nxs_csi2_core_find_geometry(&core, 2064, 1552, 12) == &core.modes[2]);
	CHECK(nxs_csi2_core_find_geometry(&core, 2064, 1552, 10) == &core.modes[3]);
	check_lock();
}

/* Mode 1 of the manual's four rows names @code: the core leaves it out and logs it, the code in hex. */
static void check_mode_left_out(u8 code, const char *line)
{
	mode_type[1] = code;
	serve(0x1000, 256, 0, 0, false);
	mode_type[1] = NXS_CSI2_DATA_TYPE_RAW10;
	kstub_log_clear();
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(core.mode_count == 3);
	CHECK(!nxs_csi2_core_find_mode(&core, 1));
	CHECK(nxs_csi2_core_find_mode(&core, 2) == &core.modes[1]);
	CHECK(!nxs_csi2_core_find_geometry(&core, 1920, 1080, 0));
	CHECK(kstub_log_holds(line));
	CHECK(kstub_log_holds(", 7 rows, 3 modes, "));
	check_lock();
}

static void test_a_yuv_mode_is_left_out(void)
{
	begin("a mode row of data type 0x1E, YUV 4:2:2 8-bit, is left out and logged");
	check_mode_left_out(NXS_CSI2_DATA_TYPE_YUV422_8,
			    "[warn] mode 1: CSI-2 data type 0x1e, no media bus code for it, not offered");
}

static void test_a_mode_of_an_unnamed_data_type_is_left_out(void)
{
	begin("a mode row of a data type the map header does not name is left out and logged");
	CHECK(!nxs_csi2_data_type_sample_bits(0x29) && !nxs_csi2_data_type_pixel_bits(0x29));
	check_mode_left_out(0x29, "[warn] mode 1: CSI-2 data type 0x29, no media bus code for it, not offered");
}

static void test_probe_is_ready_on_a_description_and_a_personality(void)
{
	begin("probe is ready on DESC_SIZE and PERSONALITY_COUNT, the modes from the CAMERA_MAP record");
	serve(0x1000, 256, 0, 0, false);
	/* The header names no payload: 0x0007 to 0x0009 are reserved and read 0. */
	CHECK(!door.map[0x0007] && !door.map[0x0008] && !door.map[0x0009]);
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(core.mode_count == 4);

	serve(0x1000, 256, 0, 0, false);
	door.desc_size = 0;
	CHECK(nxs_csi2_core_init(&core, &door.client) == -ENOENT);
	CHECK(!core.read);

	serve(0x1000, 256, 0, 0, false);
	door.map[NXS_MAP_REG_PERSONALITY_COUNT] = 0;
	CHECK(nxs_csi2_core_init(&core, &door.client) == -ENOENT);
	check_lock();
}

static void test_rows_by_family(void)
{
	const u32 at = 1 + 3 + 64 + 3 + NXS_ROW_FIELD_ID;
	struct v4l2_ctrl_handler hdl = { };

	begin("a row's id is a family byte above an id: the camera family's are standard, others custom");
	serve(0x1000, 256, 0, 3, false);
	put16(d1.buf + at + (IMX900_ROWS + 0) * NXS_ROW_SIZE, NXS_PARAM_ID_POSITION);
	put16(d1.buf + at + (IMX900_ROWS + 1) * NXS_ROW_SIZE, NXS_PARAM_ID_ENABLE);
	put16(d1.buf + at + (IMX900_ROWS + 2) * NXS_ROW_SIZE, NXS_PARAM_ID_PART);
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(nxs_csi2_core_row(&core, NXS_PARAM_ID_EXPOSURE_US) == 0);
	CHECK(nxs_csi2_core_row(&core, NXS_PARAM_ID_TRIGGER) == 3);
	CHECK(nxs_csi2_core_row(&core, NXS_PARAM_ID_STREAM) == 4);
	CHECK(nxs_csi2_core_row(&core, NXS_PARAM_ID_LINE_TIME_NS) == 6);
	CHECK(nxs_csi2_core_row(&core, NXS_PARAM_ID_POSITION) == -1);
	CHECK(nxs_csi2_core_row(&core, NXS_PARAM_ID_ENABLE) == -1);
	CHECK(nxs_csi2_core_row(&core, NXS_PARAM_ID_PART) == -1);
	CHECK(!core.rows[IMX900_ROWS].std && !core.rows[IMX900_ROWS + 1].std &&
	      !core.rows[IMX900_ROWS + 2].std);
	CHECK(nxs_csi2_core_init_controls(&core, &hdl, NXS_CSI2_CID_BASE) == 0);
	CHECK(kstub_ctrl_find(&hdl, NXS_CSI2_CID_ROW(NXS_CSI2_CID_BASE, IMX900_ROWS)));
	CHECK(kstub_ctrl_find(&hdl, NXS_CSI2_CID_ROW(NXS_CSI2_CID_BASE, IMX900_ROWS + 1)));
	CHECK(kstub_ctrl_find(&hdl, NXS_CSI2_CID_ROW(NXS_CSI2_CID_BASE, IMX900_ROWS + 2)));
	kstub_ctrl_handler_free(&hdl);
	check_lock();
}

static void test_probe_finds_the_cells_by_the_region_table(void)
{
	begin("the cells sit where the region table says, at the declared capacity");
	serve(0x2400, 300, 0, 0, false);
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(core.cells == 0x2400 && core.capacity == 300);
	CHECK(core.rows[0].live == 4216 && core.rows[0].range[NXS_CSI2_MAX] == 19000);
	CHECK(core.rows[2].range[NXS_CSI2_STEP] == 100);

	serve(0x2400, 300, 0, 0, false);
	door.map[NXS_MAP_REG_REGION_TABLE] = NXS_REGION_TYPE_SAMPLES;
	CHECK(nxs_csi2_core_init(&core, &door.client) == -ENODEV);
}

static void test_probe_reads_a_description_of_two_pages(void)
{
	begin("a description past 32 KB reads page by page through DESC_PAGE");
	serve(0x1000, 256, 40000, 0, false);
	CHECK(door.desc_size > NXS_DESC_PAGE_SIZE);
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(core.param_count == IMX900_ROWS && !strcmp(core.rows[6].name, "line_time"));
	CHECK(nxs_csi2_core_find_mode(&core, 3) != NULL);
	/* The door serves page 0 already: the core writes page 1, then 0 back. */
	CHECK(door.page_writes == 2);
	CHECK(door.pages[0] == 1 && door.pages[1] == 0);
	CHECK(door.desc_page == 0);
	check_lock();
}

static void test_probe_resets_a_page_left_set(void)
{
	begin("a DESC_PAGE another host left set reads page 0 first, and ends at 0 after an error");
	serve(0x1000, 256, 0, 0, false);
	door.desc_page = 1;
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(core.param_count == IMX900_ROWS);
	CHECK(door.page_writes == 1 && door.pages[0] == 0 && door.desc_page == 0);
	check_lock();

	serve(0x1000, 256, 40000, 0, false);
	door.cut_window_read = 2;
	CHECK(nxs_csi2_core_init(&core, &door.client) == -EIO);
	CHECK(door.desc_page == 0);
	check_lock();
}

static void test_probe_takes_a_list_before_its_rows(void)
{
	static const struct list_shape ahead = { NXS_PARAM_ID_CUSTOM, 20, true };

	begin("a ROW_VALUES record ahead of the PARAMS record binds to its row");
	serve_list(0x1000, 256, 0, 0, &ahead);
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(core.param_count == IMX900_ROWS + 1);
	if (core.param_count <= IMX900_ROWS)
		return;
	CHECK(core.rows[IMX900_ROWS].count == 20);
	CHECK(nxs_csi2_row_value(&core.rows[IMX900_ROWS], 19) == 1000 + 7 * 19);
}

static void test_probe_reads_again_when_the_epoch_moved(void)
{
	begin("a description that moved under its read is read again");
	serve(0x1000, 256, 0, 0, false);
	memset(&d2, 0, sizeof(d2));
	camera_trailer(&d2, 0, 0, NULL);
	strcpy((char *)d2.buf + 1 + 3 + 64 + 3 + 6 * NXS_ROW_SIZE, "line_ns");
	door.next_desc = d2.buf;
	door.next_desc_size = d2.len;
	door.move_after_reads = 1;
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(!strcmp(core.rows[6].name, "line_ns"));
	CHECK(core.epoch == 1);
}

static void test_probe_reads_in_the_adapters_pieces(void)
{
	static const struct i2c_adapter_quirks quirks = { .max_read_len = NXS_MAP_LATCH_SIZE };
	unsigned int window = 0;
	unsigned int i;

	begin("the description reads in the pieces the adapter takes");
	serve(0x1000, 256, 0, 0, false);
	door.adapter.quirks = &quirks;
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(door.log[0].reg == 0 && door.log[0].len == NXS_MAP_LATCH_SIZE);
	for (i = 1; i < door.log_count; i++)
		window += door.log[i].reg >= 0x8000;
	CHECK(window == DIV_ROUND_UP(door.desc_size, NXS_MAP_LATCH_SIZE));
	CHECK(core.param_count == IMX900_ROWS);
	check_lock();
}

static void test_probe_refuses_an_adapter_short_of_the_header(void)
{
	static const struct i2c_adapter_quirks quirks = { .max_read_len = NXS_MAP_LATCH_SIZE - 1 };

	begin("an adapter that cannot read the header at once is refused before any transfer");
	serve(0x1000, 256, 0, 0, false);
	door.adapter.quirks = &quirks;
	CHECK(nxs_csi2_core_init(&core, &door.client) == -EOPNOTSUPP);
	CHECK(door.log_count == 0);
	check_lock();
}

static void test_rows_past_255_and_a_list(void)
{
	struct v4l2_ctrl_handler hdl = { };
	struct nxs_csi2_rows rows = { };
	const unsigned int last = IMX900_ROWS + 293 - 1;
	struct v4l2_ctrl *ctrl;

	begin("u16 row indices, and an enum of 20 values from its ROW_VALUES record");
	serve(0x1000, 512, 0, 293, true);
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(core.param_count == 301);
	CHECK(!strcmp(core.rows[last].name, "custom_292") && core.rows[last].index == last);
	CHECK(core.rows[300].count == 20 && nxs_csi2_row_value(&core.rows[300], 19) == 1000 + 7 * 19);
	CHECK(core.rows[300].range[NXS_CSI2_MAX] == 19);

	nxs_csi2_core_session(&core, true);
	nxs_csi2_core_stage(&core, last, 777);
	nxs_csi2_rows_add(&rows, last);
	CHECK(nxs_csi2_core_commit(&core, &rows, "row 299") == 0);
	CHECK(fake_door_cell(&door, 0, last) == 777);
	CHECK(fake_door_cell(&door, 4, last) == 777);

	CHECK(nxs_csi2_core_init_controls(&core, &hdl, NXS_CSI2_CID_BASE) == 0);
	ctrl = kstub_ctrl_find(&hdl, NXS_CSI2_CID_ROW(NXS_CSI2_CID_BASE, 300));
	CHECK(ctrl && ctrl->type == V4L2_CTRL_TYPE_INTEGER_MENU && ctrl->maximum == 19);
	CHECK(ctrl && ctrl->qmenu_int[19] == 1000 + 7 * 19);
	CHECK(ctrl && ctrl->default_value == 0);
	if (ctrl) {
		ctrl->val = 19;
		CHECK(ctrl->ops->s_ctrl(ctrl) == 0);
		CHECK(fake_door_cell(&door, 0, 300) == 1000 + 7 * 19);
	}
	CHECK(kstub_ctrl_find(&hdl, NXS_CSI2_CID_ROW(NXS_CSI2_CID_BASE, last)) != NULL);
	kstub_ctrl_handler_free(&hdl);
	check_lock();
}

/* ---- the commit -------------------------------------------------------- */

static void commit_rows(const unsigned int *idx, unsigned int n, const u32 *values, int expect)
{
	struct nxs_csi2_rows rows = { };
	unsigned int i;

	for (i = 0; i < n; i++) {
		nxs_csi2_core_stage(&core, idx[i], values[i]);
		nxs_csi2_rows_add(&rows, idx[i]);
	}
	CHECK(nxs_csi2_core_commit(&core, &rows, "test") == expect);
}

static void ready(void)
{
	serve(0x1000, 256, 0, 0, false);
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	nxs_csi2_core_session(&core, true);
	fake_door_clear_log(&door);
}

static void test_commit_is_maskless(void)
{
	static const unsigned int idx[] = { 0, 2 };
	static const u32 values[] = { 20000, 1200 };
	u16 step = 0x1000 + 12 * 256;
	u16 staged = 0x1000 + 16 * 256;
	const struct fake_xfer *x = door.log;

	begin("a commit reads STEP, writes the staged cells, reads RUN_MS, writes COMMIT 1 and reads STATE once");
	ready();
	commit_rows(idx, 2, values, 0);
	CHECK(door.log_count == 8);
	CHECK(x[0].read && x[0].reg == NXS_MAP_REG_STATE &&
	      x[0].len == NXS_MAP_REG_DESCRIPTOR_EPOCH + 1 - NXS_MAP_REG_STATE);
	CHECK(x[1].read && x[1].reg == step && x[1].len == 4);
	CHECK(x[2].read && x[2].reg == step + 8 && x[2].len == 4);
	CHECK(!x[3].read && x[3].reg == staged && x[3].len == 4);
	CHECK(!x[4].read && x[4].reg == staged + 8 && x[4].len == 4);
	CHECK(x[5].read && x[5].reg == NXS_MAP_REG_RUN_MS && x[5].len == 2);
	CHECK(!x[6].read && x[6].reg == NXS_MAP_REG_COMMIT && x[6].len == 1 &&
	      x[6].first == NXS_COMMIT_APPLY);
	CHECK(x[7].read && x[7].reg == NXS_MAP_REG_STATE);
	/* The host waits the painted RUN_MS, the APPLY bound 3 ms plus 5. */
	CHECK(x[7].at_us - x[6].at_us >= 8000);
	CHECK(x[0].locked && x[1].locked && x[7].locked);
	CHECK(fake_door_cell(&door, 0, 0) == 20000 && fake_door_cell(&door, 0, 2) == 1200);
	CHECK(door.apply_count == 1 && door.commit_count == 1);
	check_lock();
}

static void test_commit_writes_adjacent_rows_at_once(void)
{
	static const unsigned int idx[] = { 0, 1, 2 };
	static const u32 values[] = { 5000, 100000, 600 };

	begin("adjacent rows read their STEP in one read and stage in one write");
	ready();
	commit_rows(idx, 3, values, 0);
	CHECK(door.log_count == 6);
	CHECK(door.log[1].read && door.log[1].reg == 0x1000 + 12 * 256 && door.log[1].len == 12);
	CHECK(!door.log[2].read && door.log[2].reg == 0x1000 + 16 * 256 && door.log[2].len == 12);
	CHECK(fake_door_cell(&door, 0, 1) == 100000);
}

static void test_commit_leaves_out_a_row_that_takes_no_value(void)
{
	static const unsigned int idx[] = { 0, 1 };
	static const u32 values[] = { 5000, 100000 };
	static const unsigned int rate[] = { 1 };

	begin("a row whose STEP reads 0 is never staged");
	ready();
	/* The rate takes no value now: a trigger the host tool set, say. */
	fake_door_row(&door, 1, NXS_PARAM_KIND_APPLY, 215843, 12, 215843, 0);
	commit_rows(idx, 2, values, 0);
	CHECK(door.fault_count == 0 && door.apply_count == 1);
	CHECK(fake_door_cell(&door, 0, 0) == 5000);
	CHECK(fake_door_cell(&door, 4, 1) == 0 && fake_door_cell(&door, 0, 1) == 215843);
	CHECK(!door.log[2].read && door.log[2].len == 4);
	CHECK(!nxs_csi2_row_takes_value(&core.rows[1]));

	fake_door_clear_log(&door);
	commit_rows(rate, 1, values + 1, 0);
	CHECK(door.commit_count == 0 && door.fault_count == 0);
	check_lock();
}

static void test_commit_refuses_a_run_ms_of_0(void)
{
	static const unsigned int idx[] = { 2 };
	static const u32 values[] = { 1200 };

	begin("a RUN_MS of 0 for a staged set discards it with no COMMIT 1");
	ready();
	door.no_run_ms = true;
	commit_rows(idx, 1, values, -EPROTO);
	CHECK(door.commit_count == 1 && door.commits[0] == NXS_COMMIT_DISCARD);
	CHECK(door.apply_count == 0 && fake_door_cell(&door, 0, 2) == 0);
	check_lock();
}

static void test_commit_splits_the_kinds(void)
{
	static const unsigned int idx[] = { 2, 3 };
	static const u32 values[] = { 600, 1 };

	begin("rows of two kinds commit as two sets");
	ready();
	commit_rows(idx, 2, values, 0);
	CHECK(door.commit_count == 2 && door.apply_count == 2);
	CHECK(fake_door_cell(&door, 0, 3) == 1);
}

static void test_a_row_of_the_retired_kind_3(void)
{
	static const unsigned int idx[] = { IMX900_ROWS };
	static const u32 values[] = { 600 };
	struct v4l2_ctrl_handler hdl = { };

	begin("a row whose KIND reads the retired 3 takes no control, and the door refuses its commit");
	serve(0x1000, 256, 0, 1, false);
	d1.buf[1 + 3 + 64 + 3 + IMX900_ROWS * NXS_ROW_SIZE + NXS_ROW_FIELD_KIND] = 3;
	door.kind[IMX900_ROWS] = 3;
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(core.rows[IMX900_ROWS].kind == 3);
	CHECK(nxs_csi2_core_init_controls(&core, &hdl, NXS_CSI2_CID_BASE) == 0);
	CHECK(!kstub_ctrl_find(&hdl, NXS_CSI2_CID_ROW(NXS_CSI2_CID_BASE, IMX900_ROWS)));
	kstub_ctrl_handler_free(&hdl);
	nxs_csi2_core_session(&core, true);
	fake_door_clear_log(&door);
	commit_rows(idx, 1, values, -EINVAL);
	CHECK(door.fault_count == 1 && fake_door_cell(&door, 0, IMX900_ROWS) == 0);
	check_lock();
}

static void test_commit_reports_the_fault(void)
{
	static const unsigned int idx[] = { 2 };
	static const u32 values[] = { 600 };

	begin("a set the door refuses ends FAULT with its errno");
	ready();
	door.read_only[2] = true;
	commit_rows(idx, 1, values, -EINVAL);
	CHECK(door.fault_count == 1 && fake_door_cell(&door, 0, 2) == 0);
	check_lock();
}

static void test_commit_runs_under_the_epoch(void)
{
	static const unsigned int idx[] = { 2 };
	static const u32 values[] = { 600 };
	struct nxs_csi2_status status;
	bool changed = false;

	begin("a commit under an epoch that moved ends stale and writes nothing");
	ready();
	door.epoch++;
	commit_rows(idx, 1, values, -ESTALE);
	CHECK(door.log_count == 1 && door.log[0].read);
	CHECK(nxs_csi2_core_read_status(&core, &status) == 0);
	CHECK(status.epoch == 1);
	CHECK(nxs_csi2_core_changed(&core, &status, &changed) == 0 && changed);
	check_lock();
}

static void test_commit_discards_a_cut_staging_write(void)
{
	static const unsigned int idx[] = { 0, 2 };
	static const u32 values[] = { 20000, 1200 };

	begin("a staging write the bus cut ends with COMMIT 2");
	ready();
	door.cut_reg = 0x1000 + 16 * 256 + 8;
	commit_rows(idx, 2, values, -EIO);
	CHECK(door.commit_count == 1 && door.commits[0] == NXS_COMMIT_DISCARD);
	CHECK(!door.marked[0] && door.apply_count == 0);
	CHECK(fake_door_cell(&door, 0, 0) == 4216);
	check_lock();
}

static void test_commit_tells_a_cut_commit_by_the_counts(void)
{
	static const unsigned int idx[] = { 2 };
	static const u32 values[] = { 1200 };
	unsigned int n;

	begin("a COMMIT write the bus cut is told by the counts after RUN_MS");
	ready();
	door.cut_reg = NXS_MAP_REG_COMMIT;
	door.cut_taken = true;
	commit_rows(idx, 1, values, 0);
	n = door.log_count;
	CHECK(n == 6 && door.log[n - 1].read && door.log[n - 1].at_us - door.log[n - 2].at_us >= 8000);
	CHECK(fake_door_cell(&door, 0, 2) == 1200);

	ready();
	door.cut_reg = NXS_MAP_REG_COMMIT;
	door.cut_taken = false;
	commit_rows(idx, 1, values, -EIO);
	CHECK(door.commit_count == 1 && door.commits[0] == NXS_COMMIT_DISCARD);
	CHECK(fake_door_cell(&door, 0, 2) == 0 && door.apply_count == 0);
	check_lock();
}

static void test_commit_waits_for_a_session(void)
{
	static const unsigned int idx[] = { 2 };
	static const u32 values[] = { 600 };

	begin("a set with no session open waits in the held rows");
	ready();
	nxs_csi2_core_session(&core, false);
	commit_rows(idx, 1, values, 0);
	CHECK(door.log_count == 0 && nxs_csi2_core_holds(&core));
	nxs_csi2_core_session(&core, true);
	CHECK(nxs_csi2_core_commit_held(&core) == 0);
	CHECK(!nxs_csi2_core_holds(&core) && fake_door_cell(&door, 0, 2) == 600);
}

static void test_commit_ends_busy_at_step_1(void)
{
	static const unsigned int idx[] = { 2 };
	static const u32 values[] = { 600 };

	begin("a door BUSY at step 1 ends the attempt with EBUSY, the bus released and nothing written");
	ready();
	door.busy_until_us = door.now_us + 100000;
	commit_rows(idx, 1, values, -EBUSY);
	CHECK(door.log_count == 1 && door.log[0].read && door.log[0].reg == NXS_MAP_REG_STATE);
	CHECK(!nxs_csi2_core_holds(&core) && fake_door_cell(&door, 4, 2) == 0);
	check_lock();
}

static void test_commit_holds_at_an_inert_door(void)
{
	static const unsigned int idx[] = { 2 };
	static const u32 values[] = { 600 };

	begin("a door INERT at step 1 takes nothing, and the rows wait in the held rows");
	ready();
	door.state = NXS_MAP_STATE_INERT;
	commit_rows(idx, 1, values, NXS_CSI2_INERT);
	CHECK(door.log_count == 1 && door.log[0].read);
	CHECK(nxs_csi2_core_holds(&core) && fake_door_cell(&door, 4, 2) == 0);
	check_lock();
	door.state = NXS_MAP_STATE_READY;
	CHECK(nxs_csi2_core_commit_held(&core) == 0);
	CHECK(!nxs_csi2_core_holds(&core) && fake_door_cell(&door, 0, 2) == 600);
	check_lock();
}

static void test_commit_ends_busy_past_run_ms(void)
{
	static const unsigned int idx[] = { 2 };
	static const u32 values[] = { 600 };
	unsigned int n;

	begin("BUSY at the one STATE read after RUN_MS is the device's failure: EBUSY");
	ready();
	door.run_ms = 50;
	commit_rows(idx, 1, values, -EBUSY);
	n = door.log_count;
	CHECK(door.commit_count == 1 && door.commits[0] == NXS_COMMIT_APPLY);
	CHECK(door.log[n - 1].read && door.log[n - 1].reg == NXS_MAP_REG_STATE);
	CHECK(door.log[n - 1].at_us - door.log[n - 2].at_us < 50000);
	check_lock();
}

static void test_commit_discards_a_cut_run_ms_read(void)
{
	static const unsigned int idx[] = { 2 };
	static const u32 values[] = { 600 };

	begin("a RUN_MS read the bus cut ends with COMMIT 2 and no COMMIT 1");
	ready();
	door.cut_read = NXS_MAP_REG_RUN_MS;
	commit_rows(idx, 1, values, -EIO);
	CHECK(door.commit_count == 1 && door.commits[0] == NXS_COMMIT_DISCARD);
	CHECK(!door.marked[2] && door.apply_count == 0 && fake_door_cell(&door, 0, 2) == 0);
	check_lock();
}

static void test_commit_held_commits_the_trigger_first(void)
{
	static const unsigned int idx[] = { 2, 3 };
	static const u32 values[] = { 600, 1 };
	const u16 staged = 0x1000 + 16 * 256;
	unsigned int writes = 0;
	u16 first = 0;
	u16 second = 0;
	unsigned int i;

	begin("the held rows commit the trigger row alone first, then the rest");
	ready();
	nxs_csi2_core_session(&core, false);
	commit_rows(idx, 2, values, 0);
	CHECK(door.log_count == 0 && nxs_csi2_core_holds(&core));
	nxs_csi2_core_session(&core, true);
	CHECK(nxs_csi2_core_commit_held(&core) == 0);
	for (i = 0; i < door.log_count; i++) {
		const struct fake_xfer *x = &door.log[i];

		if (x->read || x->reg < staged || x->reg >= staged + 4 * 256)
			continue;
		if (!writes++)
			first = x->reg;
		else
			second = x->reg;
	}
	CHECK(writes == 2 && first == staged + 4 * 3 && second == staged + 4 * 2);
	CHECK(door.commit_count == 2 && door.apply_count == 2);
	CHECK(!nxs_csi2_core_holds(&core));
	check_lock();
}

static void test_commit_translates_the_devices_errnos(void)
{
	static const struct { u8 dev; int linux_errno; } table[] = {
		{ NXS_DEV_ETIMEDOUT, ETIMEDOUT }, { NXS_DEV_ESTALE, ESTALE },
		{ NXS_DEV_ENOTSUP, EOPNOTSUPP }, { NXS_DEV_EILSEQ, EILSEQ },
		{ NXS_DEV_ECANCELED, ECANCELED }, { NXS_DEV_EBADMSG, EBADMSG },
		{ NXS_DEV_ENODATA, ENODATA }, { NXS_DEV_EBUSY, EBUSY },
	};
	static const unsigned int idx[] = { 2 };
	static const u32 values[] = { 600 };
	unsigned int i;

	begin("a FAULT's ERROR reads in the kernel's numbering, the six that differ among them");
	for (i = 0; i < ARRAY_SIZE(table); i++) {
		ready();
		door.read_only[2] = true;
		door.refuse_error = table[i].dev;
		commit_rows(idx, 1, values, -table[i].linux_errno);
	}
	check_lock();
}

static void test_mode_run(void)
{
	begin("a mode run stages the MODE row and commits it alone");
	ready();
	CHECK(nxs_csi2_core_run_mode(&core, nxs_csi2_core_find_mode(&core, 2)) == 0);
	CHECK(fake_door_cell(&door, 4, 5) == 2 && fake_door_cell(&door, 0, 5) == 2);
	CHECK(door.commit_count == 1);
	/* The RUN bound 400 ms plus 5. */
	CHECK(door.log[door.log_count - 1].at_us - door.log[door.log_count - 2].at_us >= 405000);
	check_lock();
}

static void test_mode_run_while_streaming(void)
{
	static const unsigned int mode_row[] = { 5 };
	static const unsigned int trigger_row[] = { 3 };
	static const u32 two[] = { 2 };
	static const u32 one[] = { 1 };

	begin("a RUN while streaming is refused, the trigger alone runs, and a mode run parks first and leaves the gate");
	ready();
	fake_door_row(&door, 4, NXS_PARAM_KIND_GATE, 1, 0, 1, 1);
	commit_rows(mode_row, 1, two, -EINVAL);
	CHECK(door.fault_count == 1 && fake_door_cell(&door, 0, 5) == 0);
	commit_rows(trigger_row, 1, one, 0);
	CHECK(fake_door_cell(&door, 0, 3) == 1 && fake_door_cell(&door, 0, 4) == 1);
	fake_door_clear_log(&door);
	CHECK(nxs_csi2_core_run_mode(&core, nxs_csi2_core_find_mode(&core, 2)) == 0);
	CHECK(door.commit_count == 2 && door.fault_count == 1);
	CHECK(fake_door_cell(&door, 0, 4) == 0 && fake_door_cell(&door, 0, 5) == 2);
	check_lock();
}

static void test_status_reads_the_stream_and_the_mode_in_the_rows(void)
{
	struct nxs_csi2_status status;

	begin("a session reads the stream and the mode in the LIVE cells of the STREAM and MODE rows");
	ready();
	door.map[0x0007] = 0xaa;
	door.map[0x0008] = 0xaa;
	door.map[0x0009] = 0xaa;
	fake_door_row(&door, 4, NXS_PARAM_KIND_GATE, 1, 0, 1, 1);
	fake_door_row(&door, 5, NXS_PARAM_KIND_RUN, 2, 0, 3, 1);
	CHECK(nxs_csi2_core_read_status(&core, &status) == 0);
	CHECK(status.stream == 1 && status.current_mode == 2);
	/* The two rows are adjacent: one read of both LIVE cells after the header's. */
	CHECK(door.log_count == 2 && door.log[1].read && door.log[1].reg == 0x1000 + 4 * 4 &&
	      door.log[1].len == 8);
	door.state = NXS_MAP_STATE_INERT;
	CHECK(nxs_csi2_core_read_status(&core, &status) == 0);
	CHECK(status.current_mode == NXS_CSI2_NO_MODE);
	door.state = NXS_MAP_STATE_READY;
	fake_door_clear_log(&door);
	CHECK(nxs_csi2_core_park(&core) == 0);
	CHECK(door.commit_count == 1 && fake_door_cell(&door, 0, 4) == 0);
	check_lock();
}

static void test_tree_modes_against_the_map(void)
{
	begin("a tree mode of another camera and a rate within 0.1 fps log a note, a rate past it warns");
	ready();
	kstub_log_clear();
	CHECK(nxs_csi2_core_check_tree_mode(&core, 2, 2064, 1552, 10, 117028) == &core.modes[3]);
	CHECK(!kstub_log_holds("mode2"));
	CHECK(nxs_csi2_core_check_tree_mode(&core, 1, 1920, 1080, 10, 160491) == &core.modes[1]);
	CHECK(kstub_log_holds("[info] mode1: 160.491 fps in the device tree, the camera map tops at 160.488"));
	CHECK(nxs_csi2_core_check_tree_mode(&core, 1, 1920, 1080, 10, 160588) == &core.modes[1]);
	CHECK(!nxs_csi2_core_check_tree_mode(&core, 3, 4096, 2160, 10, 30000));
	CHECK(kstub_log_holds("[info] mode3: 4096x2160 at 10 bits is in the device tree and not in the camera map"));
	nxs_csi2_core_check_tree_count(&core, 3);
	CHECK(kstub_log_holds("[info] the device tree names 3 modes, the camera map serves 4"));
	CHECK(!kstub_log_holds("[warn]"));
	CHECK(nxs_csi2_core_check_tree_mode(&core, 1, 1920, 1080, 10, 160589) == &core.modes[1]);
	CHECK(kstub_log_holds("[warn] mode1: 160.589 fps in the device tree, the camera map tops at 160.488"));
}

/* ---- the controls ------------------------------------------------------ */

/* The bytes of the first record of @type in @d's first trailer, NULL for none. */
static u8 *desc_find(struct desc *d, u8 type)
{
	u8 *p = d->buf + 1;
	unsigned int r;

	for (r = 0; r < d->buf[0]; r++) {
		if (p[0] == type)
			return p + 3;
		p += 3 + (p[1] | p[2] << 8);
	}

	return NULL;
}

/* The door serves @d next, under a moved epoch, with @rows rows. */
static void serve_next(const struct desc *d, unsigned int rows)
{
	door.desc = d->buf;
	door.desc_size = d->len;
	door.epoch++;
	fake_door_counts(&door, rows, 1);
}

static void test_controls_follow_their_rows_by_name(void)
{
	struct v4l2_ctrl_handler hdl = { };
	struct nxs_csi2_status status;
	struct v4l2_ctrl *ctrl;
	bool changed = false;

	begin("a re-read description keeps a custom row's control on the row of its name and shape");
	serve(0x1000, 256, 0, 2, false);
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(nxs_csi2_core_init_controls(&core, &hdl, NXS_CSI2_CID_BASE) == 0);
	ctrl = kstub_ctrl_find(&hdl, NXS_CSI2_CID_ROW(NXS_CSI2_CID_BASE, 8));
	CHECK(ctrl && ctrl->priv == &core.rows[8]);

	/* The next description drops custom_0: custom_1 moves to row 7. */
	memset(&d2, 0, sizeof(d2));
	camera_trailer(&d2, 0, 1, NULL);
	strcpy((char *)d2.buf + 1 + 3 + 64 + 3 + 7 * NXS_ROW_SIZE, "custom_1");
	door.desc = d2.buf;
	door.desc_size = d2.len;
	door.epoch++;
	fake_door_counts(&door, IMX900_ROWS + 1, 1);
	CHECK(nxs_csi2_core_read_status(&core, &status) == 0);
	CHECK(nxs_csi2_core_changed(&core, &status, &changed) == 0 && changed);
	CHECK(nxs_csi2_core_reread(&core) == 0);
	CHECK(ctrl && ctrl->priv == &core.rows[7] && !(ctrl->flags & V4L2_CTRL_FLAG_INACTIVE));
	ctrl = kstub_ctrl_find(&hdl, NXS_CSI2_CID_ROW(NXS_CSI2_CID_BASE, 7));
	CHECK(ctrl && ctrl->priv == &core.orphan && (ctrl->flags & V4L2_CTRL_FLAG_INACTIVE));
	kstub_ctrl_handler_free(&hdl);
}

static void test_controls_follow_a_standard_row_by_its_id(void)
{
	struct v4l2_ctrl_handler hdl = { };
	struct v4l2_ctrl *ctrl;

	begin("a standard row's control follows its id whatever the device names the row");
	serve(0x1000, 256, 0, 0, false);
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(nxs_csi2_core_init_controls(&core, &hdl, NXS_CSI2_CID_BASE) == 0);
	ctrl = kstub_ctrl_find(&hdl, NXS_CSI2_CID_ROW(NXS_CSI2_CID_BASE, 6));
	CHECK(ctrl && ctrl->priv == &core.rows[6]);
	memset(&d2, 0, sizeof(d2));
	camera_trailer(&d2, 0, 0, NULL);
	strcpy((char *)d2.buf + ROWS_AT + 6 * NXS_ROW_SIZE, "tline");
	serve_next(&d2, IMX900_ROWS);
	CHECK(nxs_csi2_core_reread(&core) == 0);
	CHECK(!strcmp(core.rows[6].name, "tline"));
	CHECK(ctrl && ctrl->priv == &core.rows[6] && !(ctrl->flags & V4L2_CTRL_FLAG_INACTIVE));
	CHECK(core.detached_count == 0);
	kstub_ctrl_handler_free(&hdl);
}

static void test_controls_refuse_a_row_of_another_shape(void)
{
	struct v4l2_ctrl_handler hdl = { };
	struct v4l2_ctrl *ctrl;

	begin("a row of the same name and another shape takes no detached control");
	serve(0x1000, 256, 0, 0, true);
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(nxs_csi2_core_init_controls(&core, &hdl, NXS_CSI2_CID_BASE) == 0);
	ctrl = kstub_ctrl_find(&hdl, NXS_CSI2_CID_ROW(NXS_CSI2_CID_BASE, IMX900_ROWS));
	CHECK(ctrl && ctrl->type == V4L2_CTRL_TYPE_INTEGER_MENU && ctrl->maximum == 19);

	/* The enum row filter becomes a range row of the same name. */
	memset(&d2, 0, sizeof(d2));
	camera_trailer(&d2, 0, 1, NULL);
	strcpy((char *)d2.buf + ROWS_AT + IMX900_ROWS * NXS_ROW_SIZE, "filter");
	serve_next(&d2, IMX900_ROWS + 1);
	fake_door_row(&door, IMX900_ROWS, NXS_PARAM_KIND_APPLY, 0, 0, 1000, 1);
	CHECK(nxs_csi2_core_reread(&core) == 0);
	CHECK(core.rows[IMX900_ROWS].type & NXS_ROW_TYPE_RANGE);
	CHECK(ctrl && ctrl->priv == &core.orphan && (ctrl->flags & V4L2_CTRL_FLAG_INACTIVE));
	CHECK(core.rows[IMX900_ROWS].ctrl == NULL && core.detached_count == 1);
	CHECK(nxs_csi2_core_refresh(&core) == 0);
	CHECK(ctrl && ctrl->maximum == 19);

	kstub_ctrl_handler_free(&hdl);

	/* An enum of the same name and count with other values is another shape too. */
	serve(0x1000, 256, 0, 0, true);
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(nxs_csi2_core_init_controls(&core, &hdl, NXS_CSI2_CID_BASE) == 0);
	ctrl = kstub_ctrl_find(&hdl, NXS_CSI2_CID_ROW(NXS_CSI2_CID_BASE, IMX900_ROWS));
	memcpy(&d2, &d1, sizeof(d2));
	put32(desc_find(&d2, NXS_TRAILER_RECORD_ROW_VALUES) + 4, 999);
	serve_next(&d2, IMX900_ROWS + 1);
	CHECK(nxs_csi2_core_reread(&core) == 0);
	CHECK(nxs_csi2_row_value(&core.rows[IMX900_ROWS], 0) == 999);
	CHECK(ctrl && ctrl->priv == &core.orphan && (ctrl->flags & V4L2_CTRL_FLAG_INACTIVE));

	/* The same values again: the control comes back. */
	put32(desc_find(&d2, NXS_TRAILER_RECORD_ROW_VALUES) + 4, 1000);
	serve_next(&d2, IMX900_ROWS + 1);
	CHECK(nxs_csi2_core_reread(&core) == 0);
	CHECK(ctrl && ctrl->priv == &core.rows[IMX900_ROWS] && !(ctrl->flags & V4L2_CTRL_FLAG_INACTIVE));
	kstub_ctrl_handler_free(&hdl);

	/* An unsigned range whose MAX passes S32_MAX takes a 64-bit control, never the 32-bit one. */
	serve(0x1000, 256, 0, 1, false);
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(nxs_csi2_core_init_controls(&core, &hdl, NXS_CSI2_CID_BASE) == 0);
	ctrl = kstub_ctrl_find(&hdl, NXS_CSI2_CID_ROW(NXS_CSI2_CID_BASE, IMX900_ROWS));
	CHECK(ctrl && ctrl->type == V4L2_CTRL_TYPE_INTEGER && ctrl->maximum == 1000);
	memset(&d2, 0, sizeof(d2));
	camera_trailer(&d2, 0, 1, NULL);
	put32(d2.buf + ROWS_AT + IMX900_ROWS * NXS_ROW_SIZE + NXS_ROW_FIELD_VALUES + 4, 0xFFFFFFF0);
	serve_next(&d2, IMX900_ROWS + 1);
	fake_door_row(&door, IMX900_ROWS, NXS_PARAM_KIND_APPLY, 0, 0, 0xFFFFFFF0, 1);
	CHECK(nxs_csi2_core_reread(&core) == 0);
	CHECK(ctrl && ctrl->priv == &core.orphan && (ctrl->flags & V4L2_CTRL_FLAG_INACTIVE));
	CHECK(core.rows[IMX900_ROWS].ctrl == NULL && core.detached_count == 1);
	CHECK(nxs_csi2_core_refresh(&core) == 0);
	CHECK(ctrl && ctrl->maximum == 1000);

	/* Back under S32_MAX, the row takes its control again. */
	put32(d2.buf + ROWS_AT + IMX900_ROWS * NXS_ROW_SIZE + NXS_ROW_FIELD_VALUES + 4, 1000);
	serve_next(&d2, IMX900_ROWS + 1);
	fake_door_row(&door, IMX900_ROWS, NXS_PARAM_KIND_APPLY, 0, 0, 1000, 1);
	CHECK(nxs_csi2_core_reread(&core) == 0);
	CHECK(ctrl && ctrl->priv == &core.rows[IMX900_ROWS] && !(ctrl->flags & V4L2_CTRL_FLAG_INACTIVE));
	kstub_ctrl_handler_free(&hdl);
}

static void test_controls_follow_rows_that_moved_to_a_larger_array(void)
{
	struct v4l2_ctrl_handler hdl = { };
	struct v4l2_ctrl *ctrl;

	begin("a re-read of more rows than the core holds puts every control on the new rows");
	serve(0x1000, 256, 0, 0, false);
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(core.rows_cap == IMX900_ROWS);
	CHECK(nxs_csi2_core_init_controls(&core, &hdl, NXS_CSI2_CID_BASE) == 0);
	memset(&d2, 0, sizeof(d2));
	camera_trailer(&d2, 0, 3, NULL);
	serve_next(&d2, IMX900_ROWS + 3);
	CHECK(nxs_csi2_core_reread(&core) == 0);
	CHECK(core.rows_cap == IMX900_ROWS + 3);
	ctrl = kstub_ctrl_find(&hdl, NXS_CSI2_CID_TRIGGER_MODE(NXS_CSI2_CID_BASE));
	CHECK(ctrl && ctrl->priv == &core.rows[3] && core.rows[3].ctrl == ctrl);
	ctrl = kstub_ctrl_find(&hdl, NXS_CSI2_CID_ROW(NXS_CSI2_CID_BASE, 6));
	CHECK(ctrl && ctrl->priv == &core.rows[6] && core.rows[6].ctrl == ctrl);
	CHECK(core.detached_count == 0);
	kstub_ctrl_handler_free(&hdl);
}

static void test_inline_enum_values_are_unsigned(void)
{
	struct v4l2_ctrl_handler hdl = { };
	u8 *row = d1.buf + ROWS_AT + IMX900_ROWS * NXS_ROW_SIZE;
	struct v4l2_ctrl *ctrl;

	begin("a signed enum row's inline u16 values read unsigned: 40000 stays 40000");
	serve(0x1000, 256, 0, 1, false);
	memset(row, 0, NXS_ROW_SIZE);
	row_enum(row, "levels", NXS_PARAM_ID_CUSTOM, NXS_PARAM_KIND_APPLY, 2, 40000);
	row[NXS_ROW_FIELD_TYPE] = NXS_ROW_TYPE_SIGNED;
	put16(row + NXS_ROW_FIELD_VALUES + 2, 40000);
	fake_door_row(&door, IMX900_ROWS, NXS_PARAM_KIND_APPLY, 0, 0, 1, 1);
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(nxs_csi2_row_value(&core.rows[IMX900_ROWS], 1) == 40000);
	CHECK(nxs_csi2_core_init_controls(&core, &hdl, NXS_CSI2_CID_BASE) == 0);
	ctrl = kstub_ctrl_find(&hdl, NXS_CSI2_CID_ROW(NXS_CSI2_CID_BASE, IMX900_ROWS));
	CHECK(ctrl && ctrl->qmenu_int[1] == 40000 && ctrl->default_value == 1);
	nxs_csi2_core_session(&core, true);
	if (ctrl) {
		ctrl->val = 1;
		CHECK(ctrl->ops->s_ctrl(ctrl) == 0);
	}
	CHECK(fake_door_cell(&door, 4, IMX900_ROWS) == 40000);
	kstub_ctrl_handler_free(&hdl);
	check_lock();
}

static void test_a_row_with_no_reading_yet(void)
{
	struct v4l2_ctrl_handler hdl = { };
	u8 *row = d1.buf + ROWS_AT + IMX900_ROWS * NXS_ROW_SIZE;
	struct v4l2_ctrl *ctrl;

	begin("a read-only row that serves ROW_UNREAD reads no value, its control inactive until a reading");
	serve(0x1000, 256, 0, 1, false);
	memset(row, 0, NXS_ROW_SIZE);
	row_range(row, "temperature", "dC", NXS_PARAM_ID_TEMPERATURE_DC, NXS_PARAM_KIND_APPLY,
		  NXS_ROW_FLAG_READ_ONLY, (u32)-400, 1250, 1, 0);
	row[NXS_ROW_FIELD_TYPE] |= NXS_ROW_TYPE_SIGNED;
	fake_door_row(&door, IMX900_ROWS, NXS_PARAM_KIND_APPLY, NXS_ROW_UNREAD, (u32)-400, 1250, 1);
	door.read_only[IMX900_ROWS] = true;
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(nxs_csi2_row_unread(&core.rows[IMX900_ROWS]));
	CHECK(nxs_csi2_core_init_controls(&core, &hdl, NXS_CSI2_CID_BASE) == 0);
	ctrl = kstub_ctrl_find(&hdl, NXS_CSI2_CID_TEMPERATURE(NXS_CSI2_CID_BASE));
	CHECK(ctrl && (ctrl->flags & V4L2_CTRL_FLAG_INACTIVE) && ctrl->val == 0);
	if (!ctrl)
		return;
	CHECK(ctrl->ops->g_volatile_ctrl(ctrl) == 0);
	CHECK((ctrl->flags & V4L2_CTRL_FLAG_INACTIVE) && ctrl->val == 0);
	CHECK(nxs_csi2_core_refresh(&core) == 0);
	CHECK(ctrl->flags & V4L2_CTRL_FLAG_INACTIVE);

	/* The first reading. */
	fake_door_row(&door, IMX900_ROWS, NXS_PARAM_KIND_APPLY, (u32)-35, (u32)-400, 1250, 1);
	CHECK(ctrl->ops->g_volatile_ctrl(ctrl) == 0);
	CHECK(!(ctrl->flags & V4L2_CTRL_FLAG_INACTIVE) && ctrl->val == -35);
	kstub_ctrl_handler_free(&hdl);
	check_lock();
}

static void test_controls_of_a_row_that_takes_no_value(void)
{
	struct v4l2_ctrl_handler hdl = { };
	struct v4l2_ctrl *ctrl;

	begin("a row whose STEP reads 0 keeps it, and its control reads inactive");
	serve(0x1000, 256, 0, 1, false);
	fake_door_row(&door, IMX900_ROWS, NXS_PARAM_KIND_APPLY, 0, 0, 0, 0);
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(core.rows[IMX900_ROWS].range[NXS_CSI2_STEP] == 0);
	CHECK(core.rows[IMX900_ROWS].range[NXS_CSI2_MAX] == 0);
	CHECK(nxs_csi2_core_init_controls(&core, &hdl, NXS_CSI2_CID_BASE) == 0);
	ctrl = kstub_ctrl_find(&hdl, NXS_CSI2_CID_ROW(NXS_CSI2_CID_BASE, IMX900_ROWS));
	CHECK(ctrl && (ctrl->flags & V4L2_CTRL_FLAG_INACTIVE));
	ctrl = kstub_ctrl_find(&hdl, NXS_CSI2_CID_TRIGGER_MODE(NXS_CSI2_CID_BASE));
	CHECK(ctrl && !(ctrl->flags & V4L2_CTRL_FLAG_INACTIVE));

	/* The device paints the row's range again: the control follows at the refresh. */
	fake_door_row(&door, IMX900_ROWS, NXS_PARAM_KIND_APPLY, 0, 0, 1000, 1);
	CHECK(nxs_csi2_core_refresh(&core) == 0);
	ctrl = kstub_ctrl_find(&hdl, NXS_CSI2_CID_ROW(NXS_CSI2_CID_BASE, IMX900_ROWS));
	CHECK(ctrl && !(ctrl->flags & V4L2_CTRL_FLAG_INACTIVE) && ctrl->maximum == 1000);
	kstub_ctrl_handler_free(&hdl);
}

static void test_controls_leave_their_rows_without_memory(void)
{
	struct v4l2_ctrl_handler hdl = { };
	struct v4l2_ctrl *ctrl;

	begin("a re-read with no memory for the names still takes every control off its row");
	serve(0x1000, 256, 0, 0, false);
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(nxs_csi2_core_init_controls(&core, &hdl, NXS_CSI2_CID_BASE) == 0);
	ctrl = kstub_ctrl_find(&hdl, NXS_CSI2_CID_TRIGGER_MODE(NXS_CSI2_CID_BASE));
	CHECK(ctrl && ctrl->priv == &core.rows[3]);

	/* Two more rows: the re-read allocates the rows again and frees the old. */
	memset(&d2, 0, sizeof(d2));
	camera_trailer(&d2, 0, 2, NULL);
	door.desc = d2.buf;
	door.desc_size = d2.len;
	door.epoch++;
	fake_door_counts(&door, IMX900_ROWS + 2, 1);
	kstub_kvcalloc_failures = 1;
	CHECK(nxs_csi2_core_reread(&core) == 0);
	CHECK(kstub_kvcalloc_failures == 0);
	CHECK(ctrl && ctrl->priv == &core.orphan && (ctrl->flags & V4L2_CTRL_FLAG_INACTIVE));
	CHECK(ctrl && ctrl->ops->g_volatile_ctrl(ctrl) == 0);
	CHECK(core.detached_count == 0);
	kstub_ctrl_handler_free(&hdl);
}

static void test_reread_takes_the_handlers_lock_itself(void)
{
	struct v4l2_ctrl_handler hdl = { };
	struct v4l2_ctrl *ctrl;
	unsigned int controls = 0;
	unsigned int takes;

	begin("the re-read of a caller that holds no lock takes the handler's lock and releases it");
	serve(0x1000, 256, 0, 1, false);
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(nxs_csi2_core_init_controls(&core, &hdl, NXS_CSI2_CID_BASE) == 0);
	for (ctrl = hdl.ctrls; ctrl; ctrl = ctrl->next)
		controls++;
	CHECK(controls > 0 && hdl.lock == &hdl._lock);
	takes = hdl._lock.takes;
	memset(&d2, 0, sizeof(d2));
	camera_trailer(&d2, 0, 1, NULL);
	serve_next(&d2, IMX900_ROWS + 1);
	CHECK(nxs_csi2_core_reread(&core) == 0);
	/* Each control leaves its row and comes back to it: one take for each. */
	CHECK(hdl._lock.takes - takes == 2 * controls);
	CHECK(hdl._lock.holder == NULL);
	kstub_ctrl_handler_free(&hdl);
}

static void test_test_pattern_menu_past_256(void)
{
	static const struct list_shape patterns = { NXS_PARAM_ID_TEST_PATTERN, 300, false };
	struct v4l2_ctrl_handler hdl = { };
	struct v4l2_ctrl *ctrl;

	begin("a test pattern list past 256 codes offers the first 256");
	serve_list(0x1000, 512, 0, 0, &patterns);
	CHECK(nxs_csi2_core_init(&core, &door.client) == 0);
	CHECK(core.param_count == IMX900_ROWS + 1);
	if (core.param_count <= IMX900_ROWS)
		return;
	CHECK(core.rows[IMX900_ROWS].count == 300);
	CHECK(nxs_csi2_core_init_controls(&core, &hdl, NXS_CSI2_CID_BASE) == 0);
	ctrl = kstub_ctrl_find(&hdl, V4L2_CID_TEST_PATTERN);
	CHECK(ctrl && ctrl->maximum == 255);
	CHECK(ctrl && !strcmp(ctrl->qmenu[255], "Pattern 2785"));
	kstub_ctrl_handler_free(&hdl);
}

/* ---- the sub-device ----------------------------------------------------- */

static void test_adapter_finds_the_line_by_its_id(void)
{
	begin("the sub-device times the line from the row of id LINE_TIME_NS, whatever its name");
	serve(0x1000, 256, 0, 0, false);
	strcpy((char *)d1.buf + ROWS_AT + 6 * NXS_ROW_SIZE, "tline");
	fake_door_row(&door, 6, NXS_PARAM_KIND_RUN, 8000, 4902, 8215, 1);
	CHECK(adapter_probe(&door.client) == 0);
	CHECK(adapter_line_row(&door.client) == 6);
	CHECK(adapter_line_ns(&door.client) == 8000);
	adapter_remove(&door.client);

	/* A line row with no reading yet times the line at its declared default. */
	serve(0x1000, 256, 0, 0, false);
	fake_door_row(&door, 6, NXS_PARAM_KIND_RUN, NXS_ROW_UNREAD, 4902, 8215, 1);
	CHECK(adapter_probe(&door.client) == 0);
	CHECK(adapter_line_row(&door.client) == 6);
	CHECK(adapter_line_ns(&door.client) == 4902);
	adapter_remove(&door.client);
	check_lock();
}

/* Row @i of @d becomes a FLIPS enum row of @n values from 0. */
static void flips_row(struct desc *d, unsigned int i, unsigned int n)
{
	u8 *row = d->buf + ROWS_AT + i * NXS_ROW_SIZE;

	memset(row, 0, NXS_ROW_SIZE);
	row_enum(row, "flips", NXS_PARAM_ID_FLIPS, NXS_PARAM_KIND_APPLY, n, 0);
}

static void test_adapter_flips_follow_a_reread(void)
{
	const unsigned int moved = IMX900_ROWS + 2;
	struct nxs_csi2_core *c;
	struct v4l2_ctrl *vflip;

	begin("both flips leave the FLIPS row at a re-read and come back to it, however the rows moved");
	serve(0x1000, 256, 0, 1, false);
	flips_row(&d1, IMX900_ROWS, 4);
	fake_door_row(&door, IMX900_ROWS, NXS_PARAM_KIND_APPLY, 0, 0, 3, 1);
	CHECK(adapter_probe(&door.client) == 0);
	vflip = adapter_flip(&door.client, true);
	CHECK(adapter_flip(&door.client, false) && vflip);
	if (!vflip)
		return;

	/* Three more rows than the core holds, the FLIPS row two further on. */
	memset(&d2, 0, sizeof(d2));
	camera_trailer(&d2, 0, 3, NULL);
	flips_row(&d2, moved, 4);
	serve_next(&d2, IMX900_ROWS + 3);
	fake_door_row(&door, IMX900_ROWS, NXS_PARAM_KIND_APPLY, 0, 0, 1000, 1);
	fake_door_row(&door, IMX900_ROWS + 1, NXS_PARAM_KIND_APPLY, 0, 0, 1000, 1);
	fake_door_row(&door, moved, NXS_PARAM_KIND_APPLY, 0, 0, 3, 1);
	CHECK(adapter_get_fmt(&door.client, NULL) == 0);
	c = adapter_core(&door.client);
	CHECK(c->param_count == IMX900_ROWS + 3 && c->rows_cap == IMX900_ROWS + 3);
	CHECK(vflip->priv == &c->rows[moved]);
	CHECK(!(vflip->flags & V4L2_CTRL_FLAG_INACTIVE));
	CHECK(adapter_set_flips(&door.client, false, true) == 0);
	CHECK(fake_door_cell(&door, 0, moved) == NXS_FLIP_Y);
	CHECK(fake_door_cell(&door, 0, IMX900_ROWS) == 0 && fake_door_cell(&door, 0, IMX900_ROWS + 1) == 0);

	/* A FLIPS row of another shape, X alone: both flips stay off it, and a set stages nothing. */
	memset(&d2, 0, sizeof(d2));
	camera_trailer(&d2, 0, 3, NULL);
	flips_row(&d2, moved, 2);
	serve_next(&d2, IMX900_ROWS + 3);
	fake_door_clear_log(&door);
	CHECK(adapter_get_fmt(&door.client, NULL) == 0);
	CHECK(vflip->priv == NULL && (vflip->flags & V4L2_CTRL_FLAG_INACTIVE));
	CHECK(adapter_flip(&door.client, false)->priv == NULL);
	fake_door_clear_log(&door);
	CHECK(adapter_set_flips(&door.client, true, false) == 0);
	CHECK(door.commit_count == 0);
	adapter_remove(&door.client);
	check_lock();
}

/*
 * A sub-device probed on a door of one custom row, then the door serves a
 * description of two behind it: custom_0 moves to row 8, and mode 0 runs
 * 1280x720. Returns custom_0's control, on row 7.
 */
static struct v4l2_ctrl *adapter_serve_moved(void)
{
	const unsigned int law = NXS_CAMERA_LAW_ROW_SIZE;
	struct v4l2_ctrl *custom;
	u8 *mode;

	serve(0x1000, 256, 0, 1, false);
	CHECK(adapter_probe(&door.client) == 0);
	custom = adapter_ctrl(&door.client, NXS_CSI2_CID_ROW(NXS_CSI2_CID_BASE, IMX900_ROWS));
	CHECK(custom && custom->priv == &adapter_core(&door.client)->rows[IMX900_ROWS]);
	memset(&d2, 0, sizeof(d2));
	camera_trailer(&d2, 0, 2, NULL);
	strcpy((char *)d2.buf + ROWS_AT + IMX900_ROWS * NXS_ROW_SIZE, "custom_9");
	strcpy((char *)d2.buf + ROWS_AT + (IMX900_ROWS + 1) * NXS_ROW_SIZE, "custom_0");
	mode = desc_find(&d2, NXS_TRAILER_RECORD_CAMERA_MAP) + NXS_CAMERA_MAP_HEAD_SIZE +
	       mode_total * law;
	put16(mode + NXS_MODE_ENTRY_WIDTH, 1280);
	put16(mode + NXS_MODE_ENTRY_HEIGHT, 720);
	serve_next(&d2, IMX900_ROWS + 2);
	fake_door_row(&door, IMX900_ROWS + 1, NXS_PARAM_KIND_APPLY, 0, 0, 1000, 1);

	return custom;
}

/*
 * The re-read returned with the lock released: the new mode's format,
 * custom_0's control on row 8 and the trigger's on row 3.
 */
static void adapter_check_moved(const struct v4l2_mbus_framefmt *fmt, struct v4l2_ctrl *custom)
{
	struct nxs_csi2_core *c = adapter_core(&door.client);
	struct v4l2_ctrl *trigger;

	CHECK(adapter_lock(&door.client)->holder == NULL);
	CHECK(c->read && c->param_count == IMX900_ROWS + 2 && c->detached_count == 0);
	CHECK(fmt->width == 1280 && fmt->height == 720);
	CHECK(fmt->code == MEDIA_BUS_FMT_SGBRG10_1X10);
	CHECK(custom && custom->priv == &c->rows[IMX900_ROWS + 1]);
	CHECK(c->rows[IMX900_ROWS + 1].ctrl == custom);
	CHECK(custom && !(custom->flags & V4L2_CTRL_FLAG_INACTIVE));
	CHECK(c->rows[IMX900_ROWS].ctrl == NULL);
	trigger = adapter_ctrl(&door.client, NXS_CSI2_CID_TRIGGER_MODE(NXS_CSI2_CID_BASE));
	CHECK(trigger && trigger->priv == &c->rows[3]);
	CHECK(trigger && !(trigger->flags & V4L2_CTRL_FLAG_INACTIVE));
}

static void test_adapter_get_fmt_rereads_under_the_held_lock(void)
{
	struct v4l2_mbus_framefmt fmt = { };
	struct v4l2_ctrl *custom;

	begin("a format read on a changed door re-reads it under the state lock the pad op holds");
	custom = adapter_serve_moved();
	CHECK(adapter_get_fmt(&door.client, &fmt) == 0);
	adapter_check_moved(&fmt, custom);
	adapter_remove(&door.client);
	check_lock();
}

static void test_adapter_set_fmt_rereads_under_the_held_lock(void)
{
	struct v4l2_mbus_framefmt fmt = { .width = 1032, .height = 776 };
	struct v4l2_ctrl *custom;

	begin("a format set on a changed door re-reads it under the state lock the pad op holds");
	custom = adapter_serve_moved();
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	adapter_check_moved(&fmt, custom);
	adapter_remove(&door.client);
	check_lock();
}

static void test_adapter_stream_on_rereads_under_its_lock(void)
{
	struct v4l2_mbus_framefmt fmt = { };
	struct v4l2_ctrl *custom;

	begin("a stream start on a changed door re-reads it under the lock the stream op takes");
	custom = adapter_serve_moved();
	CHECK(adapter_s_stream(&door.client, true) == 0);
	CHECK(adapter_streaming(&door.client));
	CHECK(fake_door_cell(&door, 0, 4) == 1);
	CHECK(adapter_get_fmt(&door.client, &fmt) == 0);
	adapter_check_moved(&fmt, custom);
	CHECK(adapter_s_stream(&door.client, false) == 0);
	CHECK(!adapter_streaming(&door.client) && fake_door_cell(&door, 0, 4) == 0);
	adapter_remove(&door.client);
	check_lock();
}

static void test_adapter_reads_a_door_emptied_mid_upload_under_the_held_lock(void)
{
	struct v4l2_mbus_framefmt fmt = { };
	struct v4l2_ctrl *custom;
	u32 size;

	begin("a door read again mid-upload, then served its description, is read under the state lock");
	custom = adapter_serve_moved();
	size = door.desc_size;
	door.desc_size = 0;
	CHECK(adapter_get_fmt(&door.client, &fmt) == 0);
	CHECK(!adapter_core(&door.client)->read && adapter_core(&door.client)->detached_count > 0);
	CHECK(custom && (custom->flags & V4L2_CTRL_FLAG_INACTIVE));
	door.desc_size = size;
	door.epoch++;
	CHECK(adapter_get_fmt(&door.client, &fmt) == 0);
	adapter_check_moved(&fmt, custom);
	adapter_remove(&door.client);
	check_lock();
}

/* ---- the passive node ---------------------------------------------------- */

/* The door's first and third modes, 1032x776 RAW10 and 2064x1552 RAW12. */
static const u32 passive_modes[] = {
	1032, 776, 10, 2, 2376, 4,
	2064, 1552, 12, 0, 2376, 4,
};

/* Mode 0's rows as serve() paints the door's cells and its line, then mode 1's at a longer line. */
static const u32 passive_rows[] = {
	0, NXS_PARAM_ID_EXPOSURE_US, 4, 19000, 1, 4216,
	0, NXS_PARAM_ID_FRAME_RATE_MFPS, 12, 215843, 1, 215843,
	0, NXS_PARAM_ID_GAIN_MDB, 0, 48000, 100, 0,
	0, NXS_PARAM_ID_LINE_TIME_NS, 4902, 4902, 0, 4902,
	1, NXS_PARAM_ID_EXPOSURE_US, 8, 33000, 1, 8000,
	1, NXS_PARAM_ID_FRAME_RATE_MFPS, 12, 72067, 1, 72067,
	1, NXS_PARAM_ID_GAIN_MDB, 0, 30000, 100, 0,
	1, NXS_PARAM_ID_LINE_TIME_NS, 8215, 8215, 0, 8215,
};

static const u32 range_cids[] = { V4L2_CID_EXPOSURE, V4L2_CID_VBLANK, V4L2_CID_ANALOGUE_GAIN };

/* A passive node of @modes on the door's client, with @rows when @nr_rows. */
static void serve_passive_tree(const u32 *modes, size_t nr_modes, const u32 *rows, size_t nr_rows)
{
	serve(0x1000, 256, 0, 0, false);
	kstub_properties_clear();
	kstub_property_set("aliensense,passive", NULL, 0);
	kstub_property_set("aliensense,modes", modes, nr_modes);
	if (nr_rows)
		kstub_property_set("aliensense,rows", rows, nr_rows);
	fake_door_clear_log(&door);
}

/* A passive node of passive_modes on the door's client, with passive_rows when @rows. */
static void serve_passive(bool rows)
{
	serve_passive_tree(passive_modes, ARRAY_SIZE(passive_modes), passive_rows,
			   rows ? ARRAY_SIZE(passive_rows) : 0);
}

static void test_passive_rows_take_a_doors_controls(void)
{
	struct v4l2_ctrl at_door[ARRAY_SIZE(range_cids)];
	struct v4l2_ctrl *ctrl;
	unsigned int i;

	begin("a passive node's tree rows take EXPOSURE, VBLANK and ANALOGUE_GAIN in lines as a door's rows do");
	serve(0x1000, 256, 0, 0, false);
	CHECK(adapter_probe(&door.client) == 0);
	for (i = 0; i < ARRAY_SIZE(range_cids); i++) {
		ctrl = adapter_ctrl(&door.client, range_cids[i]);
		CHECK(ctrl);
		if (ctrl)
			at_door[i] = *ctrl;
	}
	adapter_remove(&door.client);

	serve_passive(true);
	CHECK(adapter_probe(&door.client) == 0);
	CHECK(adapter_line_ns(&door.client) == 4902);
	for (i = 0; i < ARRAY_SIZE(range_cids); i++) {
		ctrl = adapter_ctrl(&door.client, range_cids[i]);
		CHECK(ctrl);
		if (!ctrl)
			continue;
		CHECK(ctrl->minimum == at_door[i].minimum && ctrl->maximum == at_door[i].maximum);
		CHECK(ctrl->step == at_door[i].step && ctrl->default_value == at_door[i].default_value);
		CHECK(!(ctrl->flags & V4L2_CTRL_FLAG_INACTIVE));
	}
	/* 4 us to 19000 us in lines of 4902 ns, 4216 us the nearest 860; 215843 mfps down to 12 less 776 lines. */
	ctrl = adapter_ctrl(&door.client, V4L2_CID_EXPOSURE);
	CHECK(ctrl && ctrl->minimum == 1 && ctrl->maximum == 3875 && ctrl->default_value == 860);
	ctrl = adapter_ctrl(&door.client, V4L2_CID_VBLANK);
	CHECK(ctrl && ctrl->minimum == 170 && ctrl->maximum == 16999088 && ctrl->default_value == 170);
	ctrl = adapter_ctrl(&door.client, V4L2_CID_ANALOGUE_GAIN);
	CHECK(ctrl && ctrl->minimum == 0 && ctrl->maximum == 48000 && ctrl->step == 100);
	/* One row per id across both modes; the line row, the fourth, is the core's read-only control. */
	CHECK(adapter_core(&door.client)->param_count == 4 && adapter_line_row(&door.client) == 3);
	ctrl = adapter_ctrl(&door.client, NXS_CSI2_CID_ROW(NXS_CSI2_CID_BASE, 3));
	CHECK(ctrl && (ctrl->flags & V4L2_CTRL_FLAG_READ_ONLY) && ctrl->cur.val == 4902);
	CHECK(door.log_count == 0);
	adapter_remove(&door.client);
	kstub_properties_clear();
	check_lock();
}

static void test_passive_set_transfers_nothing_and_raises_one_event(void)
{
	static const s32 values[] = { 900, 4000, 1200 };
	unsigned int i;

	begin("a set on a passive node transfers nothing and raises one control event per change");
	serve_passive(true);
	CHECK(adapter_probe(&door.client) == 0);
	for (i = 0; i < ARRAY_SIZE(range_cids); i++)
		CHECK(adapter_subscribe(&door.client, range_cids[i], true) == 0);
	for (i = 0; i < ARRAY_SIZE(range_cids); i++) {
		struct v4l2_ctrl *ctrl = adapter_ctrl(&door.client, range_cids[i]);

		CHECK(ctrl);
		if (!ctrl)
			continue;
		kstub_events_clear();
		CHECK(adapter_set_ctrl(&door.client, range_cids[i], values[i]) == 0);
		CHECK(ctrl->cur.val == values[i]);
		CHECK(kstub_event_count == 1);
		CHECK(kstub_events[0].id == range_cids[i] && kstub_events[0].value == values[i]);
		CHECK(kstub_events[0].changes == V4L2_EVENT_CTRL_CH_VALUE);
		/* The same value again changes nothing: no event. */
		CHECK(adapter_set_ctrl(&door.client, range_cids[i], values[i]) == 0);
		CHECK(kstub_event_count == 1);
	}
	/* A client that unsubscribed hears no more. */
	CHECK(adapter_subscribe(&door.client, V4L2_CID_EXPOSURE, false) == 0);
	kstub_events_clear();
	CHECK(adapter_set_ctrl(&door.client, V4L2_CID_EXPOSURE, 901) == 0);
	CHECK(kstub_event_count == 0);
	CHECK(door.log_count == 0 && door.commit_count == 0);
	adapter_remove(&door.client);
	kstub_properties_clear();
	check_lock();
}

static void test_passive_without_rows_offers_the_line_and_the_bus(void)
{
	begin("a passive node without aliensense,rows offers the line's and the bus's controls alone");
	serve_passive(false);
	CHECK(adapter_probe(&door.client) == 0);
	CHECK(adapter_ctrl_count(&door.client) == 3);
	CHECK(adapter_ctrl(&door.client, V4L2_CID_HBLANK) && adapter_ctrl(&door.client, V4L2_CID_PIXEL_RATE) &&
	      adapter_ctrl(&door.client, V4L2_CID_LINK_FREQ));
	CHECK(!adapter_ctrl(&door.client, V4L2_CID_EXPOSURE) && !adapter_ctrl(&door.client, V4L2_CID_VBLANK) &&
	      !adapter_ctrl(&door.client, V4L2_CID_ANALOGUE_GAIN));
	CHECK(adapter_subscribe(&door.client, V4L2_CID_HBLANK, true) == 0);
	CHECK(door.log_count == 0);
	adapter_remove(&door.client);
	kstub_properties_clear();
	check_lock();
}

static void test_door_controls_commit_and_raise_their_events(void)
{
	begin("a door node's controls commit a set and raise its event, its list as the rows give it");
	serve(0x1000, 256, 0, 0, false);
	CHECK(adapter_probe(&door.client) == 0);
	/* EXPOSURE, VBLANK, ANALOGUE_GAIN, the trigger, the line row, HBLANK, PIXEL_RATE, LINK_FREQ. */
	CHECK(adapter_ctrl_count(&door.client) == 8);
	CHECK(adapter_ctrl(&door.client, NXS_CSI2_CID_TRIGGER_MODE(NXS_CSI2_CID_BASE)));
	CHECK(adapter_ctrl(&door.client, NXS_CSI2_CID_ROW(NXS_CSI2_CID_BASE, 6)));
	CHECK(adapter_subscribe(&door.client, V4L2_CID_EXPOSURE, true) == 0);
	fake_door_clear_log(&door);
	kstub_events_clear();
	/* 900 lines of 4902 ns, 4412 us. */
	CHECK(adapter_set_ctrl(&door.client, V4L2_CID_EXPOSURE, 900) == 0);
	CHECK(door.commit_count == 1 && fake_door_cell(&door, 0, 0) == 4412);
	CHECK(kstub_event_count == 1 && kstub_events[0].id == V4L2_CID_EXPOSURE &&
	      kstub_events[0].value == 900);
	adapter_remove(&door.client);
	check_lock();
}

/*
 * passive_modes the other way round, the door's third mode first, with
 * their rows as the overlay writes them: the exposure, the gain and the
 * line of each mode, no rate.
 */
static const u32 select_modes[] = {
	2064, 1552, 12, 0, 2376, 4,
	1032, 776, 10, 2, 2376, 4,
};

static const u32 select_rows[] = {
	0, NXS_PARAM_ID_EXPOSURE_US, 8, 33000, 1, 8000,
	0, NXS_PARAM_ID_GAIN_MDB, 0, 30000, 100, 0,
	0, NXS_PARAM_ID_LINE_TIME_NS, 8215, 8215, 0, 8215,
	1, NXS_PARAM_ID_EXPOSURE_US, 4, 19000, 1, 4216,
	1, NXS_PARAM_ID_GAIN_MDB, 0, 48000, 100, 0,
	1, NXS_PARAM_ID_LINE_TIME_NS, 4902, 4902, 0, 4902,
};

/* The media bus code of a tree mode of @depth bits and Bayer order @phase, unflipped. */
static u32 tree_code(u8 depth, u8 phase)
{
	struct nxs_csi2_mode mode = {
		.bit_depth = depth,
		.data_type = depth == 10 ? NXS_CSI2_DATA_TYPE_RAW10 : NXS_CSI2_DATA_TYPE_RAW12,
		.phase = phase,
	};

	return nxs_csi2_core_mbus_code(&mode, 0);
}

/* A format of @width x @height on @code. */
static struct v4l2_mbus_framefmt tree_fmt(u32 width, u32 height, u32 code)
{
	struct v4l2_mbus_framefmt fmt = { .width = width, .height = height, .code = code };

	return fmt;
}

static void test_passive_format_set_selects_its_mode_and_its_rows(void)
{
	const u32 raw12 = tree_code(12, 0);
	const u32 raw10 = tree_code(10, 2);
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *gain;

	begin("a passive node's format set selects its mode and that mode's rows");
	serve_passive_tree(select_modes, ARRAY_SIZE(select_modes), select_rows,
			   ARRAY_SIZE(select_rows));
	CHECK(adapter_probe(&door.client) == 0);
	exposure = adapter_ctrl(&door.client, V4L2_CID_EXPOSURE);
	gain = adapter_ctrl(&door.client, V4L2_CID_ANALOGUE_GAIN);
	CHECK(exposure && gain);
	if (!exposure || !gain)
		goto out;
	CHECK(adapter_get_fmt(&door.client, &fmt) == 0);
	CHECK(fmt.width == 2064 && fmt.height == 1552 && fmt.code == raw12);
	/* 8 us to 33000 us in lines of 8215 ns. */
	CHECK(adapter_line_ns(&door.client) == 8215);
	CHECK(exposure->minimum == 1 && exposure->maximum == 4017 && gain->maximum == 30000);

	/* The second mode's size and code: the second mode and its rows. */
	fmt = tree_fmt(1032, 776, raw10);
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	CHECK(fmt.width == 1032 && fmt.height == 776 && fmt.code == raw10);
	CHECK(adapter_get_fmt(&door.client, &fmt) == 0);
	CHECK(fmt.width == 1032 && fmt.height == 776 && fmt.code == raw10);
	/* 4 us to 19000 us in lines of 4902 ns; 4902 ns at 950.4 Mpixel/s is 4659 pixels. */
	CHECK(adapter_line_ns(&door.client) == 4902);
	CHECK(exposure->minimum == 1 && exposure->maximum == 3875 && gain->maximum == 48000);
	CHECK(adapter_ctrl(&door.client, V4L2_CID_HBLANK)->minimum == 4659 - 1032);
	CHECK(adapter_ctrl(&door.client, V4L2_CID_PIXEL_RATE)->val64 == 950400000);

	/* A size no mode has, and a mode's size on another code: the mode stays. */
	fmt = tree_fmt(1920, 1080, raw10);
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	CHECK(fmt.width == 1032 && fmt.height == 776 && fmt.code == raw10);
	fmt = tree_fmt(2064, 1552, raw10);
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	CHECK(fmt.width == 1032 && adapter_line_ns(&door.client) == 4902);

	/* A try reports the mode in effect and changes nothing. */
	fmt = tree_fmt(2064, 1552, raw12);
	CHECK(adapter_try_fmt(&door.client, &fmt) == 0);
	CHECK(fmt.width == 1032 && fmt.code == raw10 && adapter_line_ns(&door.client) == 4902);

	/* Under a stream another mode is refused; the mode in effect is answered. */
	CHECK(adapter_s_stream(&door.client, true) == 0);
	fmt = tree_fmt(2064, 1552, raw12);
	CHECK(adapter_set_fmt(&door.client, &fmt) == -EBUSY);
	CHECK(adapter_get_fmt(&door.client, &fmt) == 0);
	CHECK(fmt.width == 1032 && adapter_line_ns(&door.client) == 4902);
	fmt = tree_fmt(1032, 776, raw10);
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	CHECK(fmt.width == 1032);
	CHECK(adapter_s_stream(&door.client, false) == 0);

	/* The first mode again, and its rows. */
	fmt = tree_fmt(2064, 1552, raw12);
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	CHECK(fmt.width == 2064 && adapter_line_ns(&door.client) == 8215);
	CHECK(exposure->minimum == 1 && exposure->maximum == 4017 && gain->maximum == 30000);
	CHECK(adapter_ctrl(&door.client, V4L2_CID_PIXEL_RATE)->val64 == 792000000);
	CHECK(door.log_count == 0 && door.commit_count == 0);
out:
	adapter_remove(&door.client);
	kstub_properties_clear();
	check_lock();
}

/*
 * select_modes with the black level on both modes, at another range each,
 * and the gain on the first mode alone.
 */
static const u32 follow_rows[] = {
	0, NXS_PARAM_ID_EXPOSURE_US, 8, 33000, 1, 8000,
	0, NXS_PARAM_ID_GAIN_MDB, 0, 30000, 100, 0,
	0, NXS_PARAM_ID_BLACK_LEVEL_DN, 0, 4095, 1, 240,
	0, NXS_PARAM_ID_LINE_TIME_NS, 8215, 8215, 0, 8215,
	1, NXS_PARAM_ID_EXPOSURE_US, 4, 19000, 1, 4216,
	1, NXS_PARAM_ID_BLACK_LEVEL_DN, 0, 1023, 1, 64,
	1, NXS_PARAM_ID_LINE_TIME_NS, 4902, 4902, 0, 4902,
};

/* Whether a subscriber heard an event of @id whose changes hold @changes. */
static bool event_heard(u32 id, u32 changes)
{
	unsigned int i;

	for (i = 0; i < kstub_event_count; i++)
		if (kstub_events[i].id == id && (kstub_events[i].changes & changes) == changes)
			return true;

	return false;
}

/* What the line row's control reads, as user space reads it; -1 for no control. */
static s32 line_row_reads(void)
{
	s32 val = -1;

	if (adapter_get_ctrl(&door.client,
			     NXS_CSI2_CID_ROW(NXS_CSI2_CID_BASE, adapter_line_row(&door.client)),
			     &val))
		return -1;

	return val;
}

static void test_passive_format_set_repaints_the_core_controls(void)
{
	const u32 black_cid = NXS_CSI2_CID_BLACK_LEVEL(NXS_CSI2_CID_BASE);
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_ctrl *black;

	begin("a passive node's format set repaints the core's controls: the line row and the black level");
	serve_passive_tree(select_modes, ARRAY_SIZE(select_modes), follow_rows,
			   ARRAY_SIZE(follow_rows));
	CHECK(adapter_probe(&door.client) == 0);
	black = adapter_ctrl(&door.client, black_cid);
	CHECK(black);
	if (!black)
		goto out;
	CHECK(line_row_reads() == 8215);
	CHECK(black->minimum == 0 && black->maximum == 4095 && black->default_value == 240);

	/* The second mode: its line and its black level's range. */
	fmt = tree_fmt(1032, 776, tree_code(10, 2));
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	CHECK(adapter_line_ns(&door.client) == 4902);
	CHECK(line_row_reads() == 4902);
	CHECK(black->minimum == 0 && black->maximum == 1023 && black->default_value == 64);

	/* The first again. */
	fmt = tree_fmt(2064, 1552, tree_code(12, 0));
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	CHECK(line_row_reads() == 8215);
	CHECK(black->maximum == 4095 && black->default_value == 240);
	CHECK(door.log_count == 0 && door.commit_count == 0);
out:
	adapter_remove(&door.client);
	kstub_properties_clear();
	check_lock();
}

static void test_passive_format_set_activates_the_modes_rows(void)
{
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_ctrl *gain;

	begin("a passive node's format set activates the rows the mode lists, each with its flags event");
	serve_passive_tree(select_modes, ARRAY_SIZE(select_modes), follow_rows,
			   ARRAY_SIZE(follow_rows));
	CHECK(adapter_probe(&door.client) == 0);
	gain = adapter_ctrl(&door.client, V4L2_CID_ANALOGUE_GAIN);
	CHECK(gain);
	if (!gain)
		goto out;
	CHECK(!(gain->flags & V4L2_CTRL_FLAG_INACTIVE));
	CHECK(adapter_subscribe(&door.client, V4L2_CID_ANALOGUE_GAIN, true) == 0);

	/* The second mode lists no gain: the gain reads inactive. */
	kstub_events_clear();
	fmt = tree_fmt(1032, 776, tree_code(10, 2));
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	CHECK(gain->flags & V4L2_CTRL_FLAG_INACTIVE);
	CHECK(event_heard(V4L2_CID_ANALOGUE_GAIN, V4L2_EVENT_CTRL_CH_FLAGS));

	/* The first mode lists it: active again. */
	kstub_events_clear();
	fmt = tree_fmt(2064, 1552, tree_code(12, 0));
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	CHECK(!(gain->flags & V4L2_CTRL_FLAG_INACTIVE));
	CHECK(event_heard(V4L2_CID_ANALOGUE_GAIN, V4L2_EVENT_CTRL_CH_FLAGS));
	CHECK(door.log_count == 0 && door.commit_count == 0);
out:
	adapter_remove(&door.client);
	kstub_properties_clear();
	check_lock();
}

/* select_modes and a second mode of the first's size and code after them, at its own line. */
static const u32 twin_modes[] = {
	2064, 1552, 12, 0, 2376, 4,
	1032, 776, 10, 2, 2376, 4,
	1032, 776, 10, 2, 2376, 4,
};

static const u32 twin_rows[] = {
	0, NXS_PARAM_ID_EXPOSURE_US, 8, 33000, 1, 8000,
	0, NXS_PARAM_ID_LINE_TIME_NS, 8215, 8215, 0, 8215,
	1, NXS_PARAM_ID_EXPOSURE_US, 4, 19000, 1, 4216,
	1, NXS_PARAM_ID_LINE_TIME_NS, 4902, 4902, 0, 4902,
	2, NXS_PARAM_ID_EXPOSURE_US, 6, 25000, 1, 5000,
	2, NXS_PARAM_ID_LINE_TIME_NS, 6000, 6000, 0, 6000,
};

static void test_passive_format_set_takes_the_first_of_two_modes_of_one_format(void)
{
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_ctrl *exposure;

	begin("two modes of one size and code are one format, and a set takes the first's rows");
	serve_passive_tree(twin_modes, ARRAY_SIZE(twin_modes), twin_rows, ARRAY_SIZE(twin_rows));
	CHECK(adapter_probe(&door.client) == 0);
	exposure = adapter_ctrl(&door.client, V4L2_CID_EXPOSURE);
	CHECK(exposure);
	if (!exposure)
		goto out;
	fmt = tree_fmt(1032, 776, tree_code(10, 2));
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	CHECK(fmt.width == 1032 && fmt.height == 776);
	/* The first twin's line and exposure: 4 us to 19000 us in lines of 4902 ns. */
	CHECK(adapter_line_ns(&door.client) == 4902);
	CHECK(exposure->minimum == 1 && exposure->maximum == 3875);
out:
	adapter_remove(&door.client);
	kstub_properties_clear();
	check_lock();
}

/* The line row's control, NULL for none. */
static struct v4l2_ctrl *line_row_ctrl(void)
{
	int row = adapter_line_row(&door.client);

	return row >= 0 ? adapter_ctrl(&door.client, NXS_CSI2_CID_ROW(NXS_CSI2_CID_BASE, row)) : NULL;
}

/* Whether the line row's control reads @line, its minimum, maximum and default @line too. */
static bool line_row_is(s32 line)
{
	const struct v4l2_ctrl *ctrl = line_row_ctrl();

	return ctrl && line_row_reads() == line && ctrl->minimum == line && ctrl->maximum == line &&
	       ctrl->default_value == line;
}

/* passive_modes with a tree line under the first mode's shortest, and the second's line. */
static const u32 short_line_rows[] = {
	0, NXS_PARAM_ID_EXPOSURE_US, 4, 19000, 1, 4216,
	0, NXS_PARAM_ID_LINE_TIME_NS, 500, 500, 0, 500,
	1, NXS_PARAM_ID_EXPOSURE_US, 8, 33000, 1, 8000,
	1, NXS_PARAM_ID_LINE_TIME_NS, 8215, 8215, 0, 8215,
};

static void test_passive_line_row_reads_the_line_in_effect_on_that_one_value(void)
{
	struct v4l2_mbus_framefmt fmt;

	begin("a passive node's line row reads the line in effect, its range that one value");
	serve_passive_tree(passive_modes, ARRAY_SIZE(passive_modes), short_line_rows,
			   ARRAY_SIZE(short_line_rows));
	CHECK(adapter_probe(&door.client) == 0);
	/* A tree line of 500 ns under 1032 pixels at 950.4 Mpixel/s, 1086 ns rounded up. */
	CHECK(adapter_line_ns(&door.client) == 1086);
	CHECK(line_row_is(1086));
	fmt = tree_fmt(2064, 1552, tree_code(12, 0));
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	CHECK(adapter_line_ns(&door.client) == 8215 && line_row_is(8215));
	fmt = tree_fmt(1032, 776, tree_code(10, 2));
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	CHECK(adapter_line_ns(&door.client) == 1086 && line_row_is(1086));
	CHECK(door.log_count == 0);
	adapter_remove(&door.client);
	kstub_properties_clear();
	check_lock();
}

/* select_modes with the line on the first mode alone. */
static const u32 one_line_rows[] = {
	0, NXS_PARAM_ID_EXPOSURE_US, 8, 33000, 1, 8000,
	0, NXS_PARAM_ID_LINE_TIME_NS, 8215, 8215, 0, 8215,
	1, NXS_PARAM_ID_EXPOSURE_US, 4, 19000, 1, 4216,
};

static void test_passive_line_row_is_active_where_the_mode_lists_it(void)
{
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_ctrl *row;

	begin("a passive node's line row is active where the mode lists it and reads the line in effect");
	serve_passive_tree(select_modes, ARRAY_SIZE(select_modes), one_line_rows,
			   ARRAY_SIZE(one_line_rows));
	CHECK(adapter_probe(&door.client) == 0);
	row = line_row_ctrl();
	CHECK(row);
	if (!row)
		goto out;
	CHECK(!(row->flags & V4L2_CTRL_FLAG_INACTIVE) && line_row_is(8215));

	/* The second mode lists no line: inactive, reading the mode's shortest, 1086 ns. */
	fmt = tree_fmt(1032, 776, tree_code(10, 2));
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	CHECK(adapter_line_ns(&door.client) == 1086);
	CHECK((row->flags & V4L2_CTRL_FLAG_INACTIVE) && line_row_is(1086));

	/* The first again: active on its line. */
	fmt = tree_fmt(2064, 1552, tree_code(12, 0));
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	CHECK(!(row->flags & V4L2_CTRL_FLAG_INACTIVE) && line_row_is(8215));
	CHECK(door.log_count == 0);
out:
	adapter_remove(&door.client);
	kstub_properties_clear();
	check_lock();
}

/* A passive node's owner controls, in their cluster's order. */
static const u32 owner_cids[] = {
	NXS_CSI2_CID_OWNER_LINE_NS(NXS_CSI2_CID_BASE),
	NXS_CSI2_CID_OWNER_EXPOSURE_MIN_US(NXS_CSI2_CID_BASE),
	NXS_CSI2_CID_OWNER_EXPOSURE_MAX_US(NXS_CSI2_CID_BASE),
};

/* The paint the product's scripted node takes as well: a line of 14815 ns, 30 us to 33100 us. */
static const s32 owner_paint[] = { 14815, 30, 33100 };

/* Whether the owner controls hold @line, @min and @max. */
static bool owner_reads(s32 line, s32 min, s32 max)
{
	const s32 want[] = { line, min, max };
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(owner_cids); i++) {
		const struct v4l2_ctrl *ctrl = adapter_ctrl(&door.client, owner_cids[i]);

		if (!ctrl || ctrl->cur.val != want[i])
			return false;
	}

	return true;
}

static void test_passive_owner_paints_the_line_and_the_exposure_bounds(void)
{
	const s32 above[] = { 14815, 40000, 35000 };
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *hblank;
	unsigned int i;

	begin("a passive node's owner paints the line and the exposure's bounds");
	serve_passive(true);
	CHECK(adapter_probe(&door.client) == 0);
	CHECK(adapter_ctrl(&door.client, NXS_CSI2_CID_OWNER_LINE_NS(NXS_CSI2_CID_BASE)));
	exposure = adapter_ctrl(&door.client, V4L2_CID_EXPOSURE);
	hblank = adapter_ctrl(&door.client, V4L2_CID_HBLANK);
	CHECK(exposure && hblank);
	if (!adapter_ctrl(&door.client, owner_cids[0]) || !exposure || !hblank)
		goto out;
	CHECK(adapter_subscribe(&door.client, V4L2_CID_EXPOSURE, true) == 0);
	CHECK(adapter_subscribe(&door.client, V4L2_CID_ANALOGUE_GAIN, true) == 0);
	kstub_events_clear();
	CHECK(adapter_set_ctrls(&door.client, owner_cids, owner_paint, 3) == 0);
	CHECK(owner_reads(14815, 30, 33100));
	/* 30 us rounds up to 3 lines of 14815 ns and 33100 us down to 2234. */
	CHECK(adapter_line_ns(&door.client) == 14815);
	CHECK(exposure->minimum == 3 && exposure->maximum == 2234);
	/* 14815 ns at 950.4 Mpixel/s is 14080 pixels, 1032 of them active. */
	CHECK(hblank->minimum == 14080 - 1032 && hblank->maximum == hblank->minimum &&
	      hblank->cur.val == hblank->minimum);
	/* The exposure's 860 lines stand inside: one range event, the gain's range unmoved. */
	CHECK(exposure->cur.val == 860);
	CHECK(kstub_event_count == 1 && kstub_events[0].id == V4L2_CID_EXPOSURE &&
	      kstub_events[0].changes == V4L2_EVENT_CTRL_CH_RANGE);
	/* The same paint again moves nothing. */
	kstub_events_clear();
	CHECK(adapter_set_ctrls(&door.client, owner_cids, owner_paint, 3) == 0);
	CHECK(kstub_event_count == 0);
	/* A minimum above the maximum is refused, alone or with the maximum, and the paint stands. */
	CHECK(adapter_set_ctrl(&door.client, owner_cids[1], 33101) == -EINVAL);
	CHECK(adapter_set_ctrls(&door.client, owner_cids, above, 3) == -EINVAL);
	CHECK(owner_reads(14815, 30, 33100));
	CHECK(exposure->minimum == 3 && exposure->maximum == 2234 && kstub_event_count == 0);
	CHECK(door.log_count == 0 && door.commit_count == 0);
out:
	adapter_remove(&door.client);

	/* A door offers none of the three. */
	kstub_properties_clear();
	serve(0x1000, 256, 0, 0, false);
	CHECK(adapter_probe(&door.client) == 0);
	for (i = 0; i < ARRAY_SIZE(owner_cids); i++)
		CHECK(!adapter_ctrl(&door.client, owner_cids[i]));
	adapter_remove(&door.client);
	check_lock();
}

static void test_passive_owner_controls_read_the_tree_before_a_paint(void)
{
	unsigned int i;

	begin("a passive node's owner controls read the tree's line and exposure bounds before a paint");
	serve_passive(true);
	CHECK(adapter_probe(&door.client) == 0);
	CHECK(owner_reads(4902, 4, 19000));
	for (i = 0; i < ARRAY_SIZE(owner_cids); i++) {
		const struct v4l2_ctrl *ctrl = adapter_ctrl(&door.client, owner_cids[i]);

		CHECK(ctrl && ctrl->type == V4L2_CTRL_TYPE_INTEGER &&
		      ctrl->maximum == (i == 0 ? NSEC_PER_SEC : S32_MAX));
		CHECK(ctrl && !(ctrl->flags & (V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE)));
	}
	adapter_remove(&door.client);
	kstub_properties_clear();
	check_lock();
}

static void test_passive_paint_clamps_the_exposure_into_its_range(void)
{
	const s32 high[] = { 14815, 30000, 33100 };
	struct v4l2_ctrl *exposure;

	begin("a paint clamps an exposure outside its new range and raises its value and range events");
	serve_passive(true);
	CHECK(adapter_probe(&door.client) == 0);
	exposure = adapter_ctrl(&door.client, V4L2_CID_EXPOSURE);
	CHECK(exposure && adapter_ctrl(&door.client, owner_cids[0]));
	if (!exposure || !adapter_ctrl(&door.client, owner_cids[0]))
		goto out;
	CHECK(adapter_subscribe(&door.client, V4L2_CID_EXPOSURE, true) == 0);
	CHECK(adapter_subscribe(&door.client, V4L2_CID_ANALOGUE_GAIN, true) == 0);
	CHECK(adapter_set_ctrl(&door.client, V4L2_CID_EXPOSURE, 3000) == 0);
	kstub_events_clear();
	/* 3000 lines past the painted 2234: the exposure's maximum, its cluster told of a range. */
	CHECK(adapter_set_ctrls(&door.client, owner_cids, owner_paint, 3) == 0);
	CHECK(exposure->cur.val == 2234);
	CHECK(kstub_event_count == 2);
	CHECK(kstub_events[0].id == V4L2_CID_EXPOSURE && kstub_events[0].value == 2234 &&
	      kstub_events[0].changes == (V4L2_EVENT_CTRL_CH_VALUE | V4L2_EVENT_CTRL_CH_RANGE));
	CHECK(kstub_events[1].id == V4L2_CID_ANALOGUE_GAIN &&
	      kstub_events[1].changes == V4L2_EVENT_CTRL_CH_RANGE);
	/* 100 lines under a painted 30000 us, 2025 lines: the exposure's minimum. */
	CHECK(adapter_set_ctrl(&door.client, V4L2_CID_EXPOSURE, 100) == 0);
	kstub_events_clear();
	CHECK(adapter_set_ctrls(&door.client, owner_cids, high, 3) == 0);
	CHECK(exposure->minimum == 2025 && exposure->cur.val == 2025);
	CHECK(kstub_event_count == 2 && kstub_events[0].value == 2025 &&
	      kstub_events[0].changes == (V4L2_EVENT_CTRL_CH_VALUE | V4L2_EVENT_CTRL_CH_RANGE));
	CHECK(door.log_count == 0 && door.commit_count == 0);
out:
	adapter_remove(&door.client);
	kstub_properties_clear();
	check_lock();
}

static void test_passive_owner_paints_while_the_node_streams(void)
{
	struct v4l2_ctrl *exposure;

	begin("a passive node's owner paints while the node streams");
	serve_passive(true);
	CHECK(adapter_probe(&door.client) == 0);
	exposure = adapter_ctrl(&door.client, V4L2_CID_EXPOSURE);
	CHECK(exposure);
	CHECK(adapter_s_stream(&door.client, true) == 0 && adapter_streaming(&door.client));
	CHECK(adapter_set_ctrls(&door.client, owner_cids, owner_paint, 3) == 0);
	CHECK(owner_reads(14815, 30, 33100) && adapter_line_ns(&door.client) == 14815);
	CHECK(exposure && exposure->minimum == 3 && exposure->maximum == 2234);
	CHECK(adapter_set_ctrl(&door.client, V4L2_CID_EXPOSURE, 2000) == 0);
	CHECK(adapter_s_stream(&door.client, false) == 0);
	CHECK(door.log_count == 0 && door.commit_count == 0);
	adapter_remove(&door.client);
	kstub_properties_clear();
	check_lock();
}

static void test_passive_format_set_returns_the_modes_tree_rows(void)
{
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_ctrl *exposure;

	begin("a format set of another mode returns that mode's tree rows over a paint, with a range event");
	serve_passive(true);
	CHECK(adapter_probe(&door.client) == 0);
	exposure = adapter_ctrl(&door.client, V4L2_CID_EXPOSURE);
	CHECK(exposure && adapter_ctrl(&door.client, owner_cids[0]));
	if (!exposure || !adapter_ctrl(&door.client, owner_cids[0]))
		goto out;
	CHECK(adapter_subscribe(&door.client, V4L2_CID_EXPOSURE, true) == 0);
	CHECK(adapter_set_ctrls(&door.client, owner_cids, owner_paint, 3) == 0);
	/* The mode in effect again: the paint stands. */
	kstub_events_clear();
	fmt = tree_fmt(1032, 776, tree_code(10, 2));
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	CHECK(owner_reads(14815, 30, 33100) && adapter_line_ns(&door.client) == 14815);
	CHECK(exposure->minimum == 3 && exposure->maximum == 2234 && kstub_event_count == 0);
	/* Mode 1: its tree's line and bounds, 8 us to 33000 us in lines of 8215 ns. */
	fmt = tree_fmt(2064, 1552, tree_code(12, 0));
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	CHECK(owner_reads(8215, 8, 33000) && adapter_line_ns(&door.client) == 8215);
	CHECK(exposure->minimum == 1 && exposure->maximum == 4017);
	CHECK(kstub_event_count >= 1 && kstub_events[0].id == V4L2_CID_EXPOSURE &&
	      (kstub_events[0].changes & V4L2_EVENT_CTRL_CH_RANGE));
	/* Mode 0 again: its tree's, not the paint made under it. */
	fmt = tree_fmt(1032, 776, tree_code(10, 2));
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	CHECK(owner_reads(4902, 4, 19000) && adapter_line_ns(&door.client) == 4902);
	CHECK(exposure->minimum == 1 && exposure->maximum == 3875);
	CHECK(door.log_count == 0 && door.commit_count == 0);
out:
	adapter_remove(&door.client);
	kstub_properties_clear();
	check_lock();
}

static void test_passive_paint_repaints_the_core_controls(void)
{
	const u32 black_cid = NXS_CSI2_CID_BLACK_LEVEL(NXS_CSI2_CID_BASE);
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_ctrl *black;

	begin("after a paint and after a format set the line row reads the line in effect on that one value, the black level the mode's range");
	serve_passive_tree(select_modes, ARRAY_SIZE(select_modes), follow_rows,
			   ARRAY_SIZE(follow_rows));
	CHECK(adapter_probe(&door.client) == 0);
	black = adapter_ctrl(&door.client, black_cid);
	CHECK(black && adapter_ctrl(&door.client, owner_cids[0]));
	if (!black || !adapter_ctrl(&door.client, owner_cids[0]))
		goto out;
	CHECK(adapter_set_ctrls(&door.client, owner_cids, owner_paint, 3) == 0);
	CHECK(adapter_line_ns(&door.client) == 14815);
	CHECK(line_row_is(14815));
	CHECK(black->minimum == 0 && black->maximum == 4095 && black->default_value == 240);

	/* The second mode: its tree's line, its black level's range. */
	fmt = tree_fmt(1032, 776, tree_code(10, 2));
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	CHECK(line_row_is(4902));
	CHECK(black->maximum == 1023 && black->default_value == 64);

	/* A paint under it. */
	CHECK(adapter_set_ctrls(&door.client, owner_cids, owner_paint, 3) == 0);
	CHECK(line_row_is(14815));
	CHECK(black->maximum == 1023 && black->default_value == 64);
	CHECK(door.log_count == 0 && door.commit_count == 0);
out:
	adapter_remove(&door.client);
	kstub_properties_clear();
	check_lock();
}

/* Whether the node offers none of the owner controls. */
static bool no_owner_controls(void)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(owner_cids); i++)
		if (adapter_ctrl(&door.client, owner_cids[i]))
			return false;

	return true;
}

/* passive_modes' first mode with the exposure and the gain, no line. */
static const u32 no_line_rows[] = {
	0, NXS_PARAM_ID_EXPOSURE_US, 4, 19000, 1, 4216,
	0, NXS_PARAM_ID_GAIN_MDB, 0, 48000, 100, 0,
};

/* passive_modes' first mode with the gain and the line, no exposure. */
static const u32 no_exposure_rows[] = {
	0, NXS_PARAM_ID_GAIN_MDB, 0, 48000, 100, 0,
	0, NXS_PARAM_ID_LINE_TIME_NS, 4902, 4902, 0, 4902,
};

static void test_passive_without_the_line_row_offers_no_owner(void)
{
	begin("a passive node whose rows name the exposure and no line probes and offers no owner control");
	serve_passive_tree(passive_modes, ARRAY_SIZE(passive_modes), no_line_rows,
			   ARRAY_SIZE(no_line_rows));
	CHECK(adapter_probe(&door.client) == 0);
	CHECK(adapter_ctrl(&door.client, V4L2_CID_EXPOSURE));
	CHECK(no_owner_controls());
	CHECK(door.log_count == 0);
	adapter_remove(&door.client);
	kstub_properties_clear();
	check_lock();
}

static void test_passive_without_the_exposure_row_offers_no_owner(void)
{
	begin("a passive node whose rows name the line and no exposure probes and offers no owner control");
	serve_passive_tree(passive_modes, ARRAY_SIZE(passive_modes), no_exposure_rows,
			   ARRAY_SIZE(no_exposure_rows));
	CHECK(adapter_probe(&door.client) == 0);
	CHECK(adapter_ctrl(&door.client, V4L2_CID_ANALOGUE_GAIN));
	CHECK(no_owner_controls());
	CHECK(door.log_count == 0);
	adapter_remove(&door.client);
	kstub_properties_clear();
	check_lock();
}

static void test_passive_owner_try_and_set_refuse_a_minimum_above_the_maximum(void)
{
	const s32 above[] = { 14815, 40000, 35000 };
	const s32 narrow[] = { 14815, 1, 5 };
	struct v4l2_mbus_framefmt fmt;

	begin("a try and a set of the owner controls alike refuse a minimum above the maximum");
	serve_passive(true);
	CHECK(adapter_probe(&door.client) == 0);
	if (!adapter_ctrl(&door.client, owner_cids[0])) {
		CHECK(adapter_ctrl(&door.client, owner_cids[0]));
		goto out;
	}
	CHECK(adapter_try_ctrls(&door.client, owner_cids, above, 3) == -EINVAL);
	CHECK(adapter_try_ctrls(&door.client, &owner_cids[1], &above[1], 1) == -EINVAL);
	CHECK(adapter_set_ctrls(&door.client, owner_cids, above, 3) == -EINVAL);
	CHECK(owner_reads(4902, 4, 19000));
	/* A try of a good paint answers 0 and changes nothing. */
	CHECK(adapter_try_ctrls(&door.client, owner_cids, owner_paint, 3) == 0);
	CHECK(owner_reads(4902, 4, 19000) && adapter_line_ns(&door.client) == 4902);

	/* The driver's own sync passes through a minimum above the maximum: 8 us over a painted 5 us. */
	CHECK(adapter_set_ctrls(&door.client, owner_cids, narrow, 3) == 0);
	fmt = tree_fmt(2064, 1552, tree_code(12, 0));
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	CHECK(owner_reads(8215, 8, 33000));
out:
	adapter_remove(&door.client);
	kstub_properties_clear();
	check_lock();
}

static void test_passive_owner_line_is_never_shorter_than_the_modes_shortest(void)
{
	const s32 short_paint[] = { 1, 30, 33100 };
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *line;

	begin("Owner Line Time's minimum is the mode's shortest line, and a paint under it reads that line");
	serve_passive(true);
	CHECK(adapter_probe(&door.client) == 0);
	line = adapter_ctrl(&door.client, owner_cids[0]);
	hblank = adapter_ctrl(&door.client, V4L2_CID_HBLANK);
	CHECK(line && hblank);
	if (!line || !hblank)
		goto out;
	/* 1032 pixels at 950.4 Mpixel/s take 1086 ns, rounded up. */
	CHECK(line->minimum == 1086);
	CHECK(adapter_set_ctrls(&door.client, owner_cids, short_paint, 3) == 0);
	CHECK(line->cur.val == 1086 && adapter_line_ns(&door.client) == 1086);
	CHECK(line_row_reads() == 1086);
	/* 1086 ns at 950.4 Mpixel/s is 1032 pixels, every one active. */
	CHECK(hblank->minimum == 0 && hblank->cur.val == 0);

	/* 2064 pixels at 792 Mpixel/s take 2607 ns: the minimum follows the mode. */
	fmt = tree_fmt(2064, 1552, tree_code(12, 0));
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	CHECK(line->minimum == 2607 && line->cur.val == 8215);
	CHECK(adapter_set_ctrls(&door.client, owner_cids, short_paint, 3) == 0);
	CHECK(line->cur.val == 2607 && adapter_line_ns(&door.client) == 2607);
	/* 2607 ns at 792 Mpixel/s is 2065 pixels, 1 past the active 2064. */
	CHECK(hblank->minimum == 1 && hblank->cur.val == 1);
out:
	adapter_remove(&door.client);
	kstub_properties_clear();
	check_lock();
}

static void test_passive_paint_activates_no_line_row_the_mode_does_not_list(void)
{
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_ctrl *row;

	begin("a paint activates no line row the mode does not list, and the row reads the painted line");
	serve_passive_tree(select_modes, ARRAY_SIZE(select_modes), one_line_rows,
			   ARRAY_SIZE(one_line_rows));
	CHECK(adapter_probe(&door.client) == 0);
	row = line_row_ctrl();
	CHECK(row && adapter_ctrl(&door.client, owner_cids[0]));
	if (!row || !adapter_ctrl(&door.client, owner_cids[0]))
		goto out;
	CHECK(adapter_subscribe(&door.client, row->id, true) == 0);
	fmt = tree_fmt(1032, 776, tree_code(10, 2));
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	CHECK(row->flags & V4L2_CTRL_FLAG_INACTIVE);

	/* The second mode lists no line: the paint moves the line and leaves the row inactive. */
	kstub_events_clear();
	CHECK(adapter_set_ctrls(&door.client, owner_cids, owner_paint, 3) == 0);
	CHECK(adapter_line_ns(&door.client) == 14815);
	CHECK(!event_heard(row->id, V4L2_EVENT_CTRL_CH_FLAGS));
	CHECK((row->flags & V4L2_CTRL_FLAG_INACTIVE) && line_row_is(14815));

	/* The first mode lists it: active on its tree line. */
	fmt = tree_fmt(2064, 1552, tree_code(12, 0));
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	CHECK(!(row->flags & V4L2_CTRL_FLAG_INACTIVE) && line_row_is(8215));
	CHECK(door.log_count == 0 && door.commit_count == 0);
out:
	adapter_remove(&door.client);
	kstub_properties_clear();
	check_lock();
}

/* Whether event @i names @id with exactly @changes and carries @value. */
static bool event_is(unsigned int i, u32 id, u32 changes, s32 value)
{
	return i < kstub_event_count && kstub_events[i].id == id &&
	       kstub_events[i].changes == changes && kstub_events[i].value == value;
}

static void test_passive_format_set_tells_the_owner_the_line_in_effect_and_the_moved_bounds(void)
{
	const s32 paint[] = { 1500, 30, 33100 };
	struct v4l2_mbus_framefmt fmt;
	unsigned int i;

	begin("a format set tells Owner Line Time's subscriber the line in effect once and a bound's where it moved");
	serve_passive_tree(select_modes, ARRAY_SIZE(select_modes), follow_rows,
			   ARRAY_SIZE(follow_rows));
	CHECK(adapter_probe(&door.client) == 0);
	if (!adapter_ctrl(&door.client, owner_cids[0])) {
		CHECK(adapter_ctrl(&door.client, owner_cids[0]));
		goto out;
	}
	fmt = tree_fmt(1032, 776, tree_code(10, 2));
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	for (i = 0; i < ARRAY_SIZE(owner_cids); i++)
		CHECK(adapter_subscribe(&door.client, owner_cids[i], true) == 0);
	/* 1500 ns, above this mode's shortest of 1086 ns. */
	CHECK(adapter_set_ctrls(&door.client, owner_cids, paint, 3) == 0);
	CHECK(adapter_line_ns(&door.client) == 1500);

	/* The first mode's shortest, 2607 ns, passes 1500: its line 8215 first, then the range. */
	kstub_events_clear();
	fmt = tree_fmt(2064, 1552, tree_code(12, 0));
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	CHECK(kstub_event_count == 4);
	CHECK(event_is(0, owner_cids[0], V4L2_EVENT_CTRL_CH_VALUE, 8215));
	CHECK(event_is(1, owner_cids[0], V4L2_EVENT_CTRL_CH_RANGE, 8215));
	CHECK(event_is(2, owner_cids[1], V4L2_EVENT_CTRL_CH_VALUE, 8));
	CHECK(event_is(3, owner_cids[2], V4L2_EVENT_CTRL_CH_VALUE, 33000));

	/* Back: the minimum falls under the line in effect, then the mode's line and bounds. */
	kstub_events_clear();
	fmt = tree_fmt(1032, 776, tree_code(10, 2));
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	CHECK(kstub_event_count == 4);
	CHECK(event_is(0, owner_cids[0], V4L2_EVENT_CTRL_CH_RANGE, 8215));
	CHECK(event_is(1, owner_cids[0], V4L2_EVENT_CTRL_CH_VALUE, 4902));
	CHECK(event_is(2, owner_cids[1], V4L2_EVENT_CTRL_CH_VALUE, 4));
	CHECK(event_is(3, owner_cids[2], V4L2_EVENT_CTRL_CH_VALUE, 19000));
	CHECK(door.log_count == 0 && door.commit_count == 0);
out:
	adapter_remove(&door.client);
	kstub_properties_clear();
	check_lock();
}

/*
 * Whether event @i is the line row's, with @changes, and carries @line as
 * its value, its minimum, its maximum and its default.
 */
static bool line_row_heard(unsigned int i, u32 changes, s32 line)
{
	const struct v4l2_ctrl *ctrl = line_row_ctrl();

	return ctrl && event_is(i, ctrl->id, changes, line) && kstub_events[i].minimum == line &&
	       kstub_events[i].maximum == line && kstub_events[i].default_value == line;
}

/* Whether the line row's stored value is @line, what a read gives, inside its range. */
static bool line_row_holds(s32 line)
{
	const struct v4l2_ctrl *ctrl = line_row_ctrl();

	return ctrl && line_row_is(line) && ctrl->cur.val == line && ctrl->cur.val >= ctrl->minimum &&
	       ctrl->cur.val <= ctrl->maximum;
}

static void test_passive_line_row_tells_its_subscriber_the_line_in_effect(void)
{
	const u32 moved = V4L2_EVENT_CTRL_CH_VALUE | V4L2_EVENT_CTRL_CH_RANGE;
	const s32 longer[] = { 20000, 30, 33100 };
	const s32 bounds[] = { 20000, 40, 33100 };
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_ctrl *row;

	begin("each paint and each format set that moves the line tells the line row's subscriber that line once");
	serve_passive_tree(select_modes, ARRAY_SIZE(select_modes), follow_rows,
			   ARRAY_SIZE(follow_rows));
	CHECK(adapter_probe(&door.client) == 0);
	row = line_row_ctrl();
	CHECK(row && adapter_ctrl(&door.client, owner_cids[0]));
	if (!row || !adapter_ctrl(&door.client, owner_cids[0]))
		goto out;
	CHECK(adapter_subscribe(&door.client, row->id, true) == 0);
	CHECK(line_row_holds(8215));

	/* Each paint of another line: one event, the line as value and range. */
	kstub_events_clear();
	CHECK(adapter_set_ctrls(&door.client, owner_cids, owner_paint, 3) == 0);
	CHECK(kstub_event_count == 1 && line_row_heard(0, moved, 14815));
	CHECK(line_row_holds(14815));
	kstub_events_clear();
	CHECK(adapter_set_ctrls(&door.client, owner_cids, longer, 3) == 0);
	CHECK(kstub_event_count == 1 && line_row_heard(0, moved, 20000));
	CHECK(line_row_holds(20000));

	/* The same line again, alone and with another bound: no event. */
	kstub_events_clear();
	CHECK(adapter_set_ctrls(&door.client, owner_cids, longer, 3) == 0);
	CHECK(adapter_set_ctrls(&door.client, owner_cids, bounds, 3) == 0);
	CHECK(kstub_event_count == 0 && line_row_holds(20000));

	/* A subscription that asks for the initial event hears the line in effect on its range. */
	CHECK(adapter_subscribe_initial(&door.client, row->id) == 0);
	CHECK(kstub_event_count == 1 &&
	      line_row_heard(0, V4L2_EVENT_CTRL_CH_FLAGS | V4L2_EVENT_CTRL_CH_VALUE, 20000));

	/* The second mode: its tree line, 4902 ns, in one event. */
	kstub_events_clear();
	fmt = tree_fmt(1032, 776, tree_code(10, 2));
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	CHECK(kstub_event_count == 1 && line_row_heard(0, moved, 4902));
	CHECK(line_row_holds(4902));

	/* The mode in effect again moves no line. */
	kstub_events_clear();
	CHECK(adapter_set_fmt(&door.client, &fmt) == 0);
	CHECK(kstub_event_count == 0 && line_row_holds(4902));
	CHECK(door.log_count == 0 && door.commit_count == 0);
out:
	adapter_remove(&door.client);
	kstub_properties_clear();
	check_lock();
}

/* 320x240 RAW8 on 4 lanes at 2376 Mbps, 1188 Mpixel/s: the shortest line is 270 ns. */
static const u32 fast_modes[] = {
	320, 240, 8, 0, 2376, 4,
};

/* The exposure, a rate down to 1 mfps and the line. */
static const u32 fast_rows[] = {
	0, NXS_PARAM_ID_EXPOSURE_US, 4, 19000, 1, 4216,
	0, NXS_PARAM_ID_FRAME_RATE_MFPS, 1, 30000, 1, 30000,
	0, NXS_PARAM_ID_LINE_TIME_NS, 1000, 1000, 0, 1000,
};

/*
 * Whether every control of the node's lines and bounds has its range inside
 * 32 bits and reads a value inside its range.
 */
static bool ranges_inside_32_bits(void)
{
	const u32 cids[] = {
		V4L2_CID_EXPOSURE, V4L2_CID_VBLANK, V4L2_CID_HBLANK, owner_cids[0], owner_cids[1],
		owner_cids[2], NXS_CSI2_CID_ROW(NXS_CSI2_CID_BASE, adapter_line_row(&door.client)),
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cids); i++) {
		const struct v4l2_ctrl *ctrl = adapter_ctrl(&door.client, cids[i]);
		s32 val;

		if (!ctrl || adapter_get_ctrl(&door.client, cids[i], &val))
			return false;
		if (ctrl->minimum < 0 || ctrl->maximum > S32_MAX || ctrl->minimum > ctrl->maximum ||
		    ctrl->default_value < ctrl->minimum || ctrl->default_value > ctrl->maximum)
			return false;
		if (val < ctrl->minimum || val > ctrl->maximum)
			return false;
	}

	return true;
}

static void test_passive_paint_keeps_every_range_inside_32_bits(void)
{
	const s32 longest[] = { S32_MAX, 0, 1000 };
	const s32 widest[] = { 1, S32_MAX - 1, S32_MAX };
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vblank;

	begin("a paint at either extreme leaves no range past 32 bits and no control reading outside its range");
	serve_passive_tree(fast_modes, ARRAY_SIZE(fast_modes), fast_rows, ARRAY_SIZE(fast_rows));
	CHECK(adapter_probe(&door.client) == 0);
	exposure = adapter_ctrl(&door.client, V4L2_CID_EXPOSURE);
	vblank = adapter_ctrl(&door.client, V4L2_CID_VBLANK);
	CHECK(exposure && vblank && adapter_ctrl(&door.client, owner_cids[0]));
	if (!exposure || !vblank || !adapter_ctrl(&door.client, owner_cids[0]))
		goto out;
	CHECK(ranges_inside_32_bits());

	/* The longest line is one second: 1188000000 pixels, 320 of them active. */
	CHECK(adapter_set_ctrls(&door.client, owner_cids, longest, 3) == 0);
	CHECK(ranges_inside_32_bits());
	CHECK(adapter_line_ns(&door.client) == NSEC_PER_SEC);
	CHECK(adapter_ctrl(&door.client, V4L2_CID_HBLANK)->minimum == 1188000000 - 320);

	/* The widest bounds at the shortest line: the exposure's lines end at S32_MAX. */
	CHECK(adapter_set_ctrls(&door.client, owner_cids, widest, 3) == 0);
	CHECK(ranges_inside_32_bits());
	CHECK(adapter_line_ns(&door.client) == 270);
	CHECK(exposure->minimum == S32_MAX && exposure->maximum == S32_MAX &&
	      exposure->cur.val == S32_MAX);
	/* 1 mfps is a frame of 1000 s, past S32_MAX lines of 270 ns: the blanking ends there. */
	CHECK(vblank->maximum == S32_MAX);
	CHECK(door.log_count == 0 && door.commit_count == 0);
out:
	adapter_remove(&door.client);
	kstub_properties_clear();
	check_lock();
}

static void test_module_version_names_the_layout(void)
{
	begin("the module version names the layout and the release");
	CHECK(!strcmp(kstub_module_version, "layout 1.3, release " NXS_VERSION));
}

int main(int argc, char **argv)
{
	kstub_verbose = argc > 1 && !strcmp(argv[1], "-v");
	test_probe_reads_the_header_and_the_description();
	test_probe_writes_nothing_for_one_page();
	test_probe_gates_the_door();
	test_probe_line_counts_the_modes();
	test_modes_take_their_depth_from_the_data_type();
	test_a_yuv_mode_is_left_out();
	test_a_mode_of_an_unnamed_data_type_is_left_out();
	test_probe_is_ready_on_a_description_and_a_personality();
	test_rows_by_family();
	test_probe_finds_the_cells_by_the_region_table();
	test_probe_reads_a_description_of_two_pages();
	test_probe_resets_a_page_left_set();
	test_probe_takes_a_list_before_its_rows();
	test_probe_reads_again_when_the_epoch_moved();
	test_probe_reads_in_the_adapters_pieces();
	test_probe_refuses_an_adapter_short_of_the_header();
	test_rows_past_255_and_a_list();
	test_commit_is_maskless();
	test_commit_writes_adjacent_rows_at_once();
	test_commit_leaves_out_a_row_that_takes_no_value();
	test_commit_refuses_a_run_ms_of_0();
	test_commit_splits_the_kinds();
	test_a_row_of_the_retired_kind_3();
	test_commit_reports_the_fault();
	test_commit_runs_under_the_epoch();
	test_commit_discards_a_cut_staging_write();
	test_commit_tells_a_cut_commit_by_the_counts();
	test_commit_waits_for_a_session();
	test_commit_ends_busy_at_step_1();
	test_commit_holds_at_an_inert_door();
	test_commit_ends_busy_past_run_ms();
	test_commit_discards_a_cut_run_ms_read();
	test_commit_held_commits_the_trigger_first();
	test_commit_translates_the_devices_errnos();
	test_mode_run();
	test_mode_run_while_streaming();
	test_status_reads_the_stream_and_the_mode_in_the_rows();
	test_tree_modes_against_the_map();
	test_controls_follow_their_rows_by_name();
	test_controls_follow_a_standard_row_by_its_id();
	test_controls_refuse_a_row_of_another_shape();
	test_controls_follow_rows_that_moved_to_a_larger_array();
	test_inline_enum_values_are_unsigned();
	test_a_row_with_no_reading_yet();
	test_controls_of_a_row_that_takes_no_value();
	test_controls_leave_their_rows_without_memory();
	test_reread_takes_the_handlers_lock_itself();
	test_test_pattern_menu_past_256();
	test_adapter_finds_the_line_by_its_id();
	test_adapter_flips_follow_a_reread();
	test_adapter_get_fmt_rereads_under_the_held_lock();
	test_adapter_set_fmt_rereads_under_the_held_lock();
	test_adapter_stream_on_rereads_under_its_lock();
	test_adapter_reads_a_door_emptied_mid_upload_under_the_held_lock();
	test_passive_rows_take_a_doors_controls();
	test_passive_set_transfers_nothing_and_raises_one_event();
	test_passive_without_rows_offers_the_line_and_the_bus();
	test_door_controls_commit_and_raise_their_events();
	test_passive_format_set_selects_its_mode_and_its_rows();
	test_passive_format_set_repaints_the_core_controls();
	test_passive_format_set_activates_the_modes_rows();
	test_passive_format_set_takes_the_first_of_two_modes_of_one_format();
	test_passive_line_row_reads_the_line_in_effect_on_that_one_value();
	test_passive_line_row_is_active_where_the_mode_lists_it();
	test_passive_owner_paints_the_line_and_the_exposure_bounds();
	test_passive_owner_controls_read_the_tree_before_a_paint();
	test_passive_paint_clamps_the_exposure_into_its_range();
	test_passive_owner_paints_while_the_node_streams();
	test_passive_format_set_returns_the_modes_tree_rows();
	test_passive_paint_repaints_the_core_controls();
	test_passive_without_the_line_row_offers_no_owner();
	test_passive_without_the_exposure_row_offers_no_owner();
	test_passive_owner_try_and_set_refuse_a_minimum_above_the_maximum();
	test_passive_owner_line_is_never_shorter_than_the_modes_shortest();
	test_passive_paint_activates_no_line_row_the_mode_does_not_list();
	test_passive_format_set_tells_the_owner_the_line_in_effect_and_the_moved_bounds();
	test_passive_line_row_tells_its_subscriber_the_line_in_effect();
	test_passive_paint_keeps_every_range_inside_32_bits();
	test_module_version_names_the_layout();
	kstub_devm_release(&door.client.dev);
	if (failures) {
		fprintf(stderr, "%d checks failed\n", failures);
		return 1;
	}
	printf("test-core: every check passed\n");

	return 0;
}

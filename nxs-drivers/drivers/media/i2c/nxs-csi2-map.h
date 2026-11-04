/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * nxs-csi2-map.h - the NXS map's capture cells, what a kernel driver knows
 *
 * generated from constants/nxs_map.yaml, constants/driver_image.yaml and
 * apps/zephyr/nxs-v1.0/VERSION by scripts/generate-constants.py --linux-out.
 * Do not edit by hand: those files are the one source, and
 * scripts/vendor-driver-map.sh writes this file into a driver repository.
 */

#ifndef NXS_CSI2_MAP_H
#define NXS_CSI2_MAP_H

/* The product version, MAJOR.MINOR.PATCH, this header belongs to. */
#define NXS_VERSION	"1.3.0"

/*
 * MapReg: The NXS map, one layout at the pod door and the camera door,
 * 16-bit register pointer.
 */
/*
 * The cells the camera door serves, where a kernel driver is bound: the
 * header's capture fields, the cell arrays, the commit block, the mode
 * table and the rows. Every other cell of the map is the pod's and reads 0
 * here.
 */
/*
 * `MAP_ID_POD` at the pod door, `MAP_ID_CAMERA` at the camera door and on a
 * smart camera.
 */
#define NXS_MAP_REG_MAP_ID	0x0000
/* Layout version of this map, `MAP_VERSION_VALUE`. */
#define NXS_MAP_REG_MAP_VERSION	0x0001
/*
 * `MapState`: INERT, READY, BUSY or FAULT, the state of the commit engine
 * both doors share. Live single-byte state, untearable under an I²C poll.
 */
#define NXS_MAP_REG_STATE	0x0002
/*
 * Positive errno of the commit that last ended FAULT, in the device's
 * numbering, 0 otherwise. The next accepted `COMMIT` clears it.
 */
#define NXS_MAP_REG_ERROR	0x0003
/* The running firmware's major version, a smart camera's bitstream version. */
#define NXS_MAP_REG_FW_VERSION_MAJOR	0x0004
#define NXS_MAP_REG_FW_VERSION_MINOR	0x0005
/*
 * Rows served at `ROWS` through this door, at most `MAX_ROWS`. The camera
 * door counts the camera personality's rows alone.
 */
#define NXS_MAP_REG_PARAM_COUNT	0x0006
/*
 * Rows in the mode table, at most `MODE_TABLE_MAX`; 0 on a pod that runs no
 * camera personality.
 */
#define NXS_MAP_REG_MODE_COUNT	0x0007
/* The mode-table value in effect, 0xFF before the first run. */
#define NXS_MAP_REG_CURRENT_MODE	0x0008
/*
 * 0 standby, 1 streaming or sampling: the `STREAM` row of the camera
 * personality at the camera door, and of the camera personality when one
 * runs, else the sampling personality, at the pod door.
 */
#define NXS_MAP_REG_STREAM	0x0009
/*
 * Wrapping count of commits that ended READY with a row applied, at either
 * door. A driver reads it to confirm a `COMMIT` landed.
 */
#define NXS_MAP_REG_APPLY_COUNT	0x000A
/* Wrapping count of commits that ended FAULT. */
#define NXS_MAP_REG_FAULT_COUNT	0x000B
/*
 * u16 LE milliseconds: the bound of a RUN commit, the longest its run
 * takes, 0 while no cam personality is loaded. The host waits this long
 * after a RUN commit before its first `STATE` read.
 */
#define NXS_MAP_REG_MODE_RUN_MS	0x0010
#define NXS_MAP_REG_MODE_RUN_MS_LEN	2
/* u16 LE milliseconds: the bound of a GATE commit. */
#define NXS_MAP_REG_GATE_RUN_MS	0x0012
#define NXS_MAP_REG_GATE_RUN_MS_LEN	2
/* u16 LE milliseconds: the bound of a TRIGGER commit. */
#define NXS_MAP_REG_TRIGGER_RUN_MS	0x0014
#define NXS_MAP_REG_TRIGGER_RUN_MS_LEN	2
/*
 * u16 LE milliseconds: the bound of an APPLY commit from the `COMMIT` byte
 * to READY. The host keeps the bus and sends nothing for this long after an
 * APPLY commit, then reads `STATE`.
 */
#define NXS_MAP_REG_APPLY_RUN_MS	0x0016
#define NXS_MAP_REG_APPLY_RUN_MS_LEN	2
/*
 * `ModuleKind`: what the device is, so a tool tells a pod from a smart
 * camera or a smart module. A pod with a head reads `POD` too.
 */
#define NXS_MAP_REG_MODULE_KIND	0x0018
/*
 * `PROTO_VERSION_VALUE`, the version of this host interface. At 0x19 so a
 * tool of the first interface reads it through a 1-byte pointer.
 */
#define NXS_MAP_REG_PROTO_VERSION	0x0019
#define NXS_MAP_REG_FW_VERSION_PATCH	0x001F
/*
 * `MAX_ROWS` u32 LE cells: the value of row i in effect, in the row's unit,
 * repainted after every commit and run.
 */
#define NXS_MAP_REG_LIVE	0x0100
#define NXS_MAP_REG_LIVE_LEN	256
/*
 * `MAX_ROWS` u32 LE cells: the lowest value row i takes in effect; 0 for an
 * enum row.
 */
#define NXS_MAP_REG_MIN	0x0200
#define NXS_MAP_REG_MIN_LEN	256
/*
 * `MAX_ROWS` u32 LE cells: the highest value row i takes in effect; the
 * count minus one for an enum row.
 */
#define NXS_MAP_REG_MAX	0x0300
#define NXS_MAP_REG_MAX_LEN	256
/*
 * `MAX_ROWS` u32 LE cells: the increment of row i in effect; 1 for an enum
 * row.
 */
#define NXS_MAP_REG_STEP	0x0400
#define NXS_MAP_REG_STEP_LEN	256
/*
 * `MAX_ROWS` u32 LE cells, read-write, row i at `MASK - 4 (i + 1)`: the
 * host's request for the next `COMMIT`.
 */
#define NXS_MAP_REG_STAGED	0x0500
#define NXS_MAP_REG_STAGED_LEN	256
/*
 * u64 LE, read-write: bit i names row i as staged for the next `COMMIT`.
 * Every masked row shares one owner and one `ParamKind`, else the commit
 * ends `EINVAL`, as does a masked read-only row or a row the device cannot
 * apply.
 */
#define NXS_MAP_REG_MASK	0x0600
#define NXS_MAP_REG_MASK_LEN	8
/*
 * Write 1 to apply the masked rows. Accepted in READY or FAULT: the write
 * takes `STAGED` and `MASK` as they stand and `STATE` turns BUSY at once.
 * Reads back 0.
 */
#define NXS_MAP_REG_COMMIT	0x0608
/*
 * `MODE_TABLE_MAX` rows of `MODE_ENTRY_SIZE` bytes, `ModeEntry` fields;
 * `MODE_COUNT` of them served.
 */
#define NXS_MAP_REG_MODE_TABLE	0x0700
#define NXS_MAP_REG_MODE_TABLE_LEN	256
/*
 * `PARAM_COUNT` descriptor rows of `ROW_SIZE` bytes, `RowField` fields, the
 * `PARAMS` record's bytes verbatim; static, served without a latch.
 */
#define NXS_MAP_REG_ROWS	0x0800
#define NXS_MAP_REG_ROWS_LEN	4096
/*
 * The device's serial, 12 bytes MSB first: the chip's UID96 on a pod, a
 * smart camera's own serial NUL-padded.
 */
#define NXS_MAP_REG_SERIAL	0x1C00
#define NXS_MAP_REG_SERIAL_LEN	12
/*
 * `STORE_SLOTS` regions of `PERSONALITY_INFO_SIZE` bytes, slot k at
 * `PERSONALITY_INFO + PERSONALITY_INFO_SIZE k`: the stored image's trailer
 * records, served from flash. At the camera door the first region serves
 * the camera personality's trailer, the block's description, and the other
 * regions read 0; a smart camera serves its own description there.
 */
#define NXS_MAP_REG_PERSONALITY_INFO	0x8000
#define NXS_MAP_REG_PERSONALITY_INFO_LEN	32768

/* MapState: `MapReg::STATE` values. */
/*
 * No personality runs, or no configure of the served personality has ended
 * DONE yet. A `COMMIT` is ignored.
 */
#define NXS_MAP_STATE_INERT	0
/* Configured and no commit in flight. */
#define NXS_MAP_STATE_READY	1
/*
 * A commit runs and the device masters its own bus. The host sends nothing
 * until the painted bound has passed.
 */
#define NXS_MAP_STATE_BUSY	2
/*
 * The last commit failed, `MapReg::ERROR` says why. The next accepted
 * `COMMIT` clears it.
 */
#define NXS_MAP_STATE_FAULT	3

/*
 * ModuleKind: `MapReg::MODULE_KIND` values: what the device is, with the
 * ports a device of that kind has as (name, PortType, PortDirection).
 */
/*
 * An NXS pod with a mikroBUS socket, a camera head or none: the one kind,
 * with a head or without one.
 */
#define NXS_MODULE_KIND_POD	1
/*
 * The 1.2.0 firmware's byte for a pod carrying a head; a reader takes it as
 * `POD`, with the same ports, so a 1.2.0 pod still reads.
 */
#define NXS_MODULE_KIND_HEADED	2
/*
 * A camera that serves the capture cells and its own description from its
 * own logic, an FPGA head, on a connector or on a hub's link.
 */
#define NXS_MODULE_KIND_SMART_CAMERA	3
/*
 * A compute module that serves this map, consumes the hub's streams and may
 * own the hub's bus, the N6 hat. Its `i2c1` is the hub's bus: fed by the
 * node that owns the hub, or the source of the hub's `i2c` input when the
 * module owns it.
 */
#define NXS_MODULE_KIND_SMART_MODULE	4

/*
 * ParamId: The standard meanings a parameter row may carry; the unit is
 * fixed per id and the compiler checks it.
 */
/* No standard meaning: the host offers the row by its name. */
#define NXS_PARAM_ID_CUSTOM	0
/* The exposure in microseconds; unit `us`; kind APPLY. */
#define NXS_PARAM_ID_EXPOSURE_US	1
/*
 * The frame rate in milli-frames per second, shown in frames per second;
 * unit `mfps`; kind APPLY. The device converts it to the frame period it
 * stages.
 */
#define NXS_PARAM_ID_FRAME_RATE_MFPS	2
/* The gain in millidecibels; unit `mdB`; kind APPLY. */
#define NXS_PARAM_ID_GAIN_MDB	3
/*
 * The black level in digital numbers at the mode's bit depth; unit `DN`;
 * kind APPLY.
 */
#define NXS_PARAM_ID_BLACK_LEVEL_DN	4
/* `FLIP_X` and `FLIP_Y` bits; an enum of 0..3; kind APPLY. */
#define NXS_PARAM_ID_FLIPS	5
/* The device's test-pattern codes, 0 first for off; an enum; kind APPLY. */
#define NXS_PARAM_ID_TEST_PATTERN	6
/*
 * The device's trigger conversions, `TriggerCode` values; an enum; kind
 * TRIGGER.
 */
#define NXS_PARAM_ID_TRIGGER	7
/*
 * 0 standby, 1 streaming or sampling; an enum; kind GATE. Every personality
 * that streams or samples carries one.
 */
#define NXS_PARAM_ID_STREAM	8
/*
 * A `MODE_VALUE` of the mode table; an enum; kind RUN, accepted in standby
 * only.
 */
#define NXS_PARAM_ID_MODE	9
/*
 * The device temperature in deci-degrees Celsius; signed; unit `dC`;
 * read-only; `TEMPERATURE_NONE` while unread.
 */
#define NXS_PARAM_ID_TEMPERATURE_DC	10
/* A lens's focus in steps; unit `step`; kind APPLY. Reserved. */
#define NXS_PARAM_ID_FOCUS_POSITION	11
/* A lens's f-number times 100; unit `f*100`; kind APPLY. Reserved. */
#define NXS_PARAM_ID_APERTURE	12
/* A lens's magnification times 100; unit `x*100`; kind APPLY. Reserved. */
#define NXS_PARAM_ID_ZOOM	13
/* A sampling personality's rate in hertz; unit `Hz`; kind RUN. */
#define NXS_PARAM_ID_SAMPLE_RATE_HZ	14
/*
 * The device's row: records the sample queue keeps before it overwrites the
 * oldest; unit `rec`; kind APPLY.
 */
#define NXS_PARAM_ID_FIFO_DEPTH	15
/*
 * The device's rows: `decimation`, the device-wide gate on the sample
 * window and every subject, then one per Cyphal SI subject, `decim.accel`,
 * `decim.gyro`, `decim.mag`, `decim.temp`, `decim.press`, `decim.gnss` and
 * `decim.scalar`; publish every Nth sample, 0 off; kind APPLY.
 */
#define NXS_PARAM_ID_DECIMATION	16
/*
 * The device's row: the CAN split termination, an enum of off, on; kind
 * APPLY.
 */
#define NXS_PARAM_ID_CAN_TERMINATION	17
/*
 * A click personality's row: the mount rotation of the sensor's frame, a
 * `Rotation` code 0..23; kind APPLY.
 */
#define NXS_PARAM_ID_MOUNT_ORIENTATION	18
/*
 * A device row: the pod's Cyphal node id, 0..125 static or 255 anonymous. A
 * commit stages it, `Cmd::STORE_PERSIST` keeps it and the next boot runs
 * it; kind APPLY.
 */
#define NXS_PARAM_ID_CAN_NODE_ID	19
/*
 * A device row: the CAN arbitration bitrate in bit/s, one of the profiles'
 * nominal rates, judged with `CAN_DATA_BITRATE` as a pair at the commit;
 * kind APPLY, the next boot runs it.
 */
#define NXS_PARAM_ID_CAN_BITRATE	20
/*
 * A device row: the CAN data-phase bitrate in bit/s, the nominal rate for
 * Classic CAN and above it for CAN FD; kind APPLY, the next boot runs it.
 */
#define NXS_PARAM_ID_CAN_DATA_BITRATE	21

/*
 * TriggerCode: The values of a `TRIGGER` row: the trigger conversions a
 * head serves, by their meaning, so a driver names them without the
 * trailer.
 */
/* No trigger: the sensor runs at its frame period. */
#define NXS_TRIGGER_CODE_FREERUN	0
/*
 * The trigger pulse's width is the exposure: one frame per pulse, the
 * exposure row set by the pulse.
 */
#define NXS_TRIGGER_CODE_FAST	1
/* One frame per trigger pulse at the exposure the row holds. */
#define NXS_TRIGGER_CODE_SEQUENTIAL	2

/*
 * RowField: Byte offsets inside one parameter row at `MapReg::ROWS`,
 * `ROW_SIZE` bytes; the same bytes as one row of the `PARAMS` record.
 */
/*
 * ASCII, NUL-padded: the parameter's name, what `nxs set` takes and a
 * custom control carries.
 */
#define NXS_ROW_FIELD_NAME	0
#define NXS_ROW_FIELD_NAME_LEN	16
/*
 * ASCII, NUL-padded: the unit of every value of the row; fixed per
 * `ParamId` for a standard row, empty for an enum.
 */
#define NXS_ROW_FIELD_UNIT	16
#define NXS_ROW_FIELD_UNIT_LEN	6
/*
 * u16 LE `ParamId`: the standard meaning the host maps to its control,
 * `ParamId::CUSTOM` for a row the host offers by name.
 */
#define NXS_ROW_FIELD_ID	22
#define NXS_ROW_FIELD_ID_LEN	2
/*
 * `ROW_TYPE_*` bits: enum or range, unsigned or signed, an enum's values 16
 * or 32 bits wide.
 */
#define NXS_ROW_FIELD_TYPE	24
/* `ParamKind`: how a commit of this row runs. */
#define NXS_ROW_FIELD_KIND	25
/* `ROW_FLAG_*` bits and the owning slot in `ROW_FLAG_OWNER`. */
#define NXS_ROW_FIELD_FLAGS	26
/*
 * An enum row's value count, at most `ROW_VALUES_MAX`, or
 * `ROW_WIDE_VALUES_MAX` under `ROW_TYPE_WIDE`; 3 for a range row.
 */
#define NXS_ROW_FIELD_COUNT	27
/* u32 LE: the value the row takes when the device configures its sensor. */
#define NXS_ROW_FIELD_DEFAULT	28
#define NXS_ROW_FIELD_DEFAULT_LEN	4
/*
 * An enum row: `COUNT` u16 LE values in the device's order, or u32 LE
 * values under `ROW_TYPE_WIDE`. A range row: min, max and step as u32 LE,
 * the rest 0.
 */
#define NXS_ROW_FIELD_VALUES	32
#define NXS_ROW_FIELD_VALUES_LEN	32

/*
 * ModeEntry: Byte offsets inside one row of `MapReg::MODE_TABLE`,
 * `MODE_ENTRY_SIZE` bytes.
 */
/* u16 LE active pixels per line. */
#define NXS_MODE_ENTRY_WIDTH	0
#define NXS_MODE_ENTRY_WIDTH_LEN	2
/* u16 LE active lines. */
#define NXS_MODE_ENTRY_HEIGHT	2
#define NXS_MODE_ENTRY_HEIGHT_LEN	2
/* Bits per pixel on the wire: 8, 10 or 12. */
#define NXS_MODE_ENTRY_BIT_DEPTH	4
/* CSI-2 lanes the mode drives. */
#define NXS_MODE_ENTRY_LANES	5
/* u16 LE megabits per second per lane. */
#define NXS_MODE_ENTRY_RATE_MBPS	6
#define NXS_MODE_ENTRY_RATE_MBPS_LEN	2
/*
 * The RAW sample depth of the stream's CSI-2 data type: 10 for RAW10, 12
 * for RAW12.
 */
#define NXS_MODE_ENTRY_DATA_TYPE	8
/* `MODE_FLAG_*` bits. */
#define NXS_MODE_ENTRY_FLAGS	9
/*
 * u32 LE: the fastest frame rate of the mode in thousandths of a frame per
 * second.
 */
#define NXS_MODE_ENTRY_FPS_MAX_MILLI	10
#define NXS_MODE_ENTRY_FPS_MAX_MILLI_LEN	4
/*
 * The value a RUN commit of the `MODE` row names, what `CURRENT_MODE`
 * reports.
 */
#define NXS_MODE_ENTRY_MODE_VALUE	14
/*
 * The Bayer order of the mode before any flip: 0 RGGB, 1 GRBG, 2 GBRG, 3
 * BGGR; 0xFF when the capture table names none.
 */
#define NXS_MODE_ENTRY_PIXEL_PHASE	15

/*
 * ParamKind: How a commit of a parameter runs; the masked rows of one
 * commit share one kind.
 */
/*
 * Every image's metadata block and every `PARAMS` row name one of
 * these four. `APPLY` takes effect in the running state: a click
 * parameter's value lives in `Param::current_value` (no bytecode
 * patch), and a consumer re-reads it when the param-change counter
 * advances; a camera parameter applies within the frame under the
 * sensor's group hold. `RUN` re-runs the configure program with the
 * new value: a click patches the bytecode operand and restarts the
 * VM (probe + configure, a sensor re-init), so the value is baked
 * into the bytecode; a camera's run is the mode switch. `GATE` parks
 * or starts the stream, and `TRIGGER` converts the trigger of a
 * running sensor.
 */
/*
 * A parameter applied in the running state: from `Param::current_value` on
 * a click, within the frame under the sensor's group hold on a camera (the
 * apply window of the camera door).
 */
#define NXS_PARAM_KIND_APPLY	0
/*
 * A parameter applied by a run of the configure program: a click's value
 * patched into the bytecode and a VM restart, a camera's mode switch, for
 * which the device reads BUSY for the whole run.
 */
#define NXS_PARAM_KIND_RUN	1
/* The stream gate: a run that parks or starts the sensor. */
#define NXS_PARAM_KIND_GATE	2
/* A trigger conversion on a running sensor. */
#define NXS_PARAM_KIND_TRIGGER	3

/*
 * The constants of the map a driver reads: the identity bytes, the sizes
 * and limits, the row and mode bits, the apply and busy contract.
 */
/* `MapReg::MAP_ID` at the pod door. */
#define NXS_MAP_ID_POD	0xAB
/* `MapReg::MAP_ID` at the camera door and on a smart camera. */
#define NXS_MAP_ID_CAMERA	0xC4
/* The layout version this file describes. */
#define NXS_MAP_VERSION_VALUE	1
/* The host interface version; 1 was the first interface with its 8-bit map. */
#define NXS_PROTO_VERSION_VALUE	2
/*
 * Bytes a read latches at its first byte, the header through the mode
 * table.
 */
#define NXS_MAP_LATCH_SIZE	2048
/* Bytes of one parameter or output row. */
#define NXS_ROW_SIZE	64
/* Parameter rows the layout holds. */
#define NXS_MAX_ROWS	64
#define NXS_ROW_NAME_LEN	16
#define NXS_ROW_UNIT_LEN	6
/* Enum values a row holds as u16. */
#define NXS_ROW_VALUES_MAX	16
#define NXS_ROW_WIDE_VALUES_MAX	8
#define NXS_ROW_TYPE_RANGE	0x01
#define NXS_ROW_TYPE_SIGNED	0x02
#define NXS_ROW_TYPE_WIDE	0x04
#define NXS_ROW_FLAG_READ_ONLY	0x01
#define NXS_ROW_FLAG_SET_BY_TRIGGER	0x02
/*
 * The graph's owner sets the row from the rig's facts, a line time for one;
 * a kernel driver offers it read-only.
 */
#define NXS_ROW_FLAG_GRAPH	0x04
/*
 * The owner tag in `RowField::FLAGS` bits 5..7: `OWNER_DEVICE` the device's
 * own rows, a personality's slot below it, else a tag no other owner
 * carries.
 */
#define NXS_ROW_FLAG_OWNER	0xE0
#define NXS_OWNER_DEVICE	7
#define NXS_MODE_TABLE_MAX	16
#define NXS_MODE_ENTRY_SIZE	16
#define NXS_MODE_FLAG_DEFAULT	0x01
#define NXS_MODE_FLAG_TRIGGERABLE	0x02
#define NXS_FLIP_X	0x01
#define NXS_FLIP_Y	0x02
#define NXS_TEMPERATURE_NONE	0x80000000
/*
 * How late past its bound a run can still be on the device's bus, the
 * transfer in flight at the deadline, a 128-byte burst at 400 kHz rounded
 * up. The device paints each kind's bound plus this, and a host that reads
 * `STATE` BUSY after the painted time reads it once more after this long.
 */
#define NXS_BOUND_LATENESS_MS	5

/*
 * The device's errno numbering, what the ERROR cell carries after a commit
 * ended FAULT (Interface Description §12). Five differ from Linux's:
 * ETIMEDOUT, ESTALE, ENOTSUP, EILSEQ and ECANCELED. A driver translates by
 * name.
 */
#define NXS_DEV_ENOENT	2
#define NXS_DEV_EIO	5
#define NXS_DEV_ENXIO	6
#define NXS_DEV_ENOEXEC	8
#define NXS_DEV_EBADF	9
#define NXS_DEV_EAGAIN	11
#define NXS_DEV_EFAULT	14
#define NXS_DEV_EBUSY	16
#define NXS_DEV_EEXIST	17
#define NXS_DEV_ENODEV	19
#define NXS_DEV_EINVAL	22
#define NXS_DEV_EFBIG	27
#define NXS_DEV_ENOSPC	28
#define NXS_DEV_ERANGE	34
#define NXS_DEV_ENODATA	61
#define NXS_DEV_ETIME	62
#define NXS_DEV_EPROTO	71
#define NXS_DEV_EBADMSG	77
#define NXS_DEV_EOPNOTSUPP	95
#define NXS_DEV_ETIMEDOUT	116
#define NXS_DEV_ESTALE	133
#define NXS_DEV_ENOTSUP	134
#define NXS_DEV_EILSEQ	138
#define NXS_DEV_ECANCELED	140

#endif /* NXS_CSI2_MAP_H */

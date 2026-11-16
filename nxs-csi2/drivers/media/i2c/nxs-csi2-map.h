/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * nxs-csi2-map.h - the NXS map's driver-door header, what a kernel driver knows
 *
 * generated from constants/nxs_map.yaml, constants/driver_image.yaml,
 * constants/personality.yaml and apps/zephyr/nxs-v1.0/VERSION by
 * scripts/generate-constants.py --linux-out.
 * Do not edit by hand: those files are the one source, and
 * scripts/vendor-driver-map.sh writes this file into a driver repository.
 */

#ifndef NXS_CSI2_MAP_H
#define NXS_CSI2_MAP_H

/* The product version, MAJOR.MINOR.PATCH, this header belongs to. */
#define NXS_VERSION	"1.3.0"

/*
 * MapReg: The NXS Device Interface, one layout at every door of every
 * device, 16-bit register pointer.
 */
/*
 * The header's cells a driver door serves, where a kernel driver is bound:
 * the identity, the commit engine, the counts and the region table. A
 * driver finds the CELLS, DESCRIPTION and DEVICE regions by their
 * REGION_TABLE entries and hard-codes no address past the header.
 */
/* `MAP_ID_VALUE` at every door of every device. */
#define NXS_MAP_REG_MAP_ID	0x0000
/*
 * `Door`: 1 the device's main door, 2 a driver door a host kernel driver
 * owns.
 */
#define NXS_MAP_REG_DOOR	0x0001
/*
 * `MapState`: INERT, READY, BUSY or FAULT, the state of the commit engine
 * both doors share. Live single-byte state, untearable under a host's read.
 */
#define NXS_MAP_REG_STATE	0x0002
/*
 * Positive errno of the commit that last ended FAULT, in the device's
 * numbering, 0 otherwise. The next accepted `COMMIT` clears it.
 */
#define NXS_MAP_REG_ERROR	0x0003
/* The running firmware's major version, an NXS Camera's bitstream version. */
#define NXS_MAP_REG_FW_VERSION_MAJOR	0x0004
#define NXS_MAP_REG_FW_VERSION_MINOR	0x0005
/*
 * Wrapping count of commits that ended READY with a row applied, at either
 * door. A driver reads it before and after a commit to tell its own verdict
 * from the last.
 */
#define NXS_MAP_REG_APPLY_COUNT	0x000A
/* Wrapping count of commits that ended FAULT. */
#define NXS_MAP_REG_FAULT_COUNT	0x000B
/*
 * u16 LE milliseconds: how long the commit of the staged rows takes, from
 * the `COMMIT` byte to READY, the device's own lateness included. The
 * device paints it at the stop of every transaction that wrote a `STAGED`
 * cell; with nothing staged it reads the time the device ends an empty
 * set's commit in, 9 ms on a pod. The host reads it, writes `COMMIT`, keeps
 * the bus silent that long and reads `STATE` once; BUSY then is the
 * device's failure. A pod computes it from the kinds of the staged rows and
 * `BOUND_LATENESS_MS`.
 */
#define NXS_MAP_REG_RUN_MS	0x0010
#define NXS_MAP_REG_RUN_MS_LEN	2
/*
 * u16 LE: rows this door serves. The camera door counts the camera
 * personality's rows, the pod door every personality's.
 */
#define NXS_MAP_REG_PARAM_COUNT	0x0012
#define NXS_MAP_REG_PARAM_COUNT_LEN	2
/*
 * u16 LE: rows the `CELLS` region holds, at most `ROW_CAPACITY_MAX`;
 * `POD_ROW_CAPACITY` on a pod.
 */
#define NXS_MAP_REG_ROW_CAPACITY	0x0014
#define NXS_MAP_REG_ROW_CAPACITY_LEN	2
/*
 * Personalities this door serves, the trailers of its description: 1 at a
 * camera door, 0 while no camera personality runs; at a pod door the
 * running personalities, the served camera personality first, then the
 * running sampling personality, then the pod's own trailer last.
 */
#define NXS_MAP_REG_PERSONALITY_COUNT	0x0016
/* Entries of `REGION_TABLE`, at most `REGION_TABLE_MAX`. */
#define NXS_MAP_REG_REGION_COUNT	0x0017
/*
 * `ModuleKind`: what the device is, so a tool tells a pod from an NXS
 * Camera or an NXS Module.
 */
#define NXS_MAP_REG_MODULE_KIND	0x0018
/*
 * Wrapping count of changes of the description: 0 until the first load,
 * then +1 on every load, so a host that reads it before and after a
 * description read knows the bytes are one description.
 */
#define NXS_MAP_REG_DESCRIPTOR_EPOCH	0x001E
#define NXS_MAP_REG_FW_VERSION_PATCH	0x001F
/*
 * `RELEASE_VALUE`, u32 LE: the release this device runs, its MAJOR, MINOR
 * and PATCH one byte each from the low byte up, then 0. A host accepts its
 * own MAJOR with a MINOR at or below its own, and refuses any other
 * release.
 */
#define NXS_MAP_REG_RELEASE	0x0026
#define NXS_MAP_REG_RELEASE_LEN	4
/* u32 LE: bytes of the description, every trailer the door serves. */
#define NXS_MAP_REG_DESC_SIZE	0x002A
#define NXS_MAP_REG_DESC_SIZE_LEN	4
/*
 * u16 LE, read-write: the `DESC_PAGE_SIZE` page of the description the
 * `DESCRIPTION` window serves, 0 at power-on.
 */
#define NXS_MAP_REG_DESC_PAGE	0x002E
#define NXS_MAP_REG_DESC_PAGE_LEN	2
/*
 * Write `COMMIT_APPLY` to apply the staged set, `COMMIT_DISCARD` to clear
 * it. Accepted in READY or FAULT: the apply takes the marked rows at the
 * stop, turns BUSY, judges the set, applies it and ends READY or FAULT.
 * Ignored while BUSY or INERT. Reads 0.
 */
#define NXS_MAP_REG_COMMIT	0x0030
/*
 * `ProbeVerdict`: how the last run of the door's personality's `probe()`
 * ended. The main door serves the sampling personality's, a driver door the
 * camera personality's. UNKNOWN until the probe runs after a boot or an
 * upload.
 */
#define NXS_MAP_REG_PROBE	0x0031
/*
 * `REGION_COUNT` entries of `REGION_ENTRY_SIZE` bytes, `RegionField`
 * fields: every region this door serves by type, base and size. A host
 * addresses the regions through it and hard-codes nothing past the header.
 */
#define NXS_MAP_REG_REGION_TABLE	0x0080
#define NXS_MAP_REG_REGION_TABLE_LEN	128

/* MapState: `MapReg::STATE` values. */
/*
 * The door serves no rows a commit takes yet: no personality runs at it, or
 * no configure of the served personality has ended DONE. A `COMMIT` is
 * ignored. A pod's main door takes the device's own rows and powers up
 * READY; its driver door reads INERT until a run configures the camera.
 */
#define NXS_MAP_STATE_INERT	0
/* Configured and no commit in flight. */
#define NXS_MAP_STATE_READY	1
/*
 * A commit runs and the device masters its own bus. The host sends nothing
 * until the painted `RUN_MS` has passed.
 */
#define NXS_MAP_STATE_BUSY	2
/*
 * The last commit failed, `MapReg::ERROR` says why. The next accepted
 * `COMMIT` clears it.
 */
#define NXS_MAP_STATE_FAULT	3

/* Door: `MapReg::DOOR` values: which door of a device a host reads. */
/*
 * The device's main door: every region the device serves, and every
 * personality's rows. A pod's pod door.
 */
#define NXS_DOOR_MAIN	1
/*
 * A door one host kernel driver owns: one personality's rows alone,
 * re-indexed from 0, and its description. The personality it serves comes
 * first at the main door, so both doors agree on its rows' indices. A pod's
 * camera door is one, and so is an NXS Camera's one door.
 */
#define NXS_DOOR_DRIVER	2

/* RegionType: `RegionField::TYPE` values: what a region of the table holds. */
/* An unused entry. */
#define NXS_REGION_TYPE_NONE	0
/*
 * Five arrays of `ROW_CAPACITY` u32 cells: `LIVE`, `MIN`, `MAX`, `STEP`,
 * `STAGED`, each `4 ROW_CAPACITY` bytes, consecutive.
 */
#define NXS_REGION_TYPE_CELLS	1
/* The `DESC_PAGE_SIZE` window onto the description. */
#define NXS_REGION_TYPE_DESCRIPTION	2
/* The latest sample record and the sample queue of a loop. */
#define NXS_REGION_TYPE_SAMPLES	3
/*
 * A pod's personality store: the directory, the upload and DFU chunks, the
 * model.
 */
#define NXS_REGION_TYPE_STORE	4
/*
 * The serial, and at a main door the build record, then the sub-regions the
 * entry's `FLAGS` lists: the time sync, the calibration record, the
 * diagnostics and the Context. A driver door serves the serial alone; a
 * main door that lists none serves the serial and the build.
 */
#define NXS_REGION_TYPE_DEVICE	5

/*
 * RegionField: Byte offsets inside one entry of `MapReg::REGION_TABLE`,
 * `REGION_ENTRY_SIZE` bytes.
 */
/* `RegionType`. */
#define NXS_REGION_FIELD_TYPE	0
/*
 * `REGION_FLAG_*` bits: bit 0 the region takes writes; on a `DEVICE` entry
 * bits 1 to 4 list its sub-regions.
 */
#define NXS_REGION_FIELD_FLAGS	1
/* u16 LE: the region's first address, a multiple of 256. */
#define NXS_REGION_FIELD_BASE	2
#define NXS_REGION_FIELD_BASE_LEN	2
/* u16 LE: the region's bytes. */
#define NXS_REGION_FIELD_SIZE	4
#define NXS_REGION_FIELD_SIZE_LEN	2

/*
 * CellsArray: The arrays of a `CELLS` region in their order, each `4
 * ROW_CAPACITY` bytes: row i's cell of array a sits at the region's `BASE`
 * + 4 (a `ROW_CAPACITY` + i). A host reads the base from the region's
 * `REGION_TABLE` entry and `ROW_CAPACITY` from the header.
 */
/*
 * The value of row i in effect, in the row's unit, repainted after every
 * commit; two's complement when signed.
 */
#define NXS_CELLS_ARRAY_LIVE	0
/* The lowest value row i takes in effect; 0 for an enum row. */
#define NXS_CELLS_ARRAY_MIN	1
/*
 * The highest value row i takes in effect; the count minus one for an enum
 * row.
 */
#define NXS_CELLS_ARRAY_MAX	2
/*
 * The increment of row i in effect; 1 for an enum row, 0 while the row
 * takes no value.
 */
#define NXS_CELLS_ARRAY_STEP	3
/*
 * Read-write: the host's request for row i. A write marks the row as staged
 * for the next `COMMIT`, and the device paints `RUN_MS` for the marked set
 * at the transaction's stop. A write of a cell at or past `PARAM_COUNT`
 * stores nothing, and the commit of its set ends FAULT with `EINVAL`.
 */
#define NXS_CELLS_ARRAY_STAGED	4

/*
 * ModuleKind: `MapReg::MODULE_KIND` values: the class of the device, a code
 * a host prints and needs for nothing else. A host describes a door by its
 * description: every trailer's `PORTS`, `ENTRIES` and `VALUES` records
 * carry its device's ports, programs and output values, a pod's own trailer
 * the pod's. A host reads a code it does not know as a device's own class.
 */
/*
 * A device that stores and runs personalities: a `STORE` region, a VM, the
 * trailers of the personalities it runs and its own trailer last. An NXS
 * Pod is one.
 */
#define NXS_MODULE_KIND_POD	1
/*
 * A device that serves its description from its own logic, a ROM, with no
 * store and no VM. An NXS Camera is one.
 */
#define NXS_MODULE_KIND_FIXED	3
/*
 * A compute module that serves this map and runs schedules on its own
 * executor (`Cmd::ACTIVATE_SLOT`). An NXS Module is one.
 */
#define NXS_MODULE_KIND_MODULE	4

/*
 * ParamId: The standard meanings a parameter row may carry, each a
 * `ParamFamily` byte above an id within the family; the unit and the kind
 * are fixed per id and the compiler checks them.
 */
/* No standard meaning: the host offers the row by its name. */
#define NXS_PARAM_ID_CUSTOM	0
/*
 * 0 standby, 1 streaming or sampling; an enum; kind GATE. A personality
 * that serves one has a gate.
 */
#define NXS_PARAM_ID_STREAM	1
/*
 * The mode in effect, one of the personality's modes; an enum; kind RUN,
 * accepted in standby only.
 */
#define NXS_PARAM_ID_MODE	2
/* A sampling personality's rate in hertz; unit `Hz`; kind RUN. */
#define NXS_PARAM_ID_SAMPLE_RATE_HZ	3
/*
 * The device temperature in deci-degrees Celsius; signed; unit `dC`;
 * read-only; `ROW_UNREAD` before its first reading.
 */
#define NXS_PARAM_ID_TEMPERATURE_DC	4
/*
 * A sampling personality's row: the mount rotation of the sensor's frame, a
 * `Rotation` code 0..23; kind APPLY.
 */
#define NXS_PARAM_ID_MOUNT_ORIENTATION	5
/*
 * 0 parked in the safe state, 1 enabled: the gate of a device's output
 * stage; an enum; kind GATE.
 */
#define NXS_PARAM_ID_ENABLE	6
/*
 * The personality's state codes, named in `PRESENTATION`; an enum;
 * read-only.
 */
#define NXS_PARAM_ID_STATE	7
/*
 * The device's fault codes, 0 for none, named in `PRESENTATION`; an enum;
 * read-only.
 */
#define NXS_PARAM_ID_FAULT	8
/* The exposure in microseconds; unit `us`; kind APPLY. */
#define NXS_PARAM_ID_EXPOSURE_US	257
/*
 * The frame rate in milli-frames per second, shown in frames per second;
 * unit `mfps`; kind APPLY. The device converts it to the frame period it
 * stages.
 */
#define NXS_PARAM_ID_FRAME_RATE_MFPS	258
/* The gain in millidecibels; unit `mdB`; kind APPLY. */
#define NXS_PARAM_ID_GAIN_MDB	259
/*
 * The black level in digital numbers at the mode's bit depth; unit `DN`;
 * kind APPLY.
 */
#define NXS_PARAM_ID_BLACK_LEVEL_DN	260
/* `FLIP_X` and `FLIP_Y` bits; an enum of 0..3; kind APPLY. */
#define NXS_PARAM_ID_FLIPS	261
/* The device's test-pattern codes, 0 first for off; an enum; kind APPLY. */
#define NXS_PARAM_ID_TEST_PATTERN	262
/*
 * The device's trigger conversions, `TriggerCode` values; an enum; kind
 * RUN.
 */
#define NXS_PARAM_ID_TRIGGER	263
/*
 * The line time in nanoseconds the device runs its lines at, a pair's
 * longest line; unit `ns`; kind RUN; the graph's owner sets it
 * (`ROW_FLAG_GRAPH`).
 */
#define NXS_PARAM_ID_LINE_TIME_NS	264
/* A lens's focus in steps; unit `step`; kind APPLY. Reserved. */
#define NXS_PARAM_ID_FOCUS_POSITION	265
/* A lens's f-number times 100; unit `f*100`; kind APPLY. Reserved. */
#define NXS_PARAM_ID_APERTURE	266
/* A lens's magnification times 100; unit `x*100`; kind APPLY. Reserved. */
#define NXS_PARAM_ID_ZOOM	267
/*
 * The part of the sensor on the head as the device read it, `SensorPart`
 * values; an enum; read-only. A pod reads it with the personality's part
 * probe at boot and at a configure, UNKNOWN until a read names one; an NXS
 * Camera serves its own.
 */
#define NXS_PARAM_ID_PART	268
/*
 * The device's row: records the sample queue keeps before it overwrites the
 * oldest; unit `rec`; kind APPLY.
 */
#define NXS_PARAM_ID_FIFO_DEPTH	513
/*
 * The device's rows: `decimation`, the device-wide gate on the sample
 * window and every subject, then one per Cyphal SI subject, `decim.accel`,
 * `decim.gyro`, `decim.mag`, `decim.temp`, `decim.press`, `decim.gnss` and
 * `decim.scalar`; publish every Nth sample, 0 off; kind APPLY.
 */
#define NXS_PARAM_ID_DECIMATION	514
/*
 * The device's row: the CAN split termination, an enum of off, on; kind
 * APPLY.
 */
#define NXS_PARAM_ID_CAN_TERMINATION	515
/*
 * A device row: the pod's Cyphal node id, 0..125 static or 255 anonymous. A
 * commit stages it, `Cmd::STORE_PERSIST` keeps it and the next boot runs
 * it; kind APPLY.
 */
#define NXS_PARAM_ID_CAN_NODE_ID	516
/*
 * A device row: the CAN arbitration bitrate in bit/s, one of the profiles'
 * nominal rates, judged with `CAN_DATA_BITRATE` as a pair at the commit;
 * kind APPLY, the next boot runs it.
 */
#define NXS_PARAM_ID_CAN_BITRATE	517
/*
 * A device row: the CAN data-phase bitrate in bit/s, the nominal rate for
 * Classic CAN and above it for CAN FD; kind APPLY, the next boot runs it.
 */
#define NXS_PARAM_ID_CAN_DATA_BITRATE	518
/*
 * A drive's position setpoint in milliradians; signed; unit `mrad`; kind
 * APPLY.
 */
#define NXS_PARAM_ID_POSITION	769
/*
 * A drive's velocity setpoint in milliradians a second; signed; unit
 * `mrad/s`; kind APPLY.
 */
#define NXS_PARAM_ID_VELOCITY	770
/*
 * A drive's torque setpoint in millinewton-metres; signed; unit `mN*m`;
 * kind APPLY.
 */
#define NXS_PARAM_ID_TORQUE	771
/*
 * A drive's current setpoint in milliamperes; signed; unit `mA`; kind
 * APPLY.
 */
#define NXS_PARAM_ID_CURRENT	772
/* The lowest position a drive takes; signed; unit `mrad`; kind APPLY. */
#define NXS_PARAM_ID_POSITION_MIN	773
/* The highest position a drive takes; signed; unit `mrad`; kind APPLY. */
#define NXS_PARAM_ID_POSITION_MAX	774
/* The fastest a drive turns, from 0; unit `mrad/s`; kind APPLY. */
#define NXS_PARAM_ID_VELOCITY_LIMIT	775
/* The most torque a drive applies, from 0; unit `mN*m`; kind APPLY. */
#define NXS_PARAM_ID_TORQUE_LIMIT	776
/* The most current a drive draws, from 0; unit `mA`; kind APPLY. */
#define NXS_PARAM_ID_CURRENT_LIMIT	777
/*
 * What a drive's loop controls: 0 position, 1 velocity, 2 torque, 3
 * current; an enum; kind RUN.
 */
#define NXS_PARAM_ID_CONTROL_MODE	778
/*
 * The position a drive measures, `POSITION` plus 0x10; signed; unit `mrad`;
 * read-only.
 */
#define NXS_PARAM_ID_POSITION_MEASURED	785
/* The velocity a drive measures; signed; unit `mrad/s`; read-only. */
#define NXS_PARAM_ID_VELOCITY_MEASURED	786
/* The torque a drive measures; signed; unit `mN*m`; read-only. */
#define NXS_PARAM_ID_TORQUE_MEASURED	787
/* The current a drive measures; signed; unit `mA`; read-only. */
#define NXS_PARAM_ID_CURRENT_MEASURED	788

/*
 * ParamFamily: The family byte of a `ParamId`, its high byte: whose table
 * names the id within the family, its low byte.
 */
/*
 * A family owns its id space. The kind and the unit stay bound to the
 * id within its family, and a host offers a row of a family it does
 * not know by the row's name, as it offers a `CUSTOM` row.
 */
/* The meanings any device serves. */
#define NXS_PARAM_FAMILY_GENERIC	0
/* A camera personality's rows. */
#define NXS_PARAM_FAMILY_CAMERA	1
/* A pod's own rows: its sample transport and its CAN profile. */
#define NXS_PARAM_FAMILY_POD	2
/*
 * A drive's rows: its setpoints, their limits, its control mode and the
 * quantities it measures.
 */
#define NXS_PARAM_FAMILY_ACTUATOR	3

/*
 * RowField: Byte offsets inside one row of the `PARAMS` record, `ROW_SIZE`
 * bytes.
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
 * or 32 bits wide, or in a `ROW_VALUES` record (`ROW_TYPE_LIST`).
 */
#define NXS_ROW_FIELD_TYPE	24
/* `ParamKind`: how a commit of this row runs. */
#define NXS_ROW_FIELD_KIND	25
/*
 * `ROW_FLAG_*` bits and the index of the row's trailer in `ROW_FLAG_OWNER`;
 * bit 1 is reserved and reads 0.
 */
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
 * then `FRESH_MS` at `ROW_FRESH_MS_AT`, the run values of the device-read
 * bounds at `ROW_BOUND_MIN_AT` and `ROW_BOUND_MAX_AT`, the rest 0.
 */
#define NXS_ROW_FIELD_VALUES	32
#define NXS_ROW_FIELD_VALUES_LEN	32

/*
 * CameraMapField: Byte offsets inside the head of the `CAMERA_MAP` record,
 * `CAMERA_MAP_HEAD_SIZE` bytes. With N its `MODE_COUNT`, N `CameraLawField`
 * rows follow the head, then N `ModeEntry` rows, then the `CameraTailField`
 * tail at `CAMERA_MAP_HEAD_SIZE` + N (`CAMERA_LAW_ROW_SIZE` +
 * `MODE_ENTRY_SIZE`).
 */
/* N, the modes of the record, at most `MODE_TABLE_MAX`. */
#define NXS_CAMERA_MAP_FIELD_MODE_COUNT	0
/* u16 LE: the gain's capture units per decibel, never 0. */
#define NXS_CAMERA_MAP_FIELD_GAIN_FACTOR	1
#define NXS_CAMERA_MAP_FIELD_GAIN_FACTOR_LEN	2
/*
 * The run-value index of each value a commit stages, 0xFF for none, in this
 * order: the action, the exposure, the frame length, the gain, the trigger,
 * the test pattern, the line time, the mode, the frame period, the black
 * level, the flips and the part.
 */
#define NXS_CAMERA_MAP_FIELD_RUN_INDICES	3
#define NXS_CAMERA_MAP_FIELD_RUN_INDICES_LEN	12
/* u16 LE: the gain's lowest value in capture units. */
#define NXS_CAMERA_MAP_FIELD_GAIN_MIN	15
#define NXS_CAMERA_MAP_FIELD_GAIN_MIN_LEN	2
/* u16 LE: the gain's highest value in capture units. */
#define NXS_CAMERA_MAP_FIELD_GAIN_MAX	17
#define NXS_CAMERA_MAP_FIELD_GAIN_MAX_LEN	2
/* u16 LE: the gain's increment in capture units. */
#define NXS_CAMERA_MAP_FIELD_GAIN_STEP	19
#define NXS_CAMERA_MAP_FIELD_GAIN_STEP_LEN	2
/* u32 LE: the longest frame in lines. */
#define NXS_CAMERA_MAP_FIELD_FRAME_LENGTH_MAX	21
#define NXS_CAMERA_MAP_FIELD_FRAME_LENGTH_MAX_LEN	4
/* u32 LE: the exposure floor in lines. */
#define NXS_CAMERA_MAP_FIELD_EXPOSURE_MIN	25
#define NXS_CAMERA_MAP_FIELD_EXPOSURE_MIN_LEN	4
/* u16 LE: the lines a frame runs past its exposure at least. */
#define NXS_CAMERA_MAP_FIELD_EXPOSURE_MARGIN	29
#define NXS_CAMERA_MAP_FIELD_EXPOSURE_MARGIN_LEN	2

/*
 * CameraLawField: Byte offsets inside one law row of the `CAMERA_MAP`
 * record, `CAMERA_LAW_ROW_SIZE` bytes, one per mode in the mode table's
 * order.
 */
/* u32 LE: the mode's line time in ns, never 0. */
#define NXS_CAMERA_LAW_FIELD_LINE_TIME_NS	0
#define NXS_CAMERA_LAW_FIELD_LINE_TIME_NS_LEN	4
/* u32 LE: the mode's shortest frame in lines. */
#define NXS_CAMERA_LAW_FIELD_FRAME_LENGTH_FLOOR	4
#define NXS_CAMERA_LAW_FIELD_FRAME_LENGTH_FLOOR_LEN	4

/*
 * ModeEntry: Byte offsets inside one row of the `CAMERA_MAP` record's mode
 * table, `MODE_ENTRY_SIZE` bytes, N of them after the N law rows.
 */
/* u16 LE active pixels per line. */
#define NXS_MODE_ENTRY_WIDTH	0
#define NXS_MODE_ENTRY_WIDTH_LEN	2
/* u16 LE active lines. */
#define NXS_MODE_ENTRY_HEIGHT	2
#define NXS_MODE_ENTRY_HEIGHT_LEN	2
/* CSI-2 lanes the mode drives. */
#define NXS_MODE_ENTRY_LANES	5
/* u16 LE megabits per second per lane. */
#define NXS_MODE_ENTRY_RATE_MBPS	6
#define NXS_MODE_ENTRY_RATE_MBPS_LEN	2
/*
 * The CSI-2 data type code of the mode's pixel stream, as the packet header
 * carries it: 0x2B RAW10, 0x1E YUV422_8. The code states the bits of one
 * sample and of one pixel on the wire (`Csi2DataType`).
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
 * The value a RUN commit of the `MODE` row names, what the row's `LIVE`
 * reads in the mode.
 */
#define NXS_MODE_ENTRY_MODE_VALUE	14
/*
 * The Bayer order of a RAW mode before any flip: 0 RGGB, 1 GRBG, 2 GBRG, 3
 * BGGR; 0xFF when the capture table names none. Every other value is
 * reserved for the sample order of another data type.
 */
#define NXS_MODE_ENTRY_PIXEL_PHASE	15

/*
 * Csi2DataType: The CSI-2 data type codes a mode row's `DATA_TYPE` names,
 * as the packet header carries them, each with the bits of one sample and
 * of one pixel on the wire. A device may state a code the table does not
 * name, and a reader that needs its depth refuses that mode by its code.
 */
/* RAW8: 8-bit samples, one a byte. */
#define NXS_CSI2_DATA_TYPE_RAW8	0x2A
/* RAW10: 10-bit samples, four in five bytes. */
#define NXS_CSI2_DATA_TYPE_RAW10	0x2B
/* RAW12: 12-bit samples, two in three bytes. */
#define NXS_CSI2_DATA_TYPE_RAW12	0x2C
/* RAW14: 14-bit samples, four in seven bytes. */
#define NXS_CSI2_DATA_TYPE_RAW14	0x2D
/* RAW16: 16-bit samples, one in two bytes. */
#define NXS_CSI2_DATA_TYPE_RAW16	0x2E
/*
 * YUV 4:2:0 8-bit: lines of Y alone alternate with lines of U Y V Y, 12
 * bits a pixel on average.
 */
#define NXS_CSI2_DATA_TYPE_YUV420_8	0x18
/* YUV 4:2:2 8-bit: U Y V Y, 16 bits a pixel. */
#define NXS_CSI2_DATA_TYPE_YUV422_8	0x1E
/* YUV 4:2:2 10-bit: U Y V Y, 20 bits a pixel. */
#define NXS_CSI2_DATA_TYPE_YUV422_10	0x1F
/*
 * RGB565: 16 bits a pixel, its red and blue samples 5 bits each and its
 * green sample 6.
 */
#define NXS_CSI2_DATA_TYPE_RGB565	0x22
/* RGB888: three 8-bit samples, 24 bits a pixel. */
#define NXS_CSI2_DATA_TYPE_RGB888	0x24

/*
 * The bits of one sample, for Csi2DataType code `code`; 0 for a code the
 * enum does not name.
 */
static inline unsigned int nxs_csi2_data_type_sample_bits(unsigned int code)
{
	switch (code) {
	case NXS_CSI2_DATA_TYPE_RAW8:
		return 8;
	case NXS_CSI2_DATA_TYPE_RAW10:
		return 10;
	case NXS_CSI2_DATA_TYPE_RAW12:
		return 12;
	case NXS_CSI2_DATA_TYPE_RAW14:
		return 14;
	case NXS_CSI2_DATA_TYPE_RAW16:
		return 16;
	case NXS_CSI2_DATA_TYPE_YUV420_8:
		return 8;
	case NXS_CSI2_DATA_TYPE_YUV422_8:
		return 8;
	case NXS_CSI2_DATA_TYPE_YUV422_10:
		return 10;
	case NXS_CSI2_DATA_TYPE_RGB565:
		return 5;
	case NXS_CSI2_DATA_TYPE_RGB888:
		return 8;
	default:
		return 0;
	}
}

/*
 * The bits one pixel takes on the wire, for Csi2DataType code `code`; 0 for
 * a code the enum does not name.
 */
static inline unsigned int nxs_csi2_data_type_pixel_bits(unsigned int code)
{
	switch (code) {
	case NXS_CSI2_DATA_TYPE_RAW8:
		return 8;
	case NXS_CSI2_DATA_TYPE_RAW10:
		return 10;
	case NXS_CSI2_DATA_TYPE_RAW12:
		return 12;
	case NXS_CSI2_DATA_TYPE_RAW14:
		return 14;
	case NXS_CSI2_DATA_TYPE_RAW16:
		return 16;
	case NXS_CSI2_DATA_TYPE_YUV420_8:
		return 12;
	case NXS_CSI2_DATA_TYPE_YUV422_8:
		return 16;
	case NXS_CSI2_DATA_TYPE_YUV422_10:
		return 20;
	case NXS_CSI2_DATA_TYPE_RGB565:
		return 16;
	case NXS_CSI2_DATA_TYPE_RGB888:
		return 24;
	default:
		return 0;
	}
}

/*
 * CameraTailField: Byte offsets inside the tail of the `CAMERA_MAP` record,
 * after its mode table. A record that ends after the mode table carries no
 * tail, and the device takes its default bounds.
 */
/*
 * Three u16 LE: the longest run of an APPLY, a RUN and a GATE commit in ms,
 * in `ParamKind` order.
 */
#define NXS_CAMERA_TAIL_FIELD_BOUND_MS	0
#define NXS_CAMERA_TAIL_FIELD_BOUND_MS_LEN	6
/*
 * The part probe's codes, at most `PART_CODES_MAX`; 0 for no probe, and the
 * record ends here.
 */
#define NXS_CAMERA_TAIL_FIELD_PART_COUNT	6
/*
 * The probe, `CAMERA_PROBE_SIZE` bytes of `CameraProbeField` fields, then
 * `PART_COUNT` codes of `CAMERA_PART_CODE_SIZE` bytes each,
 * `CameraPartCodeField` fields.
 */
#define NXS_CAMERA_TAIL_FIELD_PART_PROBE	7
#define NXS_CAMERA_TAIL_FIELD_PART_PROBE_LEN	22

/*
 * CameraProbeField: Byte offsets inside the part probe of the `CAMERA_MAP`
 * record, `CAMERA_PROBE_SIZE` bytes: the register the probe reads and the
 * register that wakes the head for the read.
 */
/* u16 LE: the register the probe reads. */
#define NXS_CAMERA_PROBE_FIELD_REG	0
#define NXS_CAMERA_PROBE_FIELD_REG_LEN	2
/* Bytes of the register's value, 1 to 4. */
#define NXS_CAMERA_PROBE_FIELD_REG_WIDTH	2
/*
 * The value's byte order: 0 least significant first, 1 most significant
 * first.
 */
#define NXS_CAMERA_PROBE_FIELD_REG_ORDER	3
/* u32 LE: the bits of the value the codes compare. */
#define NXS_CAMERA_PROBE_FIELD_MASK	4
#define NXS_CAMERA_PROBE_FIELD_MASK_LEN	4
/* u16 LE: the register that wakes the head, 0xFFFF for none. */
#define NXS_CAMERA_PROBE_FIELD_WAKE_REG	8
#define NXS_CAMERA_PROBE_FIELD_WAKE_REG_LEN	2
/* Bytes of the wake register's value. */
#define NXS_CAMERA_PROBE_FIELD_WAKE_WIDTH	10
/* The wake value's byte order, as `REG_ORDER`. */
#define NXS_CAMERA_PROBE_FIELD_WAKE_ORDER	11
/* u32 LE: the value written to the wake register before the read. */
#define NXS_CAMERA_PROBE_FIELD_WAKE_BEFORE	12
#define NXS_CAMERA_PROBE_FIELD_WAKE_BEFORE_LEN	4
/* u32 LE: the value written to it after the read. */
#define NXS_CAMERA_PROBE_FIELD_WAKE_AFTER	16
#define NXS_CAMERA_PROBE_FIELD_WAKE_AFTER_LEN	4
/* u16 LE: the settle after each wake write, in ms. */
#define NXS_CAMERA_PROBE_FIELD_SETTLE_MS	20
#define NXS_CAMERA_PROBE_FIELD_SETTLE_MS_LEN	2

/*
 * CameraPartCodeField: Byte offsets inside one code of the part probe,
 * `CAMERA_PART_CODE_SIZE` bytes.
 */
/* u32 LE: the masked value the code matches. */
#define NXS_CAMERA_PART_CODE_FIELD_VALUE	0
#define NXS_CAMERA_PART_CODE_FIELD_VALUE_LEN	4
/* The `SensorPart` the value names. */
#define NXS_CAMERA_PART_CODE_FIELD_PART	4

/*
 * ParamKind: How a commit of a parameter runs; the staged rows of one
 * commit share one kind.
 */
/*
 * Every image's metadata block and every `PARAMS` row name one of
 * these three, the mechanism a commit runs by on any device. `APPLY`
 * takes effect in the running state: a click parameter's value lives
 * in `Param::current_value` (no bytecode patch), and a consumer
 * re-reads it when the param-change counter advances; a camera
 * parameter applies within the frame under the sensor's group hold.
 * `RUN` re-runs the configure program with the new value: a click
 * patches the bytecode operand and restarts the VM (probe +
 * configure, a sensor re-init), so the value is baked into the
 * bytecode; a camera's run is the mode switch, or the trigger
 * conversion its program runs for the trigger row. `GATE` parks or
 * starts the stream. Value 3 is retired: a reader refuses the byte.
 */
/*
 * A parameter applied in the running state: from `Param::current_value` on
 * a click, within the frame under the sensor's group hold on a camera.
 */
#define NXS_PARAM_KIND_APPLY	0
/*
 * A parameter applied by a run of the configure program: a click's value
 * patched into the bytecode and a VM restart, a camera's mode switch or
 * trigger conversion, for which the device reads BUSY for the whole run.
 */
#define NXS_PARAM_KIND_RUN	1
/* The stream gate: a run that parks or starts the sensor. */
#define NXS_PARAM_KIND_GATE	2

/*
 * TrailerRecord: Record types of the descriptor trailer that follows the
 * bus profiles.
 */
/*
 * The trailer is host-owned data the firmware stores opaquely and
 * serves in a door's description while the personality runs: a count byte,
 * then records of `type u8, len u16, bytes`. Unknown types are kept
 * and skipped. `IDENTITY` names the block in the head every image
 * carries, `CAMERA_HEAD` how a cam personality's head answers, `MODES`
 * carries the index of the `mode` param and each mode's value,
 * `TRIGGERS` the same for the `trigger` param, `PROGRAM` the family's
 * program settings, `STATUS` the status probes a host reads back from
 * the sensor (register, format, decode table), `RUN_PARAMS` the run
 * parameters a host stages in physical units (index, range, default,
 * name, unit), `CAMERA_MAP` the camera law a pod converts a commit's
 * physical units with and stages a run by (the run-value indices, the
 * gain, frame and exposure law, a line time and frame floor per mode),
 * then the mode table the camera door serves; `PARAMS` the parameter
 * rows, which the camera door serves verbatim for a cam personality
 * (`constants/nxs_map.yaml`); `OUTPUTS` the output descriptor rows;
 * `PORTS` the typed ports the personality's device offers;
 * `GRAPH_SOURCE` the declaration a graph image compiles;
 * `GRAPH_BLOCKS` the blocks of a graph image, what
 * runs each one's programs, where it answers, its image and its base
 * staging; `GRAPH_RESOURCES` the ports of its blocks that programs
 * touch; `GRAPH_SCHEDULES` its schedules by name, each the ordered
 * steps of its programs; `GRAPH_PROGRAMS` each block's programs by
 * name, the schedule or the entry that runs each, and the event that
 * runs it; `ENTRIES` the entries a program of the `SCHEDULE` model
 * names;
 * `VALUES` a block's value outputs as its status program writes them;
 * `GRAPH_CONNECTIONS` the connections the solver stages between a
 * graph image's blocks, each value and law from the port that produces
 * to the port that consumes;
 * `PRESENTATION` how the rows read: a display unit and a decimal
 * exponent per `PARAMS` row, then a name per value of the rows that
 * carry names; `MODEL` the model asset a module personality names, its
 * name and SHA-256 in the `ModelField` layout; `INSTANCE_CAL` the
 * calibration a personality takes, its record's layout and size and the
 * names of its procedures in the `InstanceCalField` layout;
 * `VENDOR_BASE`..`VENDOR_BASE + 15` are never interpreted. A new record
 * type, or a field retired from a record, moves the format's minor. A
 * retired record type (`retired: true`: `SHIPPED`) is written by no
 * compiler and skipped by every decoder, and its number stays reserved;
 * the firmware reads no record but `PARAMS`, `ROW_VALUES`, `OUTPUTS`,
 * `PORTS`, `INSTANCE_CAL` and `CAMERA_MAP`, so retiring another moves
 * neither `NXS_MAJOR` nor `NXS_MINOR`.
 */
#define NXS_TRAILER_RECORD_MODES	1
#define NXS_TRAILER_RECORD_CONTROLS	2
#define NXS_TRAILER_RECORD_LAWS	3
#define NXS_TRAILER_RECORD_CAPTURE	4
/*
 * the calibration a personality takes, `INSTANCE_CAL_HEAD_SIZE` bytes in
 * the `InstanceCalField` layout of constants/nxs_map.yaml, then its
 * procedures' names in the order `Cmd::PROC_START` takes them
 */
#define NXS_TRAILER_RECORD_INSTANCE_CAL	5
#define NXS_TRAILER_RECORD_IDENTITY	6
#define NXS_TRAILER_RECORD_PROGRAM	7
#define NXS_TRAILER_RECORD_TRIGGERS	8
#define NXS_TRAILER_RECORD_STATUS	10
/*
 * the VM's run values a host stages for a camera run, in the sensor's
 * units: index, range, default, name, unit. The tool's door; the camera
 * owner converts from the `PARAMS` rows instead
 */
#define NXS_TRAILER_RECORD_RUN_PARAMS	11
/*
 * the camera law a pod converts the `PARAMS` rows with: the mode count and
 * the gain factor, the indices of the VM run values a commit stages, the
 * gain range in capture units, the frame-length capacity, the exposure
 * minimum and margin, each mode's line time and frame-length floor, the
 * mode table, the bounds and the part probe, in the `CameraMapField`,
 * `CameraLawField`, `ModeEntry`, `CameraTailField`, `CameraProbeField` and
 * `CameraPartCodeField` layouts of constants/nxs_map.yaml
 */
#define NXS_TRAILER_RECORD_CAMERA_MAP	12
/*
 * the parameters a personality declares, one `ROW_SIZE` row each in the
 * `RowField` layout of constants/nxs_map.yaml: name, unit, id, type, kind,
 * flags, default, values. The pod door serves every running personality's
 * rows and the camera door the cam personality's, as the record holds them
 */
#define NXS_TRAILER_RECORD_PARAMS	13
/*
 * the output descriptor rows, 64 bytes each, the `OutputField` layout of
 * `constants/nxs_map.yaml`
 */
#define NXS_TRAILER_RECORD_OUTPUTS	14
/*
 * the typed ports a personality's device offers, one 32-byte row each, the
 * `PortField` layout of `constants/nxs_map.yaml`
 */
#define NXS_TRAILER_RECORD_PORTS	15
/*
 * how a camera personality's head answers, `CAMERA_HEAD_SIZE` bytes in the
 * `CameraHeadField` layout of constants/nxs_map.yaml: the register widths,
 * the identity and liveness registers, the default mode and the trigger
 * flags
 */
#define NXS_TRAILER_RECORD_CAMERA_HEAD	17
/*
 * a graph image's source, `GRAPH_SOURCE_SIZE` bytes in the
 * `GraphSourceField` layout of `constants/nxs_map.yaml`: the declaration's
 * digest
 */
#define NXS_TRAILER_RECORD_GRAPH_SOURCE	16
/*
 * the entries a program of the `SCHEDULE` model names, one `ENTRY_ROW_SIZE`
 * row each in the `EntryField` layout: the value that selects the entry and
 * the entry's name; an NXS Hub program and a serializer program carry it
 */
#define NXS_TRAILER_RECORD_ENTRIES	20
/*
 * how the rows read: one `PRESENT_ROW_SIZE` entry per `PARAMS` row in the
 * `PresentField` layout of constants/nxs_map.yaml, then the names of the
 * rows flagged `PRESENT_FLAG_LABELS` in row order, each a count byte and a
 * length-prefixed ASCII name per value of at most `LABEL_CHARS_MAX`
 * characters; the tool prints a row's value from it and knows no device
 */
#define NXS_TRAILER_RECORD_PRESENTATION	21
/*
 * the model asset a module personality names, `MODEL_SIZE` bytes in the
 * `ModelField` layout of constants/nxs_map.yaml: the asset's name and its
 * SHA-256; the switch converges the NXS Module's model partition to the
 * asset of that hash, and an NXS Module refuses a personality whose model
 * is absent
 */
#define NXS_TRAILER_RECORD_MODEL	22
/*
 * a block's value outputs as its status program writes them, one
 * `VALUE_ROW_SIZE` row each in the `ValueField` layout of
 * constants/nxs_map.yaml: the value's name, the program parameter the
 * status program stores it in, its first bit and its width; a program that
 * offers ports carries it
 */
#define NXS_TRAILER_RECORD_VALUES	23
/*
 * The values of one enum row that holds more than 16: the row's index as
 * u16 LE, the value count as u16 LE, then the values as u32 LE in the
 * device's order. A personality carries one per such row, after its
 * `PARAMS`.
 */
#define NXS_TRAILER_RECORD_ROW_VALUES	24
/*
 * a graph image's connections, one `CONNECTION_ROW_SIZE` row each in the
 * `ConnectionField` layout of constants/nxs_map.yaml: the producing block
 * and port, the consuming block and port, and the `ConnectionKind`, a
 * copied value or a law; the executor reads a value through the connection
 * that names it
 */
#define NXS_TRAILER_RECORD_GRAPH_CONNECTIONS	25
/*
 * a graph image's programs, one `PROGRAM_ROW_SIZE` row each in the
 * `ProgramField` layout of constants/nxs_map.yaml: the block that declares
 * each, its name, the schedule or the entry that runs it, and the event
 * that runs it
 */
#define NXS_TRAILER_RECORD_GRAPH_PROGRAMS	26
/*
 * a graph image's blocks, one `BLOCK_ROW_SIZE` row each in the `BlockField`
 * layout of constants/nxs_map.yaml, then each block's base staging in
 * `GraphStageField` rows, in the table's order: its name, the resource it
 * belongs to, what runs its programs, the address it answers at, its image
 * and its build
 */
#define NXS_TRAILER_RECORD_GRAPH_BLOCKS	27
/*
 * a graph image's resources, one `RESOURCE_ROW_SIZE` row each in the
 * `ResourceField` layout of constants/nxs_map.yaml: the ports of its blocks
 * that programs touch, each with its block, its required pin and its
 * readiness value
 */
#define NXS_TRAILER_RECORD_GRAPH_RESOURCES	28
/*
 * a graph image's schedules, a `GraphSchedulesField` head, then each
 * schedule's `ScheduleField` head and its steps in the `GraphStepField`
 * layout of constants/nxs_map.yaml: the schedule's name and the ordered
 * program entries the executor runs by its index
 */
#define NXS_TRAILER_RECORD_GRAPH_SCHEDULES	29
#define NXS_TRAILER_RECORD_VENDOR_BASE	224

/*
 * TriggerCode: The values of a camera's `TRIGGER` row: the trigger
 * conversions a head serves, by their meaning, so a driver names them
 * without the trailer.
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
 * The constants of the map a driver reads: the identity bytes, the sizes
 * and limits, the row and mode bits, the apply and busy contract.
 */
/* `MapReg::MAP_ID` at every door of every device. */
#define NXS_MAP_ID_VALUE	0xC4
/*
 * The release this device runs, served at `RELEASE`: MAJOR, MINOR and PATCH
 * one byte each from the low byte up, then 0, the product's `VERSION` file.
 */
#define NXS_RELEASE_VALUE	769
/* Bytes a read latches at its first byte, the header. */
#define NXS_MAP_LATCH_SIZE	256
/* Bytes the `DESCRIPTION` window serves per `DESC_PAGE`. */
#define NXS_DESC_PAGE_SIZE	32768
/* Bytes of one `REGION_TABLE` entry. */
#define NXS_REGION_ENTRY_SIZE	8
/* Entries `REGION_TABLE` holds. */
#define NXS_REGION_TABLE_MAX	16
/* `RegionField::FLAGS` bit 0: the region takes writes. */
#define NXS_REGION_FLAG_WRITABLE	0x01
/* Arrays of the `CELLS` region, `LIVE` through `STAGED`. */
#define NXS_CELLS_ARRAYS	5
/* `MapReg::COMMIT` value that applies the staged set. */
#define NXS_COMMIT_APPLY	1
/*
 * `MapReg::COMMIT` value that clears the staged set and changes nothing
 * else.
 */
#define NXS_COMMIT_DISCARD	2
/* Rows the layout allows a device to declare in `ROW_CAPACITY`. */
#define NXS_ROW_CAPACITY_MAX	1024
/* Rows a pod's `CELLS` region holds, its `ROW_CAPACITY`. */
#define NXS_POD_ROW_CAPACITY	64
/* Bytes of one parameter or output row. */
#define NXS_ROW_SIZE	64
#define NXS_ROW_NAME_LEN	16
#define NXS_ROW_UNIT_LEN	6
/* Enum values a row holds as u16. */
#define NXS_ROW_VALUES_MAX	16
/*
 * The bits a `ParamId` shifts down to its `ParamFamily` byte; its low byte
 * is the id within the family.
 */
#define NXS_PARAM_FAMILY_SHIFT	8
#define NXS_ROW_WIDE_VALUES_MAX	8
#define NXS_ROW_TYPE_RANGE	0x01
#define NXS_ROW_TYPE_SIGNED	0x02
#define NXS_ROW_TYPE_WIDE	0x04
/*
 * The enum's values are in the personality's `ROW_VALUES` record, not
 * inline; `RowField::COUNT` reads 0.
 */
#define NXS_ROW_TYPE_LIST	0x08
#define NXS_ROW_FLAG_READ_ONLY	0x01
/*
 * The graph's owner sets the row from the rig's facts, a line time for one;
 * a kernel driver offers it read-only.
 */
#define NXS_ROW_FLAG_GRAPH	0x04
/*
 * The owner tag in `RowField::FLAGS` bits 5..7: the index of the trailer
 * the row belongs to, the pod's own rows carrying the last trailer's index.
 */
#define NXS_ROW_FLAG_OWNER	0xE0
#define NXS_MODE_TABLE_MAX	16
#define NXS_MODE_ENTRY_SIZE	16
/* Bytes of the `CAMERA_MAP` record's head, the first law row's offset. */
#define NXS_CAMERA_MAP_HEAD_SIZE	31
/* Bytes of one law row of the `CAMERA_MAP` record. */
#define NXS_CAMERA_LAW_ROW_SIZE	8
/*
 * Bytes of the `CAMERA_MAP` record's tail before its part probe: the bounds
 * and the code count.
 */
#define NXS_CAMERA_MAP_TAIL_SIZE	7
/* Bytes of the part probe before its codes. */
#define NXS_CAMERA_PROBE_SIZE	22
/* Bytes of one code of the part probe. */
#define NXS_CAMERA_PART_CODE_SIZE	5
/* Codes the part probe of a CAMERA_MAP record names at most. */
#define NXS_PART_CODES_MAX	8
#define NXS_MODE_FLAG_DEFAULT	0x01
#define NXS_MODE_FLAG_TRIGGERABLE	0x02
#define NXS_FLIP_X	0x01
#define NXS_FLIP_Y	0x02
/*
 * The `LIVE` a read-only row may serve before its first reading, the one
 * value outside its range: an unsigned read-only row keeps its `MAX` below
 * it.
 */
#define NXS_ROW_UNREAD	0x80000000
/*
 * How late past its bound a run can still be on the device's bus, the
 * transfer in flight at the deadline, a 128-byte burst at 400 kHz rounded
 * up. The device adds it to the longest bound of the staged rows' kinds
 * when it paints `RUN_MS`, so a host reads `STATE` once after the painted
 * time.
 */
#define NXS_BOUND_LATENESS_MS	5

/*
 * The device's errno numbering, what the ERROR cell carries after a commit
 * ended FAULT (Interface Description §12). Six differ from Linux's:
 * ETIMEDOUT, ESTALE, ENOTSUP, EILSEQ, ECANCELED and EBADMSG. A driver
 * translates by name.
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

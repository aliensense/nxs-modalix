# Changelog

All notable changes to the NXS camera package for the SiMa.ai Modalix: the modules, the kernel, the udev rules, the Fast DDS profile and the package itself. The format follows Keep a Changelog; a version is the SiMa release the package is built for and the NXS version, `<release>+nxs<X.Y.Z>`. The release section of a tag is `[<release>+nxs<X.Y.Z>]`, dated when the tag is cut.

## [Unreleased]

### Added
- A passive `nxs-csi2` node takes its parameter rows from the device-tree property `aliensense,rows`, one tuple of a mode index, a row id, a minimum, a maximum, a step and a default per row of a mode. It offers EXPOSURE, VBLANK, ANALOGUE_GAIN, the black level and the line time on the rows of the mode in effect, converted to lines as a camera door's are. A set stores the value and transfers nothing. A passive node without the property offers the line's and the bus's controls alone, as before.
- A passive `nxs-csi2` node whose rows name the exposure and the line offers its owner three integer controls in one cluster: `Owner Line Time` in nanoseconds, `Owner Exposure Min` and `Owner Exposure Max` in microseconds, ids `NXS_CSI2_CID_BASE` plus 3, 4 and 5 (`0x00982903` to `0x00982905`). The owner writes them in a `VIDIOC_S_EXT_CTRLS` of their own. A set takes the line as the line in effect and the bounds as the exposure row's range, repaints the controls' ranges in lines of that line and `HBLANK`, and transfers nothing. The line row's control reads the painted line on a range of that one value, and a paint activates no line row the mode does not list. Each paint and each format set that moves the line raises one event on the line row that carries the line as its value and its range. A minimum above the maximum answers `EINVAL` to `VIDIOC_TRY_EXT_CTRLS` and `VIDIOC_S_EXT_CTRLS` alike. `Owner Line Time` runs from the mode's shortest line, its active pixels at its pixel rate, to one second, so a shorter paint reads that line. Before a paint the three read the line in effect and the tree's bounds for the mode in effect, and a format set of another mode returns that mode's tree rows. `Owner Line Time`'s subscriber hears the line in effect once, with a range event where the mode's shortest line moves, and a bound raises an event only where it moved. Where the mode's shortest line rises above `Owner Line Time`, the new line is written before the minimum rises. A camera door offers none of them.

### Changed
- A passive `nxs-csi2` node follows its format. An active format set selects the first mode of `aliensense,modes` of the asked size and code, takes that mode's rows of `aliensense,rows` and repaints the controls' ranges, `PIXEL_RATE`, `HBLANK` and `LINK_FREQ`. The black level's range follows the mode, and a row the mode does not list reads inactive, with its flags event. The line time's control reads the line in effect, never shorter than the mode's active pixels at its pixel rate, on a range of that one value, and is active only where the mode lists it. A size and code no mode has keeps the mode, a try reports the mode in effect, and a set of another mode while the node streams answers `EBUSY`. A passive node used to serve its first mode whatever format was set.
- The package's repository holds the camera door modules' sources, their tests and their device-tree bindings.
- The camera door modules of `nxs-csi2` 1.3.0 read a camera door of the NXS Device Interface. The header is one latched read of 256 bytes, its region table names the `CELLS` and `DESCRIPTION` regions, and the description reads through the paged window between two reads of `DESCRIPTOR_EPOCH`. The rows, their `ROW_VALUES` lists in any record order and the mode table come from the description, and row indices are u16. The core takes `MAP_ID` 0xC4, `DOOR` 2 and a release of MAJOR 1 with a MINOR at or below 3.
- `nxs-csi2-core` refuses an adapter that cannot read the 256 header bytes at once, with `EOPNOTSUPP`. A `DESC_PAGE` another host left on a later page is set to 0 before the read, and `DESC_PAGE` ends at 0 after an error as well.
- `nxs-csi2-core` commits without a mask. It reads `STATE` and `DESCRIPTOR_EPOCH` in one header read, then the rows' `STEP` cells. It writes the `STAGED` cells of the rows that take a value, reads `RUN_MS`, writes `COMMIT` 1 and reads `STATE` once after `RUN_MS`. A staging write or a `RUN_MS` read the bus cut ends with `COMMIT` 2. A `RUN_MS` of 0 ends with `COMMIT` 2 and `EPROTO`, and a moved epoch ends the commit with `ESTALE`.
- A row whose `STEP` cell reads 0 takes no value in the state in effect. Its control turns inactive, and a commit leaves the row out. The device's `STEP` replaces the drivers' own reading of the trigger for the rate and the exposure.
- The test pattern menu offers the first 256 codes of a longer list.
- A row's kind is APPLY, RUN or GATE. The trigger row commits as a RUN, and a row whose `KIND` reads 3 takes no control.
- The module version of `nxs-csi2-core` names the interface layout it reads and the release it was built from, `layout 1.3, release 1.3.0`.
- `nxs-csi2-core` probes a door once it serves a description and a personality, `DESC_SIZE` and `PERSONALITY_COUNT`, and takes the mode count from the `CAMERA_MAP` record. It reads the stream and the mode in the `LIVE` cells of the `STREAM` and `MODE` rows, since the header names no payload.
- A row's id is a family byte above an id within the family. The core knows the generic family's stream, mode and temperature and the camera family's ids. A row of another family is custom.
- The probe line counts the modes in the singular for one, `1 mode`, and `4 modes` for more.
- The core takes `DOOR` 2 as a driver door and names a device by its class: an NXS Pod, an NXS Camera or an NXS Module.
- `nxs-csi2-map.h` names the `CELLS` region's arrays by index and lays out the `CAMERA_MAP` record, its head, law rows, mode table, bounds and part probe. It carries no address past the header, since the core reads every region's base from the region table. Its errno note names the six device values that differ from Linux's, `EBADMSG` among them.
- `nxs-csi2-map.h` says a driver door serves the serial alone of the `DEVICE` region. An NXS Pod's `RUN_MS` reads 9 ms with nothing staged, and its main door powers up READY.
- `nxs-csi2-map.h` names `PROBE` at `0x0031`, the verdict of the personality's probe, which the drivers do not read.
- `nxs_csi2_core_run_mode()` takes no park flag. A RUN leaves the gate as it finds it, so the sensor stays in standby after the mode's run until a GATE 1.
- `nxs-csi2-core` reads a mode's depth from its CSI-2 data type code, `DATA_TYPE`. A mode whose code is not a RAW code of the map header has no media bus code: the core logs it with the code in hex and leaves it out. `nxs-csi2-map.h` has no depth byte: byte 4 of a mode row is reserved and reads 0, and the header names the data type codes with the bits of a sample and of a pixel. `nxs-csi2` divides the lane rate by the pixel bits of the mode's code for `PIXEL_RATE`.
- The driver's log lines and the documents name the NXS Pod and the NXS Hub by their full names.

### Fixed
- The `nxs-csi2` exposure, vertical blanking and horizontal blanking ranges end at 2147483647, the largest value of a 32-bit control, on a camera door and a passive node alike. A line of more than about 1.8 s, a line under 1 us with bounds of about 30 min, or a line under about 466 ns with a rate as slow as 1 mfps used to give `EXPOSURE`, `VBLANK` or `HBLANK` a bound past it and a negative value.
- The `nxs-csi2` door times the line from the row of id `LINE_TIME_NS`, whatever the device names it. A line row with no reading yet times the line at its default.
- A re-read of the door puts each row control back on a row of its id and shape, a custom row's on its name too. A row of another shape, a menu that would outgrow its values among them, takes a fresh control at the next bind.
- VFLIP leaves the FLIPS row at a re-read together with HFLIP and comes back to it. A flip set after a re-read into a larger rows array stages the FLIPS row, never freed memory.
- An enum row's inline values read as u16, unsigned in a signed row too, as the device and the host tool read them. A value of 0x8000 or above commits as the row lists it, never sign-extended.
- A re-read keeps a 32-bit range control off a row whose `MAX` now passes 2147483647. The row takes a 64-bit control at the next bind, so no value above that limit is clamped away.
- A read-only row that serves `ROW_UNREAD` has no reading yet. Its control reads inactive and keeps its value until the first reading, instead of reading -2147483648.
- A format read, a format set or a stream start on a camera door no longer blocks for good after another personality was uploaded into the NXS Pod behind it. The door re-reads the changed description under the lock the operation already holds.

## [2.1.3+nxs1.3.0] - 2026-11-04

### Changed
- `nxs-csi2-core` gates the camera map on its `RELEASE` cell, four bytes at `0x0026` naming the release as MAJOR, MINOR and PATCH, and its probe line names that release. The bytes at `0x0001` and `0x0019` are reserved and read 0.
- `nxs-csi2-core` stages a commit's rows and `MASK` in one write, reads the `RUN_MS` the map paints for that mask at `0x0010`, writes `COMMIT` alone and reads `STATE` once after the painted time; BUSY then is the device's failure. The map header carries `NXS_MAP_REG_RUN_MS` in place of the four per-kind bound cells.
- `nxs-csi2-core` reads a changed camera map again at a session's start on every host and re-points the row controls at the rows of their names. A read the host tool's bring-up tore at probe no longer stands until a reboot, and the Jetson adapter no longer asks for a module reload.

### Added
- The first package for SiMa.ai Palette 2.1.3 on eLxr 12, kernel `6.18.3-modalix`: `nxs-modalix_2.1.3_arm64.deb`, version `2.1.3+nxs1.3.0`.
- The camera door modules of `nxs-drivers` 1.3.0, `nxs-csi2-core`, `nxs-csi2` and `nxs-hub-route`, prebuilt against SiMa's headers package, with their DKMS source under `/usr/src/nxs-csi2-1.3.0/`. They load in place of a DKMS build of the same driver.
- SiMa's kernel of release 2.1.3 rebuilt with three fixes of the camera receiver: a capture node of virtual channel 1 to 3 validates its link, the capture and scratch buffers carry a DMA guard, and a capture node sets its own channel's format. The package installs it into U-Boot's boot slot the running kernel did not boot from, keeps SiMa's Image of that slot as `Image.sima`, points `boot_path` at the slot with `fw_setenv`, and prints the slot and the overlays `dtbos` names. SiMa's kernel stays in the other slot.
- The removal puts SiMa's Image back into the slot and points `boot_path` at SiMa's slot.
- The udev rules: the camera connectors' buses as `/dev/i2c-cam0` and `/dev/i2c-cam1`, the capture nodes, media graphs and DMA heaps for the `video` group, and the capture buffers' DMA guard at 32 MiB, so four 1080p channels fit the CMA pool.
- The Fast DDS profile of the camera nodes, `/etc/nxs-modalix/fastdds.xml`: shared memory with a 32 MiB segment, five 1080p RGB images, and UDPv4 for other hosts. With the camera nodes and four image viewers under one user it carries each colour link at about 22 Hz, where UDPv4 alone carries 18.5 Hz.
- The logind drop-in `/etc/systemd/logind.conf.d/nxs-modalix.conf`, `RemoveIPC=no`. logind deletes a user's shared memory when their last session ends, and the camera nodes run as the operator outside any session. The drop-in takes effect at the next boot.
- The package refuses another SiMa release, a host without kernel `6.18.3-modalix` and an image without U-Boot's two boot slots, and ships the kernel's corresponding source under `/usr/src/nxs-modalix-2.1.3/`.

### Changed
- The `nxs-csi2` door finds the graph's line-time row by its id, `NXS_PARAM_ID_LINE_TIME_NS` 10, and offers it as a read-only control. The map header carries the renumbered ids of release 1.3.

### Fixed
- The `nxs-csi2` door reads its camera map again at the next format ask or stream start when a personality uploaded into the pod changed it, so `nxs compose` takes effect with no reboot. The door used to keep the map it had read first.

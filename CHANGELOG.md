# Changelog

All notable changes to the NXS camera package for the SiMa.ai Modalix: the modules, the kernel, the udev rules, the Fast DDS profile and the package itself. The format follows Keep a Changelog; a version is the SiMa release the package is built for and the NXS version, `<release>+nxs<X.Y.Z>`. The release section of a tag is `[<release>+nxs<X.Y.Z>]`, dated when the tag is cut.

## [Unreleased]

### Added
- The first package for SiMa.ai Palette 2.1.3 on eLxr 12, kernel `6.18.3-modalix`: `nxs-modalix_2.1.3_arm64.deb`, version `2.1.3+nxs1.3.0`.
- The camera door modules of `nxs-drivers` 1.3.0, `nxs-csi2-core`, `nxs-csi2` and `nxs-hub-route`, prebuilt against SiMa's headers package, with their DKMS source under `/usr/src/nxs-csi2-1.3.0/`. They load in place of a DKMS build of the same driver.
- SiMa's kernel of release 2.1.3 rebuilt with three fixes of the camera receiver: a capture node of virtual channel 1 to 3 validates its link, the capture and scratch buffers carry a DMA guard, and a capture node sets its own channel's format. The package installs it into U-Boot's boot slot the running kernel did not boot from, keeps SiMa's Image of that slot as `Image.sima`, points `boot_path` at the slot with `fw_setenv`, and prints the slot and the overlays `dtbos` names. SiMa's kernel stays in the other slot.
- The removal puts SiMa's Image back into the slot and points `boot_path` at SiMa's slot.
- The udev rules: the camera connectors' buses as `/dev/i2c-cam0` and `/dev/i2c-cam1`, the capture nodes, media graphs and DMA heaps for the `video` group, and the capture buffers' DMA guard at 32 MiB, so four 1080p channels fit the CMA pool.
- The Fast DDS profile of the camera nodes, `/etc/nxs-modalix/fastdds.xml`: shared memory with a 32 MiB segment, five 1080p RGB images, and UDPv4 for other hosts. With the camera nodes and four image viewers under one user it carries each colour link at about 22 Hz, where UDPv4 alone carries 18.5 Hz.
- The logind drop-in `/etc/systemd/logind.conf.d/nxs-modalix.conf`, `RemoveIPC=no`. logind deletes a user's shared memory when their last session ends, and the camera nodes run as the operator outside any session. The drop-in takes effect at the next boot.
- The package refuses another SiMa release, a host without kernel `6.18.3-modalix` and an image without U-Boot's two boot slots, and ships the kernel's corresponding source under `/usr/src/nxs-modalix-2.1.3/`.

### Fixed
- The `nxs-csi2` door reads its camera map again at the next format ask or stream start when a personality uploaded into the pod changed it, so `nxs compose` takes effect with no reboot. The door used to keep the map it had read first.

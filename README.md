# nxs-modalix

The kernel side of the NXS camera stack on a SiMa.ai Modalix: the camera door modules, a rebuild of SiMa's kernel with three fixes of its camera receiver, the udev rules and the Fast DDS profile of the camera nodes. One branch per SiMa software release, `sima-2.1.3` for Palette 2.1.3 on eLxr 12 with kernel `6.18.3-modalix`. GPL-2.0.

## Install

Take the package for the board's SiMa release from the release page and install it:

```sh
sudo apt install ./nxs-modalix_2.1.3_arm64.deb
```

It installs:

- `nxs-csi2-core.ko`, `nxs-csi2.ko` and `nxs-hub-route.ko` under `/lib/modules/6.18.3-modalix/updates/drivers/media/i2c/`, with their DKMS source under `/usr/src/nxs-csi2-1.3.0/`. A depmod override makes them load in place of a DKMS build of the same driver.
- The kernel under `/usr/lib/nxs-modalix/`, then into U-Boot's boot slot the running kernel did not boot from, `/boot/boot-1/` on a fresh image. SiMa's Image of that slot stays beside it as `Image.sima`, and the other slot keeps SiMa's kernel. The package points U-Boot's `boot_path` at its slot with `fw_setenv` and prints the slot and the overlays `dtbos` names.
- The udev rules: the camera connectors' buses as `/dev/i2c-cam0` and `/dev/i2c-cam1`, the capture nodes, the media graphs and the DMA heaps for the `video` group, and the capture buffers' DMA guard at 32 MiB.
- The Fast DDS profile of the camera nodes, `/etc/nxs-modalix/fastdds.xml`.
- The logind drop-in `/etc/systemd/logind.conf.d/nxs-modalix.conf`, `RemoveIPC=no`, so the nodes' shared memory outlives the operator's login session.
- The corresponding source of the kernel under `/usr/src/nxs-modalix-2.1.3/`: the patches, the builder, the container recipe and the manifest that names SiMa's tree.

The overlays of the camera ports are the `nxs` host tool's to write, into both boot slots. Removing the package puts SiMa's Image back into its slot and points U-Boot at SiMa's slot again.

## Build

The builder applies `kernel/patches/` onto SiMa's public tree at the commit `MANIFEST.md` names, configures it with SiMa's own configuration, and builds the modules against SiMa's headers package, inside the container from `docker/`:

```sh
docker volume create nxs-modalix-work
docker build --platform linux/arm64 -t nxs-modalix-build docker/
docker run --rm --platform linux/arm64 -v nxs-modalix-work:/build -v "$PWD":/repo nxs-modalix-build /repo/build/builder.sh -P -k -m -c -b
ls output/
```

`output/nxs-modalix_<release>_arm64.deb` is the package. `builder.sh -h` lists the stages. `build/test-kernel-slot.sh` checks the slot install on a boot partition under a temporary root. `build/test-package.sh output/nxs-modalix_<release>_arm64.deb`, run as root in the build container, installs, reinstalls and removes the package on a stand-in Modalix.

## Layout

- `kernel/patches/`: the three fixes, each a patch on SiMa's tree with its reason. A capture node of a virtual channel past the first validates on its own stream. The capture buffers and the DMA's scratch buffer carry a guard against a block that runs past a lost frame end. A capture node sets its own channel's format.
- `nxs-drivers/`: the camera door modules, vendored byte for byte from the public `nxs-drivers` repository.
- `build/kernel-slot`: the install of the kernel into a boot slot, its removal and its status.
- `rootfs/`: the files the package installs under `/etc`.
- `debian/`: the package's maintainer scripts.
- `.github/workflows/build.yml`: the release build. A tag named after the SiMa release (`2.1.3`) builds the package on this repository's arm64 runners and publishes it with the source tarball and `SHA256SUMS.txt`.
- `MANIFEST.md`: SiMa's tree and packages, by URL and checksum, and the build record.

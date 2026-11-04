#!/usr/bin/env bash
# The built package on a stand-in Modalix: an arm64 Debian 12 container with
# the kernel's module tree, U-Boot's two boot slots holding SiMa's kernel,
# and fw_printenv and fw_setenv over a file. The install puts the kernel into
# slot 1 and points boot_path at it, the modules into the kernel's tree with
# their depmod override, and the rules and the profile under /etc. The
# removal puts SiMa's kernel back into slot 1 and boots slot 0.
#
#   build/test-package.sh output/nxs-modalix_2.1.3_arm64.deb    (inside the build container)
set -euo pipefail

DEB="${1:?the package to install}"
KERNEL="6.18.3-modalix"
SIMA_BANNER="Linux version ${KERNEL} (root@elxr-builder) (aarch64-linux-gnu-gcc (Debian 12.2.0-14) 12.2.0) #1 SMP PREEMPT"
ENV_FILE=/boot/uboot-env.txt

fail() {
    echo "FAIL: $*" >&2
    exit 1
}

[ "$(id -u)" = 0 ] || fail "run as root, in a throwaway container"
command -v udevadm >/dev/null || { apt-get update -qq >/dev/null; apt-get install -y -qq udev kmod >/dev/null; }

# The stand-in board.
mkdir -p "/lib/modules/${KERNEL}/kernel" /boot/boot-0 /boot/boot-1
for slot in boot-0 boot-1; do
    { head -c 4096 /dev/zero; printf '%s\n' "${SIMA_BANNER}"; head -c 4096 /dev/zero; } > "/boot/${slot}/Image"
done
echo dtbo > /boot/boot-0/waveshare-carrier-som.dtbo
cp /boot/boot-0/waveshare-carrier-som.dtbo /boot/boot-1/
printf 'boot_path=/boot-0/\nboot_prefixes=/boot-0/ /boot-1/\ndtbos=waveshare-carrier-som.dtbo\n' > "${ENV_FILE}"
cat > /usr/local/bin/fw_printenv <<EOF
#!/bin/sh
[ "\$1" = "-n" ] && shift
line="\$(grep "^\$1=" ${ENV_FILE} | tail -n 1)" || { echo "## Error: \"\$1\" not defined" >&2; exit 1; }
printf '%s\n' "\${line#*=}"
EOF
cat > /usr/local/bin/fw_setenv <<EOF
#!/bin/sh
name="\$1"
shift
grep -v "^\$name=" ${ENV_FILE} > ${ENV_FILE}.new || true
[ \$# -gt 0 ] && printf '%s=%s\n' "\$name" "\$*" >> ${ENV_FILE}.new
mv ${ENV_FILE}.new ${ENV_FILE}
EOF
chmod +x /usr/local/bin/fw_printenv /usr/local/bin/fw_setenv

echo "== install"
dpkg -i "${DEB}"
dpkg -s nxs-modalix | grep -E '^(Package|Version|Status):'
grep -a -q -F "(nxs-modalix@" /boot/boot-1/Image || fail "slot 1 holds no package kernel"
grep -a -q -F "${SIMA_BANNER}" /boot/boot-1/Image.sima || fail "slot 1 kept no SiMa Image"
grep -a -q -F "${SIMA_BANNER}" /boot/boot-0/Image || fail "slot 0 lost SiMa's kernel"
[ "$(fw_printenv -n boot_path)" = "/boot-1/" ] || fail "boot_path is $(fw_printenv -n boot_path)"
for module in nxs-csi2-core nxs-csi2 nxs-hub-route; do
    path="$(modinfo -k "${KERNEL}" -n "${module}")"
    [ "${path}" = "/lib/modules/${KERNEL}/updates/drivers/media/i2c/${module}.ko" ] || fail "${module} resolves to ${path}"
done
[ "$(modinfo -k "${KERNEL}" -F vermagic nxs-hub-route)" = "${KERNEL} SMP preempt mod_unload aarch64" ] || fail "vermagic"
modinfo -k "${KERNEL}" -F depends nxs-csi2 | grep -q nxs-csi2-core || fail "nxs-csi2 does not depend on its core"
for file in /etc/udev/rules.d/99-nxs-modalix.rules /etc/depmod.d/nxs-modalix.conf /etc/nxs-modalix/fastdds.xml \
            /usr/src/nxs-csi2-1.3.0/dkms.conf /usr/src/nxs-modalix-2.1.3/kernel/patches \
            /usr/lib/nxs-modalix/kernel-slot /usr/share/doc/nxs-modalix/copyright; do
    [ -e "${file}" ] || fail "${file} is missing"
done
/usr/lib/nxs-modalix/kernel-slot status

echo "== a DKMS build of the same driver beside it"
mkdir -p "/lib/modules/${KERNEL}/updates/dkms"
cp "/lib/modules/${KERNEL}/updates/drivers/media/i2c/nxs-csi2.ko" "/lib/modules/${KERNEL}/updates/dkms/"
depmod -a "${KERNEL}"
[ "$(modinfo -k "${KERNEL}" -n nxs-csi2)" = "/lib/modules/${KERNEL}/updates/drivers/media/i2c/nxs-csi2.ko" ] \
    || fail "the DKMS build wins over the package's module"
rm -rf "/lib/modules/${KERNEL}/updates/dkms"

echo "== reinstall"
dpkg -i "${DEB}"
grep -a -q -F "${SIMA_BANNER}" /boot/boot-1/Image.sima || fail "the reinstall lost SiMa's copy"

echo "== remove"
dpkg -r nxs-modalix
grep -a -q -F "${SIMA_BANNER}" /boot/boot-1/Image || fail "slot 1 holds no SiMa kernel after the removal"
[ ! -e /boot/boot-1/Image.sima ] || fail "Image.sima stayed"
[ "$(fw_printenv -n boot_path)" = "/boot-0/" ] || fail "boot_path is $(fw_printenv -n boot_path) after the removal"
[ ! -e "/lib/modules/${KERNEL}/updates/drivers/media/i2c/nxs-csi2.ko" ] || fail "a module stayed"
dpkg -P nxs-modalix
[ ! -e /var/lib/nxs-modalix ] || fail "the purge left /var/lib/nxs-modalix"
echo "package: all checks pass"

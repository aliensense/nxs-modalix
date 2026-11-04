#!/bin/sh
# kernel-slot on a Modalix boot partition under a temporary root: a fresh
# image's install, the reinstall from the package's own kernel, the removal,
# an environment fw_setenv does not reach, a kernel booted from the second
# slot, and an image without the slots. U-Boot's environment is a file the
# stand-ins of fw_printenv and fw_setenv read and write.
#
#   build/test-kernel-slot.sh          runs under /bin/sh (dash on Debian)
set -eu

HERE="$(cd "$(dirname "$0")" && pwd)"
SLOT="$HERE/kernel-slot"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
SIMA_BANNER="Linux version 6.18.3-modalix (root@elxr-builder) (aarch64-linux-gnu-gcc (Debian 12.2.0-14) 12.2.0) #1 SMP PREEMPT Tue Aug 19 18:38:00 UTC 2026"
NXS_BANNER="Linux version 6.18.3-modalix (nxs-modalix@sima-2.1.3) (gcc (Debian 12.2.0-14+deb12u1) 12.2.0) #1 SMP PREEMPT Tue Sep 22 12:10:32 UTC 2026"
BENCH_BANNER="Linux version 6.18.3-modalix (root@bench) (gcc (Debian 12.2.0-14+deb12u1) 12.2.0) #5 SMP PREEMPT Thu Oct 29 08:09:30 UTC 2026"
failures=0

fail() {
    echo "FAIL: $*"
    failures=$((failures + 1))
}

expect_eq() {
    [ "$2" = "$3" ] || fail "$1: '$2', expected '$3'"
}

# An Image: a banner inside some bytes.
image() {
    mkdir -p "$(dirname "$1")"
    { head -c 4096 /dev/zero; printf '%s\n' "$2"; head -c 4096 /dev/zero; } > "$1"
}

stub_tools() {
    mkdir -p "$ROOT/bin"
    cat > "$ROOT/bin/fw_printenv" <<'EOF'
#!/bin/sh
[ "$1" = "-n" ] && shift
line="$(grep "^$1=" "$NXS_MODALIX_ROOT/uboot.env" | tail -n 1)" || { echo "## Error: \"$1\" not defined" >&2; exit 1; }
printf '%s\n' "${line#*=}"
EOF
    cat > "$ROOT/bin/fw_setenv" <<'EOF'
#!/bin/sh
name="$1"
shift
grep -v "^$name=" "$NXS_MODALIX_ROOT/uboot.env" > "$NXS_MODALIX_ROOT/uboot.env.new" || true
[ $# -gt 0 ] && printf '%s=%s\n' "$name" "$*" >> "$NXS_MODALIX_ROOT/uboot.env.new"
mv "$NXS_MODALIX_ROOT/uboot.env.new" "$NXS_MODALIX_ROOT/uboot.env"
EOF
    chmod +x "$ROOT/bin/fw_printenv" "$ROOT/bin/fw_setenv"
}

# A fresh Modalix image: SiMa's kernel in both slots, slot 0 booted, the bus overlay in slot 0 alone.
fresh() {
    ROOT="$WORK/$1"
    export NXS_MODALIX_ROOT="$ROOT"
    image "$ROOT/boot/boot-0/Image" "$SIMA_BANNER"
    image "$ROOT/boot/boot-1/Image" "$SIMA_BANNER"
    image "$ROOT/usr/lib/nxs-modalix/Image" "$NXS_BANNER"
    echo dtbo > "$ROOT/boot/boot-0/nxs-cam-buses.dtbo"
    mkdir -p "$ROOT/proc"
    printf '%s\n' "$SIMA_BANNER" > "$ROOT/proc/version"
    printf 'boot_path=/boot-0/\nboot_prefixes=/boot-0/ /boot-1/\ndtbos=nxs-cam-buses.dtbo\n' > "$ROOT/uboot.env"
    stub_tools
}

run() {
    PATH="$ROOT/bin:$PATH" "$SLOT" "$@"
}

env_of() {
    grep "^$1=" "$ROOT/uboot.env" | tail -n 1 | cut -d= -f2-
}

has_banner() {
    grep -a -q -F "$2" "$1"
}

# A fresh image: the package owns slot 1, keeps SiMa's Image there, and boots it next.
fresh install
out="$(run install)"
has_banner "$ROOT/boot/boot-1/Image" "$NXS_BANNER" || fail "install: slot 1 holds no package kernel"
has_banner "$ROOT/boot/boot-1/Image.sima" "$SIMA_BANNER" || fail "install: slot 1 kept no SiMa Image"
has_banner "$ROOT/boot/boot-0/Image" "$SIMA_BANNER" || fail "install: slot 0 lost SiMa's kernel"
[ -f "$ROOT/boot/boot-1/nxs-cam-buses.dtbo" ] || fail "install: the bus overlay is not in slot 1"
expect_eq "install: boot_path" "$(env_of boot_path)" "/boot-1/"
expect_eq "install: boot_prefixes" "$(env_of boot_prefixes)" "/boot-1/ /boot-0/"
expect_eq "install: state" "$(cat "$ROOT/var/lib/nxs-modalix/kernel-slot")" "boot-1"
expect_eq "install: lines" "$out" "nxs-modalix: kernel Linux version 6.18.3-modalix (nxs-modalix@sima-2.1.3) in /boot/boot-1, SiMa's Linux version 6.18.3-modalix (root@elxr-builder) kept in /boot/boot-0
nxs-modalix: dtbos: nxs-cam-buses.dtbo, each in /boot/boot-1
nxs-modalix: U-Boot boots /boot-1/ from the next boot (boot_path /boot-1/, boot_prefixes /boot-1/ /boot-0/)
nxs-modalix: reboot to run it: sudo reboot"

# After the reboot into it, a reinstall replaces the kernel in the same slot and keeps SiMa's copy.
printf '%s\n' "$NXS_BANNER" > "$ROOT/proc/version"
image "$ROOT/usr/lib/nxs-modalix/Image" "$NXS_BANNER rebuilt"
run install > /dev/null
has_banner "$ROOT/boot/boot-1/Image" "$NXS_BANNER rebuilt" || fail "reinstall: slot 1 holds the old package kernel"
has_banner "$ROOT/boot/boot-1/Image.sima" "$SIMA_BANNER" || fail "reinstall: SiMa's copy changed"
has_banner "$ROOT/boot/boot-0/Image" "$SIMA_BANNER" || fail "reinstall: slot 0 lost SiMa's kernel"
expect_eq "reinstall: boot_path" "$(env_of boot_path)" "/boot-1/"

# The status names both slots, the running one and the owner.
status="$(run status)"
expect_eq "status" "$status" "/boot/boot-0: Linux version 6.18.3-modalix (root@elxr-builder)
/boot/boot-1: Linux version 6.18.3-modalix (nxs-modalix@sima-2.1.3)
running: /boot/boot-1: $NXS_BANNER
owned by nxs-modalix: /boot/boot-1
boot_path: /boot-1/
boot_prefixes: /boot-1/ /boot-0/
dtbos: nxs-cam-buses.dtbo"

# The removal puts SiMa's kernel back into slot 1 and boots slot 0.
out="$(run remove)"
has_banner "$ROOT/boot/boot-1/Image" "$SIMA_BANNER" || fail "remove: slot 1 holds no SiMa kernel"
[ ! -f "$ROOT/boot/boot-1/Image.sima" ] || fail "remove: Image.sima stayed"
[ ! -f "$ROOT/var/lib/nxs-modalix/kernel-slot" ] || fail "remove: the state stayed"
expect_eq "remove: boot_path" "$(env_of boot_path)" "/boot-0/"
expect_eq "remove: boot_prefixes" "$(env_of boot_prefixes)" "/boot-0/ /boot-1/"
expect_eq "remove: lines" "$out" "nxs-modalix: /boot/boot-1 holds Linux version 6.18.3-modalix (root@elxr-builder) again
nxs-modalix: U-Boot boots /boot-0/ from the next boot (boot_path /boot-0/, boot_prefixes /boot-0/ /boot-1/)"
expect_eq "remove twice" "$(run remove)" ""

# Without fw_setenv the slot is filled and the U-Boot prompt's line printed.
fresh noenv
rm "$ROOT/bin/fw_setenv"
out="$(run install)"
has_banner "$ROOT/boot/boot-1/Image" "$NXS_BANNER" || fail "noenv: slot 1 holds no package kernel"
expect_eq "noenv: boot_path" "$(env_of boot_path)" "/boot-0/"
case "$out" in
    *"at U-Boot's prompt: env set boot_path /boot-1/; env set boot_prefixes \"/boot-1/ /boot-0/\"; saveenv; boot") ;;
    *) fail "noenv: no U-Boot line in: $out" ;;
esac

# A kernel booted from slot 1 that is neither SiMa's nor the package's: the package owns slot 0.
fresh second
image "$ROOT/boot/boot-1/Image" "$BENCH_BANNER"
printf '%s\n' "$BENCH_BANNER" > "$ROOT/proc/version"
printf 'boot_path=/boot-1/\nboot_prefixes=/boot-1/ /boot-0/\ndtbos=nxs-cam-buses.dtbo\n' > "$ROOT/uboot.env"
mv "$ROOT/boot/boot-0/nxs-cam-buses.dtbo" "$ROOT/boot/boot-1/"
run install > /dev/null
has_banner "$ROOT/boot/boot-0/Image" "$NXS_BANNER" || fail "second: slot 0 holds no package kernel"
has_banner "$ROOT/boot/boot-0/Image.sima" "$SIMA_BANNER" || fail "second: slot 0 kept no SiMa Image"
has_banner "$ROOT/boot/boot-1/Image" "$BENCH_BANNER" || fail "second: the running kernel's slot changed"
[ -f "$ROOT/boot/boot-0/nxs-cam-buses.dtbo" ] || fail "second: the bus overlay is not in slot 0"
expect_eq "second: boot_path" "$(env_of boot_path)" "/boot-0/"

# An overlay dtbos names that neither slot holds is named.
fresh lost
printf 'boot_path=/boot-0/\ndtbos=nxs-cam-buses.dtbo nxs-cam0.dtbo\n' > "$ROOT/uboot.env"
out="$(run install)"
case "$out" in
    *"/boot/boot-1 lacks nxs-cam0.dtbo, and U-Boot's boot script fails on an overlay its slot lacks"*) ;;
    *) fail "lost: no missing-overlay line in: $out" ;;
esac

# An image without the two slots keeps the kernel where the package put it.
fresh noslots
rm -rf "$ROOT/boot/boot-1"
if out="$(run install)"; then
    fail "noslots: the install succeeded"
fi
expect_eq "noslots: line" "$out" "nxs-modalix: this image has no U-Boot boot slot /boot/boot-1, so the kernel stays in /usr/lib/nxs-modalix"

if [ "$failures" -ne 0 ]; then
    echo "$failures failure(s)"
    exit 1
fi
echo "kernel-slot: all cases pass"

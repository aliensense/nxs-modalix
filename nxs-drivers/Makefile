# SPDX-License-Identifier: GPL-2.0-only
# Out-of-tree build of the NXS camera map driver:
#   make            builds nxs-csi2-core.ko, nxs-csi2.ko and nxs-hub-route.ko against
#                   /lib/modules/$(uname -r)/build
#   make KDIR=...   builds against another kernel tree
#   make KDIR=... ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-
#                   cross-builds against a tree configured for another machine,
#                   whose build helpers (scripts/basic/fixdep, scripts/mod/modpost)
#                   run on this one
#   make W=1        builds with the kernel's extra warnings
#   make install    installs the modules under /lib/modules/<release>/extra
KDIR ?= /lib/modules/$(shell uname -r)/build
M := $(CURDIR)/drivers/media/i2c
ARCH ?=
CROSS_COMPILE ?=
W ?=
KBUILD := $(MAKE) -C $(KDIR) M=$(M) CONFIG_VIDEO_NXS_CSI2=m \
	$(if $(ARCH),ARCH=$(ARCH)) $(if $(CROSS_COMPILE),CROSS_COMPILE=$(CROSS_COMPILE)) \
	$(if $(W),W=$(W))

all:
	$(KBUILD) modules

install:
	$(KBUILD) modules_install

clean:
	$(KBUILD) clean

.PHONY: all install clean

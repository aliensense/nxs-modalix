# SPDX-License-Identifier: GPL-2.0-only
# The drivers' user-space test: nxs-csi2-core.c and nxs-csi2.c against a fake camera door.
#   make -C tests        builds and runs it
#   make -C tests clean  removes the binary
CC ?= cc
I2C := ../nxs-csi2/drivers/media/i2c
# A use after free or an undefined operation fails the run.
SANITIZE ?= -fsanitize=address,undefined -fno-sanitize-recover=undefined
CFLAGS := -std=gnu11 -O1 -g $(SANITIZE) -Wall -Wextra -Werror -Wno-unused-parameter \
	-Wno-sign-compare -Wno-missing-field-initializers -Wno-pointer-sign -Ikstub -I. -I$(I2C)
SRCS := test-core.c test-adapter.c fake-door.c kstub/kstub.c kstub/kstub-subdev.c \
	$(I2C)/nxs-csi2-core.c
HDRS := adapter.h fake-door.h kstub/kstub.h kstub/kstub-subdev.h $(I2C)/nxs-csi2-core.h \
	$(I2C)/nxs-csi2-map.h $(I2C)/nxs-csi2.c

test: test-core
	ASAN_OPTIONS=detect_leaks=0 ./test-core

test-core: $(SRCS) $(HDRS)
	$(CC) $(CFLAGS) -o $@ $(SRCS)

clean:
	rm -rf test-core test-core.dSYM

.PHONY: test clean

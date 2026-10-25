# Out-of-tree build:  make [KDIR=/lib/modules/$(uname -r)/build]
obj-m += nicogpu.o
nicogpu-y := nicogpu_main.o nicogpu_drm.o nicogpu_debugfs.o nicogpu_mm.o nicogpu_vm_core.o nicogpu_sched.o nicogpu_sched_core.o
nicogpu-y += nicogpu_dfx.o nicogpu_dfx_core.o nicogpu_img.o nicogpu_dump_core.o nicogpu_ginfo_core.o nicogpu_fence.o
# gen/nicogpu_pte.h is ptegen output (vendored); kcompat/ supplies its <stdint.h>
ccflags-y += -I$(src) -I$(src)/gen -I$(src)/kcompat
KDIR ?= /lib/modules/$(shell uname -r)/build
all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules
clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean

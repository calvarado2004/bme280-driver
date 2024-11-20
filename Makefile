# Set up the kernel directory to Buildroot output
KERNELDIR ?= /home/carlos/final-project-calvarado2004/buildroot/output/build/linux-custom
PWD := $(shell pwd)

# Target module name
obj-m += bme_driver.o

# Compiler settings
CC ?= $(CROSS_COMPILE)gcc
CFLAGS ?= -Wall
LDFLAGS ?=

# Pass custom CFLAGS to the kernel build system
EXTRA_CFLAGS := $(CFLAGS)

all:
	$(MAKE) -C $(KERNELDIR) ARCH=arm64 CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) EXTRA_CFLAGS="$(CFLAGS)" modules

clean:
	$(MAKE) -C $(KERNELDIR) ARCH=arm64 CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) clean

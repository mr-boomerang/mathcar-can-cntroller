
# ------------------------- Secondary Call ----------------------------------
ifneq ($(KERNELRELEASE),)

#EXTRA_CFLAGS = $(INCLUDES) $(MODFLAGS)
EXTRA_CFLAGS = -g

obj-m :=	systec_can.o


# ------------------------- Primary Call ------------------------------------
else

.EXPORT_ALL_VARIABLES:
KDIR			:= /lib/modules/$(shell uname -r)/build
PWD			:= $(shell pwd)
FW_DIR			:= /lib/firmware
FIRMWARE_FILES		:= systec_can-*.fw

# ------------- Default-Target -------------
all:
			$(MAKE) -C $(KDIR) M=$(PWD) modules

modules_install:
			$(MAKE) -C $(KDIR) M=$(PWD) modules_install
			@[ -r $(KDIR)/System.map ] || depmod -a

firmware_install:
			@mkdir -p $(FW_DIR)
			@install --mode=644 $(FIRMWARE_FILES) $(FW_DIR)
			@echo "Firmware files installed to $(FW_DIR)"

install:		modules_install firmware_install

firmware:
			$(MAKE) -C hex2bin firmware

dist:
			git archive --format=tar --prefix=systec_can/ $(shell git describe) | bzip2 > systec_can-$(shell git describe).tar.bz2

endif



clean:
			$(MAKE)	-C $(KDIR) M=$(PWD) clean

.PHONY: all clean

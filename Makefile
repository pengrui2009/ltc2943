#
# Makefile for the input misc STM acc lis3dh driver
#
obj-m := drvltc2943.o
drvltc2943-objs := ltc2943.o ltc2943_core.o ltc2943_i2c.o
KERNELDIR := /home/linux/imx6/linux-3.14.52/
PWD := $(shell pwd)

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
	#arm-linux-gnueabihf-gcc -o test_lis3dh test/test_lis3dh.c -I./
clean:
	-rm -rf *.o *.ko *.mod.c .*.cmd .tmp_versions Module.symvers modules.order 

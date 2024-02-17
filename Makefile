ARCH := arm
CROSS_COMPILE :=arm-linux-gnueabihf-
KERNELDIR := ../linux-6.1.58
# KERNELDIR:=/lib/modules/$(shell uname -r)/build
CURRENT_PATH := $(shell pwd)
obj-m += echodev-drv.o

all:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules
clean:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNELDIR) M=$(CURRENT_PATH) clean
bar0_test: bar0_test.o
	$(CROSS_COMPILE)gcc bar0_test.o -o bar0_test
bar1_test: bar1_test.o
	$(CROSS_COMPILE)gcc bar1_test.o -o bar1_test
bar0_test.o : bar0_test.c
	$(CROSS_COMPILE)gcc -c bar0_test.c
bar1_test.o : bar1_test.c
	$(CROSS_COMPILE)gcc -c bar1_test.c
my_process : my_process.o
	$(CROSS_COMPILE)gcc my_process.o -o my_process -lpthread
my_process.o : my_process.c
	$(CROSS_COMPILE)gcc -c my_process.c
clean:
	make -C ../linux-6.1.58 M=$(PWD) clean
	rm -rf bar0_test bar1_test.o bar1_test bar1_test.o my_process my_process.o

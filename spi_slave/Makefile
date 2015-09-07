CROSS_COMPILE=/opt/Stone/usr/local/arm/3.4.1/bin/arm-linux-
#CROSS_COMPILE=arm-linux-
ARCH:=arm
CC:=$(CROSS_COMPILE)gcc
LD:=$(CROSS_COMPILE)ld
obj-m = spi_s3c24xx_slave.o
module-objs = spi_s3c24xx_slave.o spi_s3c24xx_fiq.o

#KDIR = /home/rdp/work/s3c2440/linux-2.6.35/
KDIR = /opt/Stone/linux-2.6.35
PWD = $(shell pwd)
default:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules
clean:
	rm -rf  *.o *.ko *~

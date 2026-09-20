# 内核模块构建 (在 ARM 服务器上执行, 需要对应内核版本的 headers)
obj-m += valarm.o

KDIR ?= /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean

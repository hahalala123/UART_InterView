# 用户态驱动构建 (在 ARM 服务器上执行)
CC      ?= gcc
CFLAGS  ?= -O2 -Wall -Wextra -std=gnu11
LDLIBS  ?= -lrt

all: alarmd libfoo/libfoo.so

alarmd: alarmd.c libfoo/libfoo.so
	$(CC) $(CFLAGS) -o $@ alarmd.c -Llibfoo -lfoo $(LDLIBS) \
		-Wl,-rpath,'$$ORIGIN/libfoo' -Wl,-z,now

libfoo/libfoo.so: libfoo/foo.c
	$(CC) $(CFLAGS) -shared -fPIC -o $@ $<

clean:
	rm -f alarmd libfoo/libfoo.so

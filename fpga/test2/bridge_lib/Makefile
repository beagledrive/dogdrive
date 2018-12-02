ARCH = arm
CC = gcc
PWD := $(shell pwd)

CFLAGS = \
        -std=c99 \
	-g \
	-O3 \
        -W \
        -Wall \
        -Wextra \
	-Wp,-MMD,$(dir $@).$(notdir $@).d \
        -Wp,-MT,$@ \

bins-y += main

all: $(bins-y)

$(bins-y):
	$(CC) -o $@ $^
main: main.o bw_bridge.o

clean:
	$(RM) *.o *~ $(bins-y)

-include .*.d

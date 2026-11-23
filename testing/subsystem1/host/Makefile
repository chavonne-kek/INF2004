CC     ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -Wconversion -O1 -D_DEFAULT_SOURCE
INC     = -I stubs -I ../../../app_program/subsystem1

run: test_host
	./test_host

test_host: test_host.c ../../../app_program/subsystem1/barcode.c ../../../app_program/subsystem1/line_sense.c
	$(CC) $(CFLAGS) $(INC) -o $@ test_host.c -lm

clean:
	rm -f test_host

.PHONY: run clean

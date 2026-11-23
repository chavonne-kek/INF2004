SRC     = ../../../app_program/subsystem4
CC     ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -O1 -D_DEFAULT_SOURCE
INC     = -I ../../host_common/stubs -I $(SRC)

run: test_host
	./test_host

test_host: test_host.c $(SRC)/scan_calc.c
	$(CC) $(CFLAGS) $(INC) -o $@ test_host.c $(SRC)/scan_calc.c -lm

clean:
	rm -f test_host

.PHONY: run clean

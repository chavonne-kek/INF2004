SRC     = ../../../app_program/subsystem5
CC     ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -O1
INC     = -I ../../host_common/stubs -I $(SRC)

run: test_host
	./test_host

test_host: test_host.c $(SRC)/comms_codec.c
	$(CC) $(CFLAGS) $(INC) -o $@ test_host.c $(SRC)/comms_codec.c

clean:
	rm -f test_host

.PHONY: run clean

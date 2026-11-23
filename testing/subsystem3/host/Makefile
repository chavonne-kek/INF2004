SRC     = ../../../app_program/subsystem3
CC     ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -O1 -D_DEFAULT_SOURCE
INC     = -I ../../host_common/stubs -I $(SRC)
MODS    = $(SRC)/imu.c $(SRC)/imu_glue.c $(SRC)/imu_math.c

run: test_host
	./test_host

test_host: test_host.c $(MODS)
	$(CC) $(CFLAGS) $(INC) -o $@ test_host.c $(MODS) -lm

clean:
	rm -f test_host

.PHONY: run clean

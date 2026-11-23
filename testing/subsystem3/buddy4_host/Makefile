# Zi Siang's own PC tests for imu.c (35 checks), unchanged since 26 Oct
# except for where imu.c now lives (app_program/subsystem3).
SRC     = ../../../app_program/subsystem3
CC     ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -Werror -DIMU_TEST_HOOKS

run: imu_host_test
	./imu_host_test

imu_host_test: imu_host_test.c $(SRC)/imu.c $(SRC)/imu.h
	$(CC) $(CFLAGS) -I $(SRC) -o $@ imu_host_test.c $(SRC)/imu.c

clean:
	rm -f imu_host_test imu_host_test.exe

.PHONY: run clean

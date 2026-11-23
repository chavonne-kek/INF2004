# Buddy 5's own host tests (his test/host_test.c, unchanged), built
# against the integrated b5_*.c files in app_program/subsystem4.
SRC     = ../../../app_program/subsystem4
CC     ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -Wconversion -DB5_HOST_TEST \
          -D_POSIX_C_SOURCE=200809L -D_DEFAULT_SOURCE

run: b5_host_test
	./b5_host_test

b5_host_test: host_test.c $(SRC)/b5_types.c $(SRC)/b5_profile.c $(SRC)/b5_plan.c
	$(CC) $(CFLAGS) -I $(SRC) -o $@ $(SRC)/b5_types.c $(SRC)/b5_profile.c \
	    $(SRC)/b5_plan.c host_test.c -lm

clean:
	rm -f b5_host_test

.PHONY: run clean

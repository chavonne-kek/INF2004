# Round trip: firmware pack functions (C) -> vectors -> dashboard decoder
# (Python).  Tribunal 1 risk R2.  Needs gcc and python3.
APP     = ../../../app_program
CC     ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -O1
INC     = -I ../../host_common/stubs
SRCS    = $(APP)/subsystem1/line_sense.c $(APP)/subsystem1/barcode.c \
          $(APP)/subsystem2/motion_calc.c $(APP)/subsystem3/imu_glue.c \
          $(APP)/subsystem3/imu.c $(APP)/subsystem3/imu_math.c \
          $(APP)/subsystem4/scan_calc.c $(APP)/subsystem5/comms_codec.c

run: gen_vectors
	./gen_vectors vectors.jsonl
	python3 ../../../tools/is25_dashboard.py --selftest vectors.jsonl

gen_vectors: gen_vectors.c $(SRCS)
	$(CC) $(CFLAGS) $(INC) -o $@ gen_vectors.c $(SRCS) -lm

clean:
	rm -f gen_vectors gen_vectors.exe vectors.jsonl

.PHONY: run clean

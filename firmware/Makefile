ROOT := ..
CC := gcc
CFLAGS := -std=c11 -O2 -Wall -Wextra -Werror -Wconversion -Wshadow -Iinclude
SOURCES := src/protocol.c src/supervisor.c src/balance.c src/estimator.c src/kinematics.c src/fallen.c
# M12 following: built with the application, not into the balance core library.
FOLLOWER := src/follower.c
# M13 learned controller's runtime, selection and handover (no weights: the forward
# pass, src/locomotion/locomotion.c, links only where its generated header exists).
LEARNED := src/locomotion/learned.c
# learned.c needs the M10 serial kinematics: its tests and ARM object use the M10
# generated_config.h (the one the M11 image compiles against), from make m10-config.

.PHONY: all config m10-config test arm teensy teensy-m11 locomotion locomotion-arm clean
all: config build/libbipe.so
config:
	$(ROOT)/.venv/bin/python $(ROOT)/tools/generate_firmware_config.py
build/libbipe.so: $(SOURCES) include/bipe.h include/fallen.h include/generated_config.h
	mkdir -p build
	$(CC) $(CFLAGS) -fPIC -shared $(SOURCES) -lm -o $@
test: all m10-config
	$(CC) $(CFLAGS) tests/test_core.c $(SOURCES) -lm -o build/test_core
	./build/test_core
	$(CC) $(CFLAGS) tests/test_control.c $(SOURCES) -lm -o build/test_control
	./build/test_control
	$(CC) $(CFLAGS) tests/test_fallen.c $(filter-out src/fallen.c,$(SOURCES)) src/fallen.c -lm -o build/test_fallen
	./build/test_fallen
	$(CC) $(CFLAGS) -DBIPE_ALLOW_SYNTHETIC_APPLICATION=1 tests/test_application.c src/application/application.c $(FOLLOWER) $(SOURCES) -lm -o build/test_application
	./build/test_application
	$(CC) $(CFLAGS) -Wno-unused-function -DBIPE_ALLOW_SYNTHETIC_APPLICATION=0 tests/test_application.c src/application/application.c $(FOLLOWER) $(SOURCES) -lm -o build/test_application_inhibited
	./build/test_application_inhibited
	$(CC) $(CFLAGS) tests/test_follower.c $(FOLLOWER) -lm -o build/test_follower
	./build/test_follower
	$(CC) $(CFLAGS) -iquote build/m11-config tests/test_learned.c $(LEARNED) $(SOURCES) -lm -o build/test_learned
	./build/test_learned
	$(CC) $(CFLAGS) -iquote build/m11-config -DBIPE_LEARNED_LEG_FILTER_TAU_S=0.03 tests/test_learned.c $(LEARNED) $(SOURCES) -lm -o build/test_learned_filter
	./build/test_learned_filter
arm: config m10-config
	mkdir -p build/arm
	$(ROOT)/.local-tools/root/usr/bin/arm-none-eabi-gcc $(CFLAGS) -iquote build/m11-config -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -I$(ROOT)/.local-tools/root/usr/include/newlib -c $(LEARNED) -o build/arm/learned.o
	set -e; for source in $(SOURCES) $(FOLLOWER) src/application/application.c; do $(ROOT)/.local-tools/root/usr/bin/arm-none-eabi-gcc $(CFLAGS) -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -I$(ROOT)/.local-tools/root/usr/include/newlib -c $$source -o build/arm/$$(basename $$source .c).o; done
teensy: config
	PLATFORMIO_CORE_DIR=$(abspath $(ROOT))/.local-tools/platformio $(ROOT)/.venv/platformio/bin/pio run -e teensy40
m10-config:
	mkdir -p build/m11-config
	$(ROOT)/.venv/bin/python $(ROOT)/tools/generate_m11_hardware.py --m10-config build/m11-config/generated_config.h
teensy-m11: m10-config include/generated_locomotion_weights.h include/generated_locomotion_parity.h
	PLATFORMIO_CORE_DIR=$(abspath $(ROOT))/.local-tools/platformio $(ROOT)/.venv/platformio/bin/pio run -e teensy40_m11
include/generated_locomotion_weights.h include/generated_locomotion_parity.h:
	@echo "$@ is missing: run .venv/bin/python -m bipe.m13.headers (numpy only)"; exit 1
# M13 forward pass; python -m bipe.m13.export writes the weights header first
locomotion: build/liblocomotion.so
build/liblocomotion.so: src/locomotion/locomotion.c include/locomotion.h include/generated_locomotion_weights.h
	mkdir -p build
	$(CC) $(CFLAGS) -fPIC -shared src/locomotion/locomotion.c -lm -o $@
# Cortex-M7 image of the forward pass alone (entry bipe_locomotion_forward), for the
# instruction count of python -m bipe.m13.export_check budget
ARM := $(ROOT)/.local-tools/root/usr
locomotion-arm: src/locomotion/locomotion.c include/locomotion.h include/generated_locomotion_weights.h
	mkdir -p build/arm
	$(ARM)/bin/arm-none-eabi-gcc $(CFLAGS) -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -I$(ARM)/include/newlib -nostartfiles -nostdlib -Wl,-Ttext=0x10000 -Wl,-e,bipe_locomotion_forward -Wl,--no-warn-rwx-segments src/locomotion/locomotion.c -L$(ARM)/lib/arm-none-eabi/newlib/thumb/v7e-m+dp/hard -lm -lc -lnosys -lgcc -o build/arm/locomotion.elf
	$(ARM)/bin/arm-none-eabi-objcopy -O binary build/arm/locomotion.elf build/arm/locomotion.bin
clean:
	rm -rf build

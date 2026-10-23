# FGG-XSense
#
#   export PS5_PAYLOAD_SDK=/opt/ps5-payload-sdk
#   make

PS5_HOST ?= ps5
PS5_PORT ?= 9021

ifdef PS5_PAYLOAD_SDK
    include $(PS5_PAYLOAD_SDK)/toolchain/prospero.mk
else
    $(error PS5_PAYLOAD_SDK is undefined)
endif

ELF   := fgg-xsense.elf
BUILD := build

CFLAGS := -std=c11 -Wall -Wextra -Werror -O2 -Isrc

# No libScePad: the host process has not loaded it, so calls into the module
# fault. The ioctls it wraps are issued directly instead.

SRCS := src/main.c src/gip.c src/vpad.c src/log.c src/xbox_adapter.c
OBJS := $(patsubst %.c,$(BUILD)/%.o,$(SRCS))

FIRMWARE := FW_ACC_00U.bin
ifneq ($(wildcard $(FIRMWARE)),)
CFLAGS += -DHAS_XBOX_WIRELESS_FIRMWARE
OBJS += $(BUILD)/firmware_blob.o
endif

.PHONY: all clean test

all: $(ELF)

$(ELF): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD)/firmware_blob.c: $(FIRMWARE) tools/embed_firmware.py
	@mkdir -p $(dir $@)
	python3 tools/embed_firmware.py $(FIRMWARE) $@

$(BUILD)/firmware_blob.o: $(BUILD)/firmware_blob.c src/firmware_blob.h
	$(CC) $(CFLAGS) -c -o $@ $<

test: $(ELF)
	$(PS5_DEPLOY) -h $(PS5_HOST) -p $(PS5_PORT) $^

clean:
	rm -rf $(BUILD) $(ELF)

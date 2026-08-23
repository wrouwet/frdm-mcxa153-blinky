TARGET   := blinky
BUILDDIR := build

SDK   := vendor/sdk
CMSIS := vendor/cmsis

DEVICE_DIR := $(SDK)/devices/MCXA153

CC      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE    := arm-none-eabi-size

CPU_FLAGS := -mcpu=cortex-m33+nodsp -mthumb -mfloat-abi=soft

INCLUDES := \
	-I$(DEVICE_DIR) \
	-I$(CMSIS)/CMSIS/Core/Include \
	-Isrc

DEFINES := -DCPU_MCXA153VLH -D__STARTUP_CLEAR_BSS

CFLAGS := $(CPU_FLAGS) $(DEFINES) $(INCLUDES) \
	-std=gnu11 -Wall -Wextra -O2 -g3 \
	-ffunction-sections -fdata-sections \
	-fno-common -MMD -MP

ASFLAGS := $(CPU_FLAGS) $(DEFINES) $(INCLUDES)

LDSCRIPT := $(DEVICE_DIR)/gcc/MCXA153_flash.ld

LDFLAGS := $(CPU_FLAGS) -T$(LDSCRIPT) \
	--specs=nano.specs --specs=nosys.specs \
	-Wl,--gc-sections -Wl,-Map=$(BUILDDIR)/$(TARGET).map \
	-static

SRCS_C := src/main.c src/rtt.c $(DEVICE_DIR)/system_MCXA153.c
SRCS_S := $(DEVICE_DIR)/gcc/startup_MCXA153.S

OBJS := $(SRCS_C:%.c=$(BUILDDIR)/%.o) $(SRCS_S:%.S=$(BUILDDIR)/%.o)

.PHONY: all clean flash

all: $(BUILDDIR)/$(TARGET).elf $(BUILDDIR)/$(TARGET).bin $(BUILDDIR)/$(TARGET).hex
	$(SIZE) $(BUILDDIR)/$(TARGET).elf

$(BUILDDIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILDDIR)/%.o: %.S
	@mkdir -p $(dir $@)
	$(CC) $(ASFLAGS) -c $< -o $@

$(BUILDDIR)/$(TARGET).elf: $(OBJS)
	@mkdir -p $(BUILDDIR)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

$(BUILDDIR)/$(TARGET).bin: $(BUILDDIR)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(BUILDDIR)/$(TARGET).hex: $(BUILDDIR)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

flash: $(BUILDDIR)/$(TARGET).elf
	probe-rs download --chip MCXA153 --binary-format elf $(BUILDDIR)/$(TARGET).elf
	probe-rs reset --chip MCXA153

clean:
	rm -rf $(BUILDDIR)

-include $(OBJS:.o=.d)

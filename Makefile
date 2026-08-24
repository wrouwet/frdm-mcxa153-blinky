TARGET   ?= blinky
BUILDDIR := build/$(TARGET)

SDK   := vendor/sdk
CMSIS := vendor/cmsis
USB   := vendor/usb

DEVICE_DIR := $(SDK)/devices/MCXA153

CC      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE    := arm-none-eabi-size

CPU_FLAGS := -mcpu=cortex-m33+nodsp -mthumb -mfloat-abi=soft

COMMON_INCLUDES := \
	-I$(DEVICE_DIR) \
	-I$(CMSIS)/CMSIS/Core/Include \
	-Isrc

COMMON_DEFINES := -DCPU_MCXA153VLH -D__STARTUP_CLEAR_BSS

ifeq ($(TARGET),blinky)
APP_SRCS_C := src/main.c src/rtt.c
APP_INCLUDES :=
APP_DEFINES :=

else ifeq ($(TARGET),usb_vcom)
APP_SRCS_C := \
	src/usb_main.c src/rtt.c src/usb/retarget.c \
	src/usb/usb_device_descriptor.c \
	$(USB)/device/usb_device_dci.c \
	$(USB)/device/usb_device_khci.c \
	$(USB)/device/usb_device_ch9.c \
	$(USB)/device/class/usb_device_class.c \
	$(USB)/device/class/usb_device_cdc_acm.c \
	$(SDK)/drivers/common/fsl_common.c \
	$(SDK)/drivers/common/fsl_common_arm.c \
	$(SDK)/components/osa/fsl_os_abstraction_bm.c \
	$(SDK)/components/lists/fsl_component_generic_list.c \
	$(DEVICE_DIR)/drivers/fsl_clock.c \
	$(DEVICE_DIR)/drivers/fsl_reset.c \
	$(SDK)/drivers/lpi2c/fsl_lpi2c.c
APP_INCLUDES := \
	-Isrc/usb \
	-I$(USB)/include \
	-I$(USB)/device \
	-I$(USB)/device/class \
	-I$(SDK)/drivers/common \
	-I$(SDK)/drivers/lpi2c \
	-I$(SDK)/drivers/port \
	-I$(SDK)/components/osa \
	-I$(SDK)/components/lists \
	-I$(DEVICE_DIR)/drivers
APP_DEFINES := -DSDK_DEBUGCONSOLE=0 -DI2C_RETRY_TIMES=50000U

else
$(error Unknown TARGET '$(TARGET)'; use TARGET=blinky or TARGET=usb_vcom)
endif

INCLUDES := $(COMMON_INCLUDES) $(APP_INCLUDES)
DEFINES  := $(COMMON_DEFINES) $(APP_DEFINES)

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

SRCS_C := $(APP_SRCS_C) $(DEVICE_DIR)/system_MCXA153.c
SRCS_S := $(DEVICE_DIR)/gcc/startup_MCXA153.S

OBJS := $(SRCS_C:%.c=$(BUILDDIR)/%.o) $(SRCS_S:%.S=$(BUILDDIR)/%.o)

.PHONY: all clean flash run

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

run: $(BUILDDIR)/$(TARGET).elf
	probe-rs run --chip MCXA153 $(BUILDDIR)/$(TARGET).elf

clean:
	rm -rf build

-include $(OBJS:.o=.d)

# Check arguments
ifeq ($(HW), ROBOTIC_ARM) # HW argument
TARGET_HW = robotic_arm
STM_VERSION = STM32F446xx
else ifeq ($(HW), ARM_SLEEVE)
TARGET_HW = arm_sleeve
STM_VERSION = STM32F411xE
else ifeq ($(MAKECMDGOALS), clean)
else ifeq ($(MAKECMDGOALS), format)
# HW argument not required for clean, format
else
$(error "Must pass HW=ROBOTIC_ARM or HW=ARM_SLEEVE")
endif
TARGET_NAME = $(TARGET_HW)

ifneq ($(TEST),) # TEST argument
ifeq ($(findstring test_, $(TEST)), )
$(error "TEST=$(TEST) is invalid (test function must start with test_)")
else
TARGET_NAME=$(TEST)
endif
endif

# Directories

# Tools path for Arm Directory 
## Personal: /Applications/ArmGNUToolchain/14.3.rel1/arm-none-eabi
TOOLS_ARM_DIR = ${TOOLS_ARM_DIR_PATH}
ARMGCC_ROOT_DIR = $(TOOLS_ARM_DIR)
ARMGCC_BIN_DIR = $(ARMGCC_ROOT_DIR)/bin
ARMGCC_INCLUDE_DIR = $(ARMGCC_ROOT_DIR)/include
ARMGCC_LIB_DIR = $(ARMGCC_ROOT_DIR)/lib
ARMGCC_LIB_GCC_DIR = $(ARMGCC_LIB_DIR)/gcc/arm-none-eabi/14.3.1
ARMGCC_STANDARD_LIB_INCLUDE_DIRS = $(ARMGCC_LIB_GCC_DIR)/include \
								   $(ARMGCC_LIB_GCC_DIR)/include-fixed \

# include directory features files taken from STM32CubeIDE
INCLUDE = $(ARMGCC_INCLUDE_DIR)
INCLUDE_DIRS = $(INCLUDE)
LIB_DIRS = $(INCLUDE)
BUILD_DIR = build/$(TARGET_NAME)
OBJ_DIR = $(BUILD_DIR)/obj
BIN_DIR = $(BUILD_DIR)/bin

# OpenOCD = Debugger / Program Loader
OPEN_OCD = openocd

# Tools Path for Open-ocd
## Personal path: /opt/homebrew/Cellar/open-ocd/0.12.0_1/share/openocd/scripts
TOOLS_OPEN_OCD_DIR = ${TOOLS_OPEN_OCD_DIR_PATH}
OPEN_OCD_DIR = $(TOOLS_ARM_OPEN_OCD_DIR)
OPEN_OCD_TARGET = $(OPEN_OCD_DIR)/target/stm32f4x.cfg
OPEN_OCD_STLINK = $(OPEN_OCD_DIR)/interface/stlink.cfg

# Toolchain
CC = $(ARMGCC_BIN_DIR)/arm-none-eabi-gcc
RM = rm
CPPCHECK = cppcheck
FORMAT = clang-format
SIZE = $(ARMGCC_BIN_DIR)/arm-none-eabi-size
READELF = $(ARMGCC_BIN_DIR)/arm-none-eabi-readelf

# Files
TARGET = $(BUILD_DIR)/bin/$(TARGET_HW)/$(TARGET_NAME)

SOURCES_WITH_HEADERS = \
		  src/drivers/led.c \
		  src/drivers/io.c \
		  src/drivers/adc.c \
		  src/drivers/mcu_init.c \
		  src/common/assert_handler.c \

ifndef TEST
SOURCES = \
		  src/main.c \
		  $(SOURCES_WITH_HEADERS)
else
SOURCES = \
		  src/test/test.c \
		  $(SOURCES_WITH_HEADERS)
# Delete object file to force rebuild when changing test
$(shell rm -f $(BUILD_DIR)/obj/src/test/test.o)
endif

HEADERS = \
		  $(SOURCES_WITH_HEADERS:.c=.h) \
		  src/common/defines.h \

OBJECT_NAMES = $(SOURCES:.c=.o)
OBJECTS = $(patsubst %, $(OBJ_DIR)/%, $(OBJECT_NAMES))

# Defines
HW_DEFINE = $(addprefix -D, $(HW)) # e.g. -DROBOTIC_ARM or -DARM_SLEEVE
TEST_DEFINE = $(addprefix -DTEST=, $(TEST))
DEFINES = \
	$(HW_DEFINE) \
	$(TEST_DEFINE) \

# Flags
MCPU = cortex-m4
LINKER_SCRIPT = $(INCLUDE)/linker_script.ld
STARTUP = $(INCLUDE)/startup.c

# Warning flags
WFLAGS = -Wall -Wextra -Werror -Wshadow
# Compiler flags
CFLAGS = -mcpu=$(MCPU) -mthumb -D$(STM_VERSION) $(WFLAGS) $(addprefix -I, $(INCLUDE_DIRS)) $(DEFINES) -Og -g 
# Linker flags
LDFLAGS = -mcpu=$(MCPU) -mthumb -D$(STM_VERSION) $(DEFINES) $(addprefix -L, $(LIB_DIRS)) -T$(LINKER_SCRIPT) $(addprefix -I, $(INCLUDE_DIRS))
# Start up flags
SUPFLAGS = $(STARTUP) -Wl,--gc-sections

# Build
## Linking
$(TARGET): $(OBJECTS)
	echo $(OBJECTS)
	@mkdir -p $(dir $@)
	$(CC) $(LDFLAGS) $(SUPFLAGS) $^ -o $@

## Compiling
$(OBJ_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
# @ is output ^ is input
	$(CC) $(CFLAGS) -c -o $@ $^ 

.PHONY: all clean cppcheck format

all: $(TARGET)

clean:
	$(RM) -rf $(BUILD_DIR)

flash: $(TARGET)
	$(OPEN_OCD) -f $(OPEN_OCD_STLINK) -f $(OPEN_OCD_TARGET) -c "program $(TARGET) verify reset exit"

cppcheck:
	$(CPPCHECK) --enable=all --error-exitcode=1 \
		--inline-suppr --force \
		--check-level=exhaustive \
		--suppress=missingIncludeSystem \
		--suppress=unusedFunction \
		--suppress=unmatchedSuppression \
		--suppress=staticFunction \
		--suppress=*:$(INCLUDE)/* \
		-D$(STM_VERSION) \
		-I $(INCLUDE_DIRS) \
		-I $(ARMGCC_INCLUDE_DIR) \
		$(addprefix -I, $(ARMGCC_STANDARD_LIB_INCLUDE_DIRS)) \
		$(SOURCES) \
		$(DEFINES)

format:
	@$(FORMAT) -i $(SOURCES) $(HEADERS)

size: $(TARGET)
	@$(SIZE) $(TARGET)

symbols: $(TARGET)
# List symbols table sorted by size
	@$(READELF) -s $(TARGET) | sort -n -k3

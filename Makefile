# Directories
ARMGCC_ROOT_DIR = /Applications/ArmGNUToolchain/14.3.rel1/arm-none-eabi
ARMGCC_BIN_DIR = $(ARMGCC_ROOT_DIR)/bin
ARMGCC_INCLUDE_DIR = $(ARMGCC_ROOT_DIR)/include
ARMGCC_LIB_DIR = $(ARMGCC_ROOT_DIR)/lib
ARMGCC_LIB_GCC_DIR = $(ARMGCC_LIB_DIR)/gcc/arm-none-eabi/14.3.1
ARMGCC_STANDARD_LIB_INCLUDE_DIRS = $(ARMGCC_LIB_GCC_DIR)/include \
								   $(ARMGCC_LIB_GCC_DIR)/include-fixed \

# include directory features files taken from STM32CubeIDE
INCLUDE = include
INCLUDE_DIRS = $(INCLUDE)
LIB_DIRS = $(INCLUDE)
BUILD_DIR = build
OBJ_DIR = $(BUILD_DIR)/obj
BIN_DIR = $(BUILD_DIR)/bin

# OpenOCD = Debugger / Program Loader
OPEN_OCD = openocd
OPEN_OCD_DIR = /opt/homebrew/Cellar/open-ocd/0.12.0_1/share/openocd/scripts
OPEN_OCD_TARGET = $(OPEN_OCD_DIR)/target/stm32f4x.cfg
OPEN_OCD_STLINK = $(OPEN_OCD_DIR)/interface/stlink.cfg

# Toolchain
CC = $(ARMGCC_BIN_DIR)/arm-none-eabi-gcc
RM = rm
CPPCHECK = cppcheck

# Files
TARGET = $(BIN_DIR)/blink

SOURCES = src/main.c \
		  src/led.c

OBJECT_NAMES = $(SOURCES:.c=.o)
OBJECTS = $(patsubst %, $(OBJ_DIR)/%, $(OBJECT_NAMES))

# Flags
MCPU = cortex-m4
STM_VERSION = DSTM32F446xx
LINKER_SCRIPT = $(INCLUDE)/linker_script.ld
STARTUP = $(INCLUDE)/startup.c

# Warning flags
WFLAGS = -Wall -Wextra -Werror -Wshadow
# Compiler flags
CFLAGS = -mcpu=$(MCPU) -mthumb -$(STM_VERSION) $(WFLAGS) $(addprefix -I, $(INCLUDE_DIRS)) -Og -g 
# Linker flags
LDFLAGS = -mcpu=$(MCPU) -mthumb -$(STM_VERSION) $(addprefix -L, $(LIB_DIRS)) -T$(LINKER_SCRIPT) $(addprefix -I, $(INCLUDE_DIRS))
# Start up flags
SUPFLAGS = $(STARTUP) -Wl,--gc-sections

# Build
$(TARGET): $(OBJECTS)
	@mkdir -p $(dir $@)
	$(CC) $(LDFLAGS) $(SUPFLAGS) $^ -o $@

## Compiling
$(OBJ_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
# @ is output ^ is input
	$(CC) $(CFLAGS) -c -o $@ $^ 

.PHONY: all clean cppcheck

all: $(TARGET)

clean:
	$(RM) -r $(BUILD_DIR)

flash: $(TARGET)
	$(OPEN_OCD) -f $(OPEN_OCD_STLINK) -f $(OPEN_OCD_TARGET) -c "program $(TARGET) verify reset exit"

cppcheck:
	@$(CPPCHECK) --quiet --enable=all --error-exitcode=1 \
	--inline-suppr --force \
	--suppress=missingIncludeSystem \
	--suppress=unusedFunction \
	--suppress=*:include/* \
	-DSTM32F446xx \
	-I $(INCLUDE_DIRS) \
	-I $(ARMGCC_INCLUDE_DIR) \
	$(addprefix -I, $(ARMGCC_STANDARD_LIB_INCLUDE_DIRS)) \
	$(SOURCES)
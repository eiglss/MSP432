#===============================================================
# Makefile for building MSP Code Examples in command line
# environement using the GCC Open Source Compiler for MSP432
# Edited by Enzo IGLESIS
#===============================================================


ifndef DEVICE
    $(info Device unspecified, default: DEVICE=MSP432P401R)
    DEVICE			:= MSP432P401R
endif

ifndef TARGET
    $(info Target unspecified, default: TARGET=main.out)
    TARGET 			:= main.out
endif

SOURCES				:= $(patsubst libraries/src/%.c, %.c, $(wildcard libraries/src/*.c))
SOURCES 			+= $(patsubst src/%.c, %.c, $(wildcard src/*.c))

OBJ_DIR 			:= output

######################################

# GCC Configuration
DEVICE 					?= MSP432P401R
# Possible modification to do here
INSTALL_DIR				:= /home/enzo/ti/msp432_gcc
#
GCC_MSP_INC_DIR 		?= $(INSTALL_DIR)/arm/include
GCC_CMSIS_INC_DIR 		?= $(GCC_MSP_INC_DIR)/CMSIS
LDDIR					:= $(GCC_MSP_INC_DIR)/$(shell echo $(DEVICE) | tr A-Z a-z)

INC_DIR					:= libraries

RM						:= rm -rf
MKDIR					= mkdir -p -- $@

DONE					= [\e[32mDONE\e[0;m]:
NOTE					= [\e[36mNOTE\e[0;m]:

BOLD					=\e[1m
UNDERLINE				=\e[4m
DEFAULT					=\e[0;m

######################################
GCC_BIN_DIR				?= $(INSTALL_DIR)/arm_compiler/bin/
GCC_INC_DIR				?= $(INSTALL_DIR)/arm_compiler/arm-none-eabi/include
######################################
CC              		:= $(GCC_BIN_DIR)arm-none-eabi-gcc
GDB			    		:= $(GCC_BIN_DIR)arm-none-eabi-gdb
INCLUDES 				:= -I $(GCC_CMSIS_INC_DIR) -I $(GCC_MSP_INC_DIR) -I $(GCC_INC_DIR) -I $(INC_DIR)
COMMONFLAGS 			:= -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mthumb -D__$(DEVICE)__ -DTARGET_IS_MSP432P4XX -Dgcc -g -gstrict-dwarf -Wall -Wextra -pedantic
CFLAGS 					:= $(COMMONFLAGS) -ffunction-sections -fdata-sections -Wno-main
LDFLAGS 				:= $(COMMONFLAGS) -T$(LDDIR).lds -l'c' -l'gcc' -l'nosys'

ifeq ($(DEVICE), MSP432P401R)
    SOURCES += startup_msp432p401r_gcc.c
    SOURCES += system_msp432p401r.c
else
    ifeq ($(DEVICE), MSP432P401M)
        SOURCES += startup_msp432p401m_gcc.c
        SOURCES += system_msp432p401m.c
    else
        $(error Unknown device. Makefile for MSP432P401R and MSP432P401M only)
    endif
endif

VPATH := src:libraries:libraries/src:$(INSTALL_DIR)/arm/src

OBJECTS := $(addsuffix .o,$(addprefix $(OBJ_DIR)/, $(basename $(SOURCES))))

######################################

all: $(OBJ_DIR)/$(TARGET)

$(OBJECTS): | $(OBJ_DIR)

$(OBJ_DIR):
	@echo "created directory '$(BOLD)$(UNDERLINE)$(OBJ_DIR)/$(DEFAULT)'"
	@$(MKDIR) $(OBJ_DIR)


$(OBJ_DIR)/%.o: %.c
	@echo ======================================================================
	@echo "$(NOTE) Generating $(BOLD)$(UNDERLINE)$@$(DEFAULT)"
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(OBJ_DIR)/$(TARGET): $(OBJECTS)
	@echo ======================================================================
	@echo "$(NOTE) Linking objects and generating output binary $(BOLD)$(UNDERLINE)$@$(DEFAULT)"
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@ $(IsNCLUDES)
	@echo ======================================================================
	@echo "$(DONE) output binary: $(BOLD)$(UNDERLINE)$@$(DEFAULT)"
	@date

debug: all
	$(GDB) $(OBJ_DIR)/$(TARGET)

clean:
	@$(RM) $(OBJ_DIR)

test:
	@echo $(VPATH)

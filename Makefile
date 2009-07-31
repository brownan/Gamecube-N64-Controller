TARGET=gamecube

# Source directory for all arduino library files (should contain WProgram.h)
ARDUINO = ./libs

MCU = atmega328p
F_CPU = 16000000


AVR_TOOLS_PATH = /usr/bin

AVRDUDE = ../arduino-0016/hardware/tools/avrdude
AVRDUDE_CONF = ../arduino-0016/hardware/tools/avrdude.conf
AVRDUDE_PORT = /dev/ttyUSB0

# Probably don't need to edit below here
# #--------------------------

# Debugging format.
# Native formats for AVR-GCC's -g are stabs [default], or dwarf-2.
# AVR (extended) COFF requires stabs, plus an avr-objcopy run.
DEBUG = stabs

# Optimization level
OPT = s

# Place -D or -U options here
CDEFS = -DF_CPU=$(F_CPU)
CXXDEFS = -DF_CPU=$(F_CPU)

# Place -I options here
CINCS = -I$(ARDUINO)
CXXINCS = -I$(ARDUINO)

# Compiler flag to set the C Standard level.
# c89   - "ANSI" C
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions
CSTANDARD = -std=gnu99
CDEBUG = -g$(DEBUG)
CWARN = -Wall -Wstrict-prototypes
CTUNING = -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
#CEXTRA = -Wa,-adhlns=$(<:.c=.lst)

#CFLAGS = $(CDEBUG) $(CDEFS) $(CINCS) -O$(OPT) $(CWARN) $(CSTANDARD) $(CEXTRA)
CFLAGS = -g -Os -ffunction-sections -fdata-sections -DF_CPU=$(F_CPU)L -I$(ARDUINO)
#CXXFLAGS = $(CDEFS) $(CINCS) -O$(OPT)
CXXFLAGS = -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -DF_CPU=$(F_CPU)L -I$(ARDUINO)
#ASFLAGS = -Wa,-adhlns=$(<:.S=.lst),-gstabs 
LDFLAGS = -Os -Wl,--gc-sections -mmcu=$(MCU) -lm


# Program settings
CC = $(AVR_TOOLS_PATH)/avr-gcc
CXX = $(AVR_TOOLS_PATH)/avr-g++
OBJCOPY = $(AVR_TOOLS_PATH)/avr-objcopy
OBJDUMP = $(AVR_TOOLS_PATH)/avr-objdump
AR  = $(AVR_TOOLS_PATH)/avr-ar
SIZE = $(AVR_TOOLS_PATH)/avr-size
NM = $(AVR_TOOLS_PATH)/avr-nm
REMOVE = rm -f
MV = mv -f

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = -mmcu=$(MCU) -I. $(CFLAGS)
ALL_CXXFLAGS = -mmcu=$(MCU) -I. $(CXXFLAGS)
ALL_ASFLAGS = -mmcu=$(MCU) -I. -x assembler-with-cpp $(ASFLAGS)

LIB_SRC_CC = $(wildcard $(ARDUINO)/*.c)
LIB_SRC_CPP = $(wildcard $(ARDUINO)/*.cpp)
LIB_OBJ = $(LIB_SRC_CC:.c=.o) $(LIB_SRC_CPP:.cpp=.o)

FORMAT = ihex


# Default target.
all: $(TARGET).hex

upload: $(TARGET).hex
	$(AVRDUDE) -C$(AVRDUDE_CONF) -v -v -v -v -pm328p -cstk500v1 -P$(AVRDUDE_PORT) -b57600 -D -Uflash:w:$<:i

%.hex: %.elf
	$(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@

%.elf: %.o $(ARDUINO)/core.a
	$(CC) -o $@ $(LDFLAGS) $< $(ARDUINO)/core.a

$(ARDUINO)/core.a: $(LIB_OBJ)
	@for i in $(LIB_OBJ); do echo $(AR) rcs $(ARDUINO)/core.a $$i; $(AR) rcs $(ARDUINO)/core.a $$i; done


# Here is the "pre-preprocessing".  It creates a .c file based with the
# same name as the .pde file.  On top of the new file comes the
# WProgram.h header.  At the end there is a generic main() function
# attached.
%.cpp: %.pde
	echo '#include "WProgram.h"' > $@
	cat $< >> $@
	cat $(ARDUINO)/main.cxx >> $@

# Compile: create object files from C++ source files
%.o: %.cpp
	$(CXX) -c $(ALL_CXXFLAGS) $< -o $@ 

# Compile: create object files from C source files.
%.o: %.c
	$(CC) -c $(ALL_CFLAGS) $< -o $@ 

# Compile: create assembler files from C source files.
%.s: %.c
	$(CC) -S $(ALL_CFLAGS) $< -o $@
%.s: %.cpp
	$(CXX) -S $(ALL_CXXFLAGS) $< -o $@

# Preprocess C files
%.i: %.c
	$(CC) -E $(ALL_CFLAGS) $< -o $@
%.i: %.cpp
	$(CXX) -E $(ALL_CXXFLAGS) $< -o $@

clean:
	rm -f $(ARDUINO)/core.a $(wildcard $(ARDUINO)/*.o)

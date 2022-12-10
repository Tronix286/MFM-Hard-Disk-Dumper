# indent -kr -nut -i3
#
# Execute 'make' to create prucode.bin and mfm_read
# Other options:
# make all
# make pru
# make mfm_read
# make mfm_util
# make clean
#

OBJDIR=obj
INCDIR=inc
INCL_PATH = -I $(INCDIR)/

#LIBRARIES = pthread prussdrv m rt

SOURCES =  mfm_read.c mfm_decoder.c wd_mfm_decoder.c xebec_mfm_decoder.c \
	crc_ecc.c pru_setup.c msg.c parse_cmdline.c analyze.c \
	drive.c deltas_read.c emu_tran_file.c corvus_mfm_decoder.c \
	northstar_mfm_decoder.c board.c drive_read.c tagged_mfm_decoder.c \
        perq_mfm_decoder.c CSerialPort.c
OBJECTS = $(addprefix $(OBJDIR)/, $(SOURCES:.c=.o))
SOURCES2 =  mfm_util.c mfm_decoder.c wd_mfm_decoder.c xebec_mfm_decoder.c \
	crc_ecc.c msg.c parse_cmdline.c emu_tran_file.c corvus_mfm_decoder.c \
	northstar_mfm_decoder.c analyze.c deltas_read_file.c drive_file.c \
        tagged_mfm_decoder.c perq_mfm_decoder.c
OBJECTS2 = $(addprefix $(OBJDIR)/, $(SOURCES2:.c=.o))
SOURCES3 =  mfm_write.c msg.c parse_cmdline.c emu_tran_file.c \
	drive.c pru_setup.c crc_ecc.c board.c drive_write.c
OBJECTS3 = $(addprefix $(OBJDIR)/, $(SOURCES3:.c=.o))
INCLUDES = $(addprefix $(INCDIR)/, analyze.h cmd.h crc_ecc.h deltas_read.h \
	drive.h emu_tran_file.h mfm_decoder.h msg.h parse_cmdline.h \
	pru_setup.h version.h)

CC = gcc

CPP = g++
GOT_CPP := $(shell $(CPP) --version 2>/dev/null)
ifndef GOT_CPP
$(info no c++ compiler found, CRC support program not built)
	CPP = echo c++
endif


CFLAGS = $(EXTRA_DEFINE) $(INCL_PATH) -march=pentium4 -O3 -Wall -D_FILE_OFFSET_BITS=64 -D_XOPEN_SOURCE=600

all : mfm_util find_crc_info mfm_read

mfm_read :  $(OBJECTS)
	$(CC)  $(OBJECTS)  -Wl,-rpath=$(LIB_PATH) $(LIB_PATH:%=-L %) $(LIBRARIES:%=-l%) -o $@
mfm_util :   $(OBJECTS2)
	$(CC)  $(OBJECTS2) $(LIB_PATH:%=-L %) -lm -lrt -liberty -o $@
find_crc_info : find_crc_info.cpp
	$(CPP) -O3 -std=c++0x -Wall $< -o $@

clean :
	rm -rf $(OBJDIR)/*.o *.bin mfm_read mfm_util core *~ find_crc_info

$(OBJDIR)/%.o: %.c $(INCLUDES) | obj
	$(CC) $(CFLAGS) -c -o $@ $<
obj:
	mkdir $@

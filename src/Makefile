LIBNAME	= ../libusb/usbstack
APPNAME = eprom-reader

# Tool definitions
CC      = arm-none-eabi-gcc
LD      = arm-none-eabi-ld -v
AR      = arm-none-eabi-ar
AS      = arm-none-eabi-as
CP      = arm-none-eabi-objcopy
OD		= arm-none-eabi-objdump
RM		= rm

# Tool flags
CFLAGS  = -I./ -I../libusb -c -W -Wall -Os -g -DDEBUG -mcpu=arm7tdmi
ASFLAGS = -ahls -mapcs-32 -o crt.o
LFLAGS  =  -nostartfiles
CPFLAGS = -O ihex
ODFLAGS	= -x --syms

LINKFILE	= lpc2148-rom.ld

CSRCS	= startup.c printf.c console.c
OBJS 	= crt.o $(CSRCS:.c=.o)

all: depend eprom-reader

eprom-reader: $(OBJS) eprom-reader.o serial_fifo.o armVIC.o $(LIBNAME).a
	@ echo "Building $@..."
	$(CC) -T $(LINKFILE) $(LFLAGS) $^ -o $@.elf -Wl,-Map,$@.map
	$(CP) $(CPFLAGS) $@.elf $@.hex
	$(OD) $(ODFLAGS) $@.elf > $@.dmp

crt.o: crt.s
	@ echo ".assembling"
	$(CC) -c $(AFLAGS) -Wa,-ahlms=crt.lst crt.s -o crt.o

clean:
	rm -f *.hex *.elf *.o *.lst *.dmp *.map .depend

# recompile if the Makefile changes
$(OBJS): Makefile

# dependency checking
depend: $(CSRCS)
	$(CC) $(CFLAGS) -MM $^ > .depend || rm -f .depend

# phony targets
.PHONY: clean

-include .depend


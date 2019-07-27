# Makefile to build light_ws2812 library examples
# This is not a very good example of a makefile - the dependencies do not work, therefore everything is rebuilt every time.

# Change these parameters for your device

F_CPU = 9830400
DEVICE = atmega328p
PROGRAMMER = -c usbtiny -P usb
# Tools:
CC = avr-gcc

LIB       = light_ws2812 
EXAMPLES  = Main
DEP		  = Light_WS2812/light_ws2812.c Soft_Uart/softuart.c Seven_Seg/seven_seg.c
AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE)

CFLAGS = -g2 -I. -ILight_WS2812 -ISoft_Uart -ISeven_Seg -mmcu=$(DEVICE) -DF_CPU=$(F_CPU) 
CFLAGS+= -Os -ffunction-sections -fdata-sections -fpack-struct -fno-move-loop-invariants -fno-tree-scev-cprop -fno-inline-small-functions  
CFLAGS+= -Wall -Wno-pointer-to-int-cast
#CFLAGS+= -Wa,-ahls=$<.lst

LDFLAGS = -Wl,--relax,--section-start=.text=0,-Map=main.map

all:	$(EXAMPLES) 


$(EXAMPLES): $(LIB) 
	@echo Building $@
	@$(CC) $(CFLAGS) -o Objects/$@.o Examples/$@.c $(DEP)
	@avr-size Objects/$@.o
	@avr-objcopy -j .text  -j .data -O ihex Objects/$@.o $@.hex
	@avr-objdump -d -S Objects/$@.o >Objects/$@.lss

flash:	all
	$(AVRDUDE) -U flash:w:Main.hex:i

.PHONY:	clean

clean:
	rm -f *.hex Objects/*.o Objects/*.lss

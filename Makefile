# project executable name
PROJECT = blinky

# processor
MCU = attiny10

# cpu frequency
F_CPU = 1000000

# sources
SRC = main.c
# SRC += timer.c

# optimization level
OPT = Os
# OPT = O0

# compiler
CC = avr-gcc

# compiler flags
CFLAGS = -Wall -g -$(OPT) -mmcu=$(MCU) -DF_CPU=$(F_CPU)


all:
	$(CC) $(CFLAGS) -I. $(INCDIR) -o $(PROJECT).bin $(SRC)
	avr-objcopy -O ihex $(PROJECT).bin $(PROJECT).hex
	gtags -q
	avr-size $(PROJECT).bin

flash:
	avrdude -v -c usbasp -p t10 -U flash:w:$(PROJECT).hex:i

clean:
	rm -f *.bin
	rm -f *.hex

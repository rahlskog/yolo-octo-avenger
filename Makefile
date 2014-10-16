DEVICE     = attiny85
CLOCK      = 8000000
PROGRAMMER = stk500v1
PORT	   = /dev/tty.usbmodem1421
BAUD       = 19200
FILENAME   = main
COMPILE    = avr-gcc -Wall -Wextra -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)
 
all: clean build
  
usb:
	ls /dev/cu.*

build:
	$(COMPILE) -c $(FILENAME).c -o $(FILENAME).o
	$(COMPILE) -o $(FILENAME).elf $(FILENAME).o
	avr-objcopy -j .text -j .data -O ihex $(FILENAME).elf $(FILENAME).hex
	avr-size $(FILENAME).elf
	
upload:
	avrdude -v -p $(DEVICE) -c $(PROGRAMMER) -P $(PORT) -b $(BAUD) -U flash:w:$(FILENAME).hex:i 

clean:
	rm main.o
	rm main.elf
	rm main.hex

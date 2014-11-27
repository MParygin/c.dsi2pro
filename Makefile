CFLAGS = -Wall
CC = gcc

all:
	$(CC) -L/usr/include/libusb-1.0 ./dsi2pro.c -lusb-1.0 -lcfitsio -lm -lrt -o dsi2pro

clean:
	rm -f dsi2pro


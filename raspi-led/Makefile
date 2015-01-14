CFLAGS = -Wall

all: raspi-led

raspi-led: main.c ledControl.c
	gcc $(CFLAGS) $^ -o $@

clean:
	rm raspi-led
	rm *~



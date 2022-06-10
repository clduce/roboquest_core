#!/usr/bin/env python3

from serial import Serial
from time import sleep, time

FREQUENCY = 10
UART_PORT = '/dev/ttyUSB0'
UART_DATA_RATE = 57600
TELEM_HEAD = b'$$TELEM '
TELEM_TAIL = b' 0.20 0.10 1.00 1.63 1.63 1.62 0.00 1\n'
SCREEN = '$$SCREEN 1 0\n'

seconds_per_write = 1 / FREQUENCY

uart = Serial(port=UART_PORT,
              baudrate=UART_DATA_RATE,
              bytesize=8,
              parity='N',
              stopbits=1,
              xonxoff=False,
              rtscts=False,
              write_timeout=seconds_per_write,
              timeout=seconds_per_write)

screen = bytearray(SCREEN, 'ascii')

print(f"Frequency: {FREQUENCY}")

cycles = 1

last_write = time()

while True:
    send_more = FREQUENCY
    voltage = 5.0

    while send_more:
        while (time() - last_write) < seconds_per_write:
            sleep(0.01)

        uart.write(TELEM_HEAD)
        uart.write(bytearray(f"{voltage}", 'ascii'))
        uart.write(TELEM_TAIL)
        last_write = time()
        send_more -= 1
        voltage += 1.0

    uart.write(screen)

    print(f"{cycles}")
    cycles += 1

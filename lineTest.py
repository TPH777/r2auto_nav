import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(2, GPIO.IN)

try:
    while True:
        input = GPIO.input(2)
        print(input)
except KeyboardInterrupt:
    GPIO.cleanup()


import Jetson.GPIO as GPIO
import time
import sys

SERVO_MIN_DUTY = 3
SERVO_MAX_DUTY = 12

tilt_pin = 32

up = 150
down = 40

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(tilt_pin, GPIO.OUT)

pwm_1 = GPIO.PWM(tilt_pin, 50)

pwm_1.start((1./18.)*100 + 2)

def tilt_control(deg):
  pos = deg
  dc_1 = (1./18.)*(pos) + 2
  pwm_1.ChangeDutyCycle(dc_1)
  time.sleep(1)


if __name__ == '__main__' :
  mode = sys.argv[1]
  # Fork Up Mode
  if mode == 'up':
    tilt_control(up)

  # Fork Down Mode
  elif mode == 'down' :
    tilt_control(down)


  pwm_1.stop()
  GPIO.cleanup()


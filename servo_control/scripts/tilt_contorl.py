import Jetson.GPIO as GPIO
import time
import sys

SERVO_MIN_DUTY = 3
SERVO_MAX_DUTY = 12

tilt_pin = 33

front_view = 120
object_view = 180
line_view = 155

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
  # Front View Mode
  if mode == 'front':
    tilt_control(front_view)

  # Object View Mode
  elif mode == 'object' :
    tilt_control(object_view)

  # Line View Mode
  elif mode == 'line':
    tilt_control(line_view)

  pwm_1.stop()
  GPIO.cleanup()


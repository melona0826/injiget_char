import rospy
from std_msgs.msg import String
import Jetson.GPIO as GPIO
import time
import sys

SERVO_MIN_DUTY = 3
SERVO_MAX_DUTY = 12

tilt_pin = 33

front_view = 120
object_view = 150
line_view = 140

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


def callback(data) :
  mode = data.data
  # Front View Mode
  if mode == 'front':
    tilt_control(front_view)

  # Object View Mode
  elif mode == 'object' :
    tilt_control(object_view)

  # Line View Mode
  elif mode == 'line':
    tilt_control(line_view)


def move() :
  rospy.init_node('tilt', anonymous=True)
  rospy.Subscriber('/tilt/mode', String, callback)

  rospy.spin()


if __name__ == '__main__' :
  tilt_control(front_view)
  move()




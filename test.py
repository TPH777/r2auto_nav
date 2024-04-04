import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist
import time
import requests

# IR Setup
GPIO.setmode(GPIO.BCM)
leftIr = 2 # Left IR GPIO Pin
rightIr = 3 # Right IR GPIO Pin
lside = 15
rside = 21
GPIO.setup(leftIr, GPIO.IN)
GPIO.setup(rightIr, GPIO.IN)
GPIO.setup(lside, GPIO.IN)
GPIO.setup(rside, GPIO.IN)

# Servo setup to drop
servoPin = 12
GPIO.setup(servoPin, GPIO.OUT)
servo = GPIO.PWM(servoPin, 50)

# constants
maxSpeed = 0.1
reducedSpeed = 0.01
nudgeAngle = 0.5
turnSpeed = 0.05
turnAngle = 1.0 # +ve is left, -ve is right 
turnTime = 1.6
ip_address = "192.168.18.191"

class Mission(Node):
    
    def __init__(self):
        super().__init__('mission')
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10) # Create Publisher
        self.juncCount = 0; # Junction encountered
        
    def httpCall(self):
        url = f'http://{ip_address}/openDoor'
        headers = {'Content-Type': 'application/json'}
        data = '{"action": "openDoor", "parameters": {"robotId": 32}}'
        door_value = ""
        
        try:
            response = requests.post(url, headers=headers, data=data)
            response_data = response.json()
            door_value = response_data.get('data', {}).get('message', door_value)
            print('Status Code: ', response.status_code)
            print(door_value)
            if (door_value == 'door1'):
                self.turnLeft()
            else:
                self.turnRight()
        except requests.exceptions.RequestException as e:
            print('HTTP Request failed', e)
            
    def drop(self):
        print("in drop")
        servo.start(0)
        servo.ChangeDutyCycle(7.5) # Extend servo
        time.sleep(1)
        servo.stop()
        GPIO.cleanup()
    
    def stopBot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        time.sleep(0.1)
        self.publisher_.publish(twist)
        self.juncCount += 1 # Junction encountered +1
        print(self.juncCount)
        time.sleep(2)
# =============================================================================
#         if (self.juncCount == 1):
#             self.httpCall()
#             self.get_logger().info('http')
#         elif (self.juncCount == 2):
#             self.drop()
#             self.get_logger().info('shoot')
#         else: # juncCount = 3
#             #self.autoNav()
#             self.get_logger().info('done')
#         self.juncCount += 1
#         self.get_logger().info('stop')
# =============================================================================

    def turnLeft(self):
        twist = Twist()
        twist.linear.x = turnSpeed
        twist.angular.z = turnAngle
        time.sleep(0.1)
        self.publisher_.publish(twist)
        time.sleep(turnTime)
        
    def turnRight(self):
        twist = Twist()
        twist.linear.x = turnSpeed
        twist.angular.z = -turnAngle
        time.sleep(0.1)
        self.publisher_.publish(twist)
        time.sleep(turnTime)
        
    def nudgeLeft(self):
        twist = Twist()
        twist.linear.x = reducedSpeed
        twist.angular.z = nudgeAngle
        self.publisher_.publish(twist)
        self.get_logger().info('nudge left')  

    def nudgeRight(self):
        twist = Twist()
        twist.linear.x -= reducedSpeed
        twist.angular.z = -nudgeAngle
        self.publisher_.publish(twist)
        self.get_logger().info('nudge right')    
        
    def forward(self):
        twist = Twist()
        twist.linear.x = maxSpeed
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info('forward')   
        
    def irmover(self):
        leftOut = GPIO.input(leftIr) 
        rightOut = GPIO.input(rightIr)
        lOnLine = GPIO.input(lside) 
        rOnLine = GPIO.input(rside) 

        if lOnLine and rOnLine:
            self.stopBot()
        elif rOnLine:
            self.turnRight()
            time.sleep(turnTime)
        elif lOnLine:
            self.turnLeft()
            time.sleep(turnTime)
        elif not leftOut:
            self.nudgeRight()
        elif not rightOut:
            self.nudgeLeft()
        else:
            self.forward()

    def mover(self):
        try:
            while True:
                #self.drop()
                self.irmover()
        except Exception as e:
            print(e)
        finally: # Ctrl-c detected
            self.stopBot() # stop moving
            servo.stop()
            GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    mission = Mission()
    mission.mover()
    mission.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
    
    
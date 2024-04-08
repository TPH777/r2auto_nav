import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist
import time
import requests

# IR Setup
GPIO.setmode(GPIO.BCM)
leftIr = 26 # Left IR GPIO Pin
rightIr = 2 # Right IR GPIO Pin
GPIO.setup(leftIr, GPIO.IN)
GPIO.setup(rightIr, GPIO.IN)

# Servo setup to drop
servoPin = 12
GPIO.setup(servoPin, GPIO.OUT)
servo = GPIO.PWM(servoPin, 50)

# constants
maxSpeed = -0.21
reducedSpeed = -0.01
nudgeAngle = 2.3
turnSpeed = -0.05
turnAngle = 1.3 # +ve is left, -ve is right 
turnTime = 1.0
ip_address = "192.168.181.191"

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
            self.httpCall()
            
    def drop(self):
        servo.start(0)
        servo.ChangeDutyCycle(7.5) # Extend servo
        time.sleep(2)
        servo.ChangeDutyCycle(2.5)
        time.sleep(1)
    
    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        time.sleep(0.1)
        self.publisher_.publish(twist)
        self.juncCount += 1 # Junction encountered +1
        if (self.juncCount == 1):
            self.httpCall()
        elif (self.juncCount == 2):
            self.drop()


    def turnLeft(self):
        twist = Twist()
        twist.linear.x = turnSpeed
        twist.angular.z = turnAngle
        time.sleep(0.2)
        self.publisher_.publish(twist)
        time.sleep(turnTime)
        
    def turnRight(self):
        twist = Twist()
        twist.linear.x = turnSpeed
        twist.angular.z = -turnAngle
        time.sleep(0.2)
        self.publisher_.publish(twist)
        time.sleep(turnTime)
        
    def nudgeLeft(self):
        twist = Twist()
        twist.linear.x = reducedSpeed
        twist.angular.z = nudgeAngle
        self.publisher_.publish(twist) 

    def nudgeRight(self):
        twist = Twist()
        twist.linear.x -= reducedSpeed
        twist.angular.z = -nudgeAngle
        self.publisher_.publish(twist)  
        
    def forward(self):
        twist = Twist()
        twist.linear.x = maxSpeed
        twist.angular.z = 0.0
        self.publisher_.publish(twist) 
        
    def irmover(self):
        leftOnLine = GPIO.input(leftIr) 
        rightOnLine = GPIO.input(rightIr)
        
        if leftOnLine:
            if rightOnLine:
                self.stop()
                print('stop')
            else:
                self.nudgeLeft()
                print('nudge left')
        else:
            if rightOnLine:
                self.nudgeRight()
                print('nudge right')
            else:
                self.forward() 
                print('forward')

    def mover(self):
        try:
            while True:
                if self.juncCount == 2:
                    break
                self.irmover()
        except Exception as e:
            print(e)
        finally: # Ctrl-c detected
            self.stop() # stop moving
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
    
    
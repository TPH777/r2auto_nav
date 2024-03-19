# Create new package (setup.py, package.xml)
# Change .bashrc alias
# Calibrate nudging (Speed and angle, can deal with sharp turn?)

import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist
import time

# Setup
leftIr = 2 # Left IR GPIO Pin
rightIr = 3 # Right IR GPIO Pin
GPIO.setmode(GPIO.BCM)
GPIO.setup(leftIr, GPIO.IN)
GPIO.setup(rightIr, GPIO.IN)

motor = 4
GPIO.setup(motor, GPIO.OUT)

# constants
maxSpeed = 0.2
reducedSpeed = 0.01
nudgeAngle = 2.0
turnSpeed = 0.1
turnAngle = 1.0 # +ve is left, -ve is right 
turnTime = 1.0

class Mission(Node):
    
    def __init__(self):
        super().__init__('mission')
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10) # Create Publisher
        self.juncCount = 0; # Junction encountered
    
# =============================================================================
#     def shoot(self):
#         GPIO.output(motor,1)
# =============================================================================
    
    def stopBot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.juncCount += 1 # Junction encountered +1
        if (self.juncCount == 1):
            #self.httpCall()
            self.get_logger().info('http')
        elif (self.juncCount == 2):
            #self.shoot()
            self.get_logger().info('shoot')
        else: # juncCount = 3
            #self.autoNav()
            self.get_logger().info('done')
        self.juncCount += 1
        self.get_logger().info('stop')

    def turnLeft(self):
        twist = Twist()
        twist.linear.x = turnSpeed
        twist.angular.z = turnAngle
        self.publisher_.publish(twist)
        time.sleep(turnTime)
        
    def turnRight(self):
        twist = Twist()
        twist.linear.x = turnSpeed
        twist.angular.z = -turnAngle
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
        leftOnLine = GPIO.input(leftIr) 
        rightOnLine = GPIO.input(rightIr)

        if (leftOnLine): # 1 if on line, 0 if not on line
             if(rightOnLine):
                 self.stopBot()
             else:
                 self.nudgeLeft()
        else:
             if (rightOnLine):
                 self.nudgeRight()
             else:
                 self.forward()
                   
    def mover(self):
        try:
            while True:
                self.irmover()
               # GPIO.output(motor,1)
        except Exception as e:
            print(e)
        finally: # Ctrl-c detected
            self.stopBot() # stop moving
            GPIO.output(motor,0)

def main(args=None):
    rclpy.init(args=args)
    mission = Mission()
    mission.mover()
    mission.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
    
    
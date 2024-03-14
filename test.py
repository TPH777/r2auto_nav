import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist

# Setup
leftIr = 0 # Left IR GPIO Pin
rightIr = 0 # Right IR GPIO Pin
GPIO.setmode(GPIO.BOARD)
GPIO.setup(leftIr, GPIO.IN)
GPIO.setup(rightIr, GPIO.IN)

# constants
rotatechange = 0.1
speedchange = 0.05
maxSpeed = 0.5
perpendicular = 1



class Mission(Node):
    
    def __init__(self):
        super().__init__('auto_nav')
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10) # Create Publisher
        self.juncCount = 0; # Junction encountered
    
    def stopBot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
# =============================================================================
#         self.juncCount += 1 # Junction encountered +1
#         if (juncCount == 1):
#             self.httpCall()
#         elif (juncCount == 2):
#             self.shoot()
#         else: # juncCount = 3
#             self.autoNav()
# =============================================================================
    
    def turnLeft(self):
        twist = Twist()
        twist.linear.x = maxSpeed
        self.publisher_.publish(twist)
        time.sleep(forwardTime)
        twist.angular.z = perpendicular
        self.publisher_.publist(twist)
        time.sleep(turnTime)
        
    def nudgeLeft(self):
        twist = Twist()
        twist.linear.x -= speedchange
        twist.angular.z -= rotatechange
        self.publisher_.publish(twist)

    def nudgeRight(self):
        twist = Twist()
        twist.linear.x -= speedchange
        twist.angular.z += rotatechange
        self.publisher_.publish(twist)
        
    def forward(self):
        twist = Twist()
        twist.linear.x = maxSpeed
        twist.angular.z = 0
        self.publisher_.publish(twist)
        
    def irmover(self):
        leftOnLine = GPIO.input(leftIr) # 1 if out, 0 if in
        rightOnLine = GPIO.input(rightIr)
        self.get_logger().info('leftOut')
        self.get_logger().info('rightOut')
# =============================================================================
#         if (leftOnLine):
#             if(rightOnLine):
#                 self.stopBot()
#                 self.get_logger().info('stop')
#             else:
#                 self.nudgeLeft()
#                 self.get_logger().info('nudge left')
#         else:
#             if (rightOnLine):
#                 self.nudgeRight()
#                 self.get_logger().info('nudge right')
#             else:
#                 self.forward()
# =============================================================================
            
        
    def mover(self):
        try:
            self.irmover()
            while rclpy.ok():               
                rclpy.spin_once(self) # allow the callback functions to run
        except Exception as e:
            print(e)
        finally: # Ctrl-c detected
            self.stopbot() # stop moving

def main(args=None):
    # rclpy.init(args=args)
    mission = Mission()
    mission.mover()
    mission.destroy_node()
    # rclpy.shutdown()
        
if __name__ == '__main__':
    main()
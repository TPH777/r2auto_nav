import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from PIL import Image
import math
import scipy.stats
import time
import heapq

# constants
occ_bins = [-1, 0, 50, 100]
map_bg_color = 1

#def euler_from_quaternion(x, y, z, w):
#    """
#    Convert a quaternion into euler angles (roll, pitch, yaw)
#    roll is rotation around x in radians (counterclockwise)
#    pitch is rotation around y in radians (counterclockwise)
#    yaw is rotation around z in radians (counterclockwise)
#    """
#    t0 = +2.0 * (w * x + y * z)
#    t1 = +1.0 - 2.0 * (x * x + y * y)
#    roll_x = math.atan2(t0, t1)
#
#    t2 = +2.0 * (w * y - z * x)
#    t2 = +1.0 if t2 > +1.0 else t2
#    t2 = -1.0 if t2 < -1.0 else t2
#    pitch_y = math.asin(t2)
#
#    t3 = +2.0 * (w * z + x * y)
#    t4 = +1.0 - 2.0 * (y * y + z * z)
#    yaw_z = math.atan2(t3, t4)
#
#    return roll_x, pitch_y, yaw_z # in radians
#

####
#def heuristic(node, target):
#    return (node[0] - target[0])**2 + (node[1] - target[1])**2
#
#def find_neighbors(node, occupancy_data, number_of_rows, number_of_cols):
#    neighbors = []
#    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
#    for dir in directions:
#        neighbor = (node[0] + dir[0], node[1] + dir[1])
#        if 0 <= neighbor[0] < number_of_rows and 0 <= neighbor[1] < number_of_cols:
#            if occupancy_data[neighbor[0], neighbor[1]] != 3:  # Check if it is not wall
#                neighbors.append(neighbor)
#    return neighbors
#
#def a_star(node, target, occupancy_data, number_of_rows, number_of_cols):
#    open_set = []
#    closed_set = set()
#    heapq.heappush(open_set, (0, node))
#    came_from = {}
#
#    g_score = {node: 0}
#    f_score = {node: heuristic(node, target)}
#
#    while open_set:
#        current = heapq.heappop(open_set)[1]
#
#        if current == target:
#            path = []
#            while current in came_from:
#                path.append(current)
#                current = came_from[current]
#            path.append(node)
#            path.reverse()
#            return path
#
#        closed_set.add(current)
#
#        for neighbor in find_neighbors(current, occupancy_data, number_of_rows, number_of_cols):
#            if neighbor in closed_set: 
#                continue
#
#            tentative_g_score = g_score[current] + 1  # Assuming cost of moving from one cell to another is 1
#
#            if neighbor not in [x[1] for x in open_set] or tentative_g_score < g_score.get(neighbor, float('inf')):
#                came_from[neighbor] = current
#                g_score[neighbor] = tentative_g_score
#                f_score[neighbor] = tentative_g_score + heuristic(neighbor, target)
#                heapq.heappush(open_set, (f_score[neighbor], neighbor))
#
#    return None  # No path found
####
    
class PriorityQueue():
    def __init__(self):
        self.elements = []
        
    def top(self):
        if self.elements:
            return self.elements[0][1]
        else:
            return None
    
    def pop(self):
        if self.elements:
            return heapq.heappop(self.elements)[1]
        else:
            return None
    
    def insert(self, s, k):
        heapq.heappush(self.elements, (k, s))
        
    def top_key(self):
        if self.elements:
            return self.elements[0]
        else:
            return (float('inf'),float('inf'))
        
    def update(self, s, k):
        # Search for the vertex s in the priority queue
        for i, (priority, vertex) in enumerate(self.elements):
            if vertex == s:
                # Update the priority of the vertex
                self.elements[i] = (k, vertex)
                # Re-heapify the priority queue after updating the element
                heapq.heapify(self.elements)
                return
            
    def remove(self, s):
        # Search for the vertex s in the priority queue
        for i, (priority, vertex) in enumerate(self.elements):
            if vertex == s:
                # Remove the element at index i
                del self.elements[i]
                # Re-heapify the priority queue after removing the element
                heapq.heapify(self.elements)
                return

class Graph():
    def __init__(self, occupancy_data, number_of_rows, number_of_cols): # TO BE ADDED
        self.vertices = set()  # Set to store the vertices
        self.edges = {}  # Dictionary to store the edges and their costs
        self.row = number_of_rows
        self.col = number_of_cols
        self.occupancy_data = occupancy_data

    def add_vertex(self, vertex):
        self.vertices.add(vertex)

    def get_vertices(self):
        return self.vertices
    
    def add_edge(self, start, end):
        if start not in self.vertices or end not in self.vertices:
            raise ValueError("Vertices not in graph")
        self.edges[(start, end)] = heuristic(start, end)
        
    def get_edges(self):
        return self.edges

    def get_predecessors(self, vertex):
        return [start for start, end in self.edges if end == vertex]
    
    def get_successors(self, vertex):
        return [end for start, end in self.edges if start == vertex]

    def g_score(self, start, end):
        return self.edges.get((start, end), float('inf'))

    def cost(self, start, end):
        return self.edges.get((start, end), float('inf'))
    
    def rhs_value(self, s, node):
        if s == node:
            return 0
        else:
            pred = self.get_predecessors(s)
            if not pred:
                return float('inf')  # No predecessors, set to infinity
            else:
                return min(self.g_score(s_prime, s) + self.cost(s_prime, s) for s_prime in pred)

def heuristic(start, target) -> float:
    x_distance = abs(int(start.split('x')[1][0]) - int(target.split('x')[1][0]))
    y_distance = abs(int(start.split('y')[1][0]) - int(target.split('y')[1][0]))
    return (x_distance**2 + y_distance**2)**(0.5)
    
class D_Star_Lite():
    def __init__(self, graph: Graph(occupancy_data, number_of_rows, number_of_cols), s_start, s_goal):
        self.s_start = s_start
        self.s_goal = s_goal
        self.s_last = s_start
        self.k_m = 0  # accumulation
        self.U = PriorityQueue()
        
        for s in (graph.get_predecessors(s)+graph.get_successors(s)):
            self.S = graph.get_predecessors(s)+graph.get_successors(s)
            graph.rhs_value(s, s_start) = float('inf')
            graph.g_score(s_start, s) = graph.rhs_value(s, s_start).copy()
            
        graph.rhs_value(self.s_last, s_start) = 0
        self.U.insert(self.s_goal, (heuristic(self.s_start, self.s_goal), 0))
        
        self.new_edges_and_old_costs = None
    
    def calculate_key(self, s):
        return (min(graph.g_score(self.s_start,s), graph.rhs_value(s,self.s_start)) + heuristic(s, s_start) + self.k_m, min(graph.g_score(s_start,s), graph.rhs_value(s,s_start)))

    def update_vertex(self, s):
        if (graph.g_score(self.s_start,s) != graph.rhs_value(s,self.s_start)) and (s in self.S):
            self.U.update(s,calculate_key(s))
        elif (graph.g_score(self.s_start,s) != graph.rhs_value(s,self.s_start)) and (s not in self.S):
            self.U.insert(s,calculate_key(s))
        elif (graph.g_score(self.s_start,s) == graph.rhs_value(s,self.s_start)) and (s in self.S):
            self.U.remove(s)
    
    def compute_shortest_path(self):
        while self.U.top_key() < self.calculate_key(self.s_start) or graph.rhs_value(self.s_start, self.s_start) > graph.g_score(self.s_start, self.s_start):
            u = self.U.top()
            k_old = self.U.top_key()
            k_new = self.calculate_key(u)

            if k_old < k_new:
                self.U.update(u, k_new)
            elif graph.g_score(u, self.s_start) > graph.rhs_value(u, self.s_start):
                graph.g_score(u, self.s_start) = graph.rhs_value(u, self.s_start)
                self.U.remove(u)
                pred = graph.get_predecessors(u)
                for s in pred:
                    if s != self.s_goal:
                        graph.rhs_value(s, self.s_start) = min(graph.rhs_value(s, self.s_start), graph.cost(s, u) + graph.g_score(u, self.s_start))
                    self.update_vertex(s)
            else:
                self.g_old = graph.g_score(u, self.s_start)
                graph.g_score(u, self.s_start) = float('inf')
                pred = graph.get_predecessors(u)
                pred.append(u)
                for s in pred:
                    if graph.rhs_value(s, self.s_start) == graph.cost(s, u) + self.g_old:
                        if s != self.s_goal:
                            min_s = float('inf')
                            succ = graph.get_successors(s)
                            for s_ in succ:
                                temp = graph.cost(s, s_) + graph.g_score(s_, self.s_start)
                                if min_s > temp:
                                    min_s = temp
                            graph.rhs_value(s, self.s_start) = min_s
                    self.update_vertex(u)
                    
    def rescan(self):
        if self.new_edges_and_old_costs:
            for vertex in self.new_edges_and_old_costs:
                graph.add_vertex(vertex)
        new_edges_and_old_costs = self.new_edges_and_old_costs
        self.new_edges_and_old_costs = None
        return new_edges_and_old_costs

    def move_and_replan(self, robot_position):
        path = [robot_position]
        self.s_start = robot_position
        self.s_last = self.s_start
        self.compute_shortest_path()

        while self.s_start != self.s_goal:
            assert (graph.rhs_values(self.s_start, self.s_start) != float('inf')), "There is no known path!"

            succ = graph.get_successors(self.s_start)
            min_s = float('inf')
            arg_min = None
            for s_ in succ:
                temp = graph.cost(self.s_start, s_) + graph.g_score(s_, self.s_start)
                if temp < min_s:
                    min_s = temp
                    arg_min = s_

            ### algorithm sometimes gets stuck here for some reason !!! FIX
            self.s_start = arg_min
            path.append(self.s_start)
            # scan graph for changed costs
            changed_edges_with_old_cost = self.rescan()
            #print("len path: {}".format(len(path)))
            # if any edge costs changed
            if changed_edges_with_old_cost:
                self.k_m += heuristic(self.s_last, self.s_start)
                self.s_last = self.s_start

                # for all directed edges (u,v) with changed edge costs
                vertices = changed_edges_with_old_cost.get_vertices()
                for vertex in vertices:
                    succ_v = changed_edges_with_old_cost.get_successors(vertex)
                    for u, c_old in succ_v.items():
                        c_new = graph.cost(u, vertex)
                        if c_old > c_new:
                            if u != self.s_goal:
                                graph.rhs_value(u, self.s_start) = min(graph.rhs_value(u, self.s_start), graph.cost(u, vertex) + graph.g_score(vertex, self.s_start))
                        elif graph.rhs_value(u, self.s_start) == c_old + graph.g_score(vertex, self.s_start):
                            if u != self.s_goal:
                                min_s = float('inf')
                                succ_u = graph.get_successors(u)
                                for s_ in succ_u:
                                    temp = graph.cost(u, s_) + graph.g_score(s_, self.s_start)
                                    if min_s > temp:
                                        min_s = temp
                                graph.rhs_value(u, self.s_start) = min_s
                            self.update_vertex(u)
            self.compute_shortest_path()
        print("path found!")
        return path, self.g, self.rhs
        

#class Occupy(Node):
#    def __init__(self):
#        super().__init__('occupy')
#        self.occupancy_subscription = self.create_subscription(
#            OccupancyGrid,
#            'map',
#            self.listener_callback,
#            qos_profile_sensor_data)
#        self.occupancy_subscription # prevent unused variable warning
#        
#        self.scan_subscription = self.create_subscription(
#            LaserScan,
#            'scan',
#            self.listener_callback,
#            qos_profile_sensor_data)
#        self.scan_subscription  # prevent unused variable warning
#        
#        # create subscription to track orientation
#        self.odom_subscription = self.create_subscription(
#            Odometry,
#            'odom',
#            self.odom_callback,
#            10)
#        # self.get_logger().info('Created subscriber')
#        self.odom_subscription  # prevent unused variable warning
#        
#        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
#        self.tfBuffer = tf2_ros.Buffer()
#        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
#        
#        # initialize variables
#        self.roll = 0
#        self.pitch = 0
#        self.yaw = 0
#        self.x = 0
#        self.y = 0
#
#    def listener_callback(self, msg):
#        # create numpy array
#        occdata = np.array(msg.data)
#        # compute histogram to identify bins with -1, values between 0 and below 50, 
#        # and values between 50 and 100. The binned_statistic function will also
#        # return the bin numbers so we can use that easily to create the image 
#        occ_counts, edges, binnum = scipy.stats.binned_statistic(occdata, np.nan, statistic='count', bins=occ_bins)
#        # get width and height of map
#        iwidth = msg.info.width
#        iheight = msg.info.height
#        # calculate total number of bins
#        total_bins = iwidth * iheight
#        # log the info
#        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0], occ_counts[1], occ_counts[2], total_bins))
#
#        # find transform to obtain base_link coordinates in the map frame
#        # lookup_transform(target_frame, source_frame, time)
#        try:
#            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
#        except (LookupException, ConnectivityException, ExtrapolationException) as e:
#            self.get_logger().info('No transformation found')
#            return
#            
#        cur_pos = trans.transform.translation
#        cur_rot = trans.transform.rotation
#        # self.get_logger().info('Trans: %f, %f' % (cur_pos.x, cur_pos.y))
#        # convert quaternion to Euler angles
#        roll, pitch, yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
#        # self.get_logger().info('Rot-Yaw: R: %f D: %f' % (yaw, np.degrees(yaw)))
#
#        # get map resolution
#        map_res = msg.info.resolution
#        # get map origin struct has fields of x, y, and z
#        map_origin = msg.info.origin.position
#        # get map grid positions for x, y position
#        grid_x = round((cur_pos.x - map_origin.x) / map_res)
#        grid_y = round(((cur_pos.y - map_origin.y) / map_res))
#        # self.get_logger().info('Grid Y: %i Grid X: %i' % (grid_y, grid_x))
#
#        # binnum go from 1 to 3 so we can use uint8
#        # convert into 2D array using column order
#        odata = np.uint8(binnum.reshape(msg.info.height,msg.info.width))
#        # set current robot location to 0
#        odata[grid_y][grid_x] = 0
#        # create image from 2D array using PIL
#        img = Image.fromarray(odata)
#        # find center of image
#        i_centerx = iwidth/2
#        i_centery = iheight/2
#        # find how much to shift the image to move grid_x and grid_y to center of image
#        shift_x = round(grid_x - i_centerx)
#        shift_y = round(grid_y - i_centery)
#        # self.get_logger().info('Shift Y: %i Shift X: %i' % (shift_y, shift_x))
#
#        # pad image to move robot position to the center
#        # adapted from https://note.nkmk.me/en/python-pillow-add-margin-expand-canvas/ 
#        left = 0
#        right = 0
#        top = 0
#        bottom = 0
#        if shift_x > 0:
#            # pad right margin
#            right = 2 * shift_x
#        else:
#            # pad left margin
#            left = 2 * (-shift_x)
#            
#        if shift_y > 0:
#            # pad bottom margin
#            bottom = 2 * shift_y
#        else:
#            # pad top margin
#            top = 2 * (-shift_y)
#            
#        # create new image
#        new_width = iwidth + right + left
#        new_height = iheight + top + bottom
#        img_transformed = Image.new(img.mode, (new_width, new_height), map_bg_color)
#        img_transformed.paste(img, (left, top))
#
#        # rotate by 90 degrees so that the forward direction is at the top of the image
#        rotated = img_transformed.rotate(np.degrees(yaw)-90, expand=True, fillcolor=map_bg_color)
#
#        # show the image using grayscale map
#        # plt.imshow(img, cmap='gray', origin='lower')
#        # plt.imshow(img_transformed, cmap='gray', origin='lower')
#        plt.imshow(rotated, cmap='gray', origin='lower')
#        plt.draw_all()
#        # pause to make sure the plot gets created
#        plt.pause(0.00000000001)
#        
#    def odom_callback(self, msg):
#        # self.get_logger().info('In odom_callback')
#        orientation_quat =  msg.pose.pose.orientation
#        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
#        
#    def scan_callback(self, msg):
#        # self.get_logger().info('In scan_callback')
#        # create numpy array
#        self.laser_range = np.array(msg.ranges)
#        # print to file
#        np.savetxt(scanfile, self.laser_range)
#        # replace 0's with nan
#        self.laser_range[self.laser_range==0] = np.nan
#        
#    # function to rotate the TurtleBot
#    def rotatebot(self, rot_angle):
#        # self.get_logger().info('In rotatebot')
#        # create Twist object
#        twist = Twist()
#        
#        # get current yaw angle
#        current_yaw = self.yaw
#        # log the info
#        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
#        # we are going to use complex numbers to avoid problems when the angles go from
#        # 360 to 0, or from -180 to 180
#        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
#        # calculate desired yaw
#        target_yaw = current_yaw + math.radians(rot_angle)
#        # convert to complex notation
#        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
#        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
#        # divide the two complex numbers to get the change in direction
#        c_change = c_target_yaw / c_yaw
#        # get the sign of the imaginary component to figure out which way we have to turn
#        c_change_dir = np.sign(c_change.imag)
#        # set linear speed to zero so the TurtleBot rotates on the spot
#        twist.linear.x = 0.0
#        # set the direction to rotate
#        twist.angular.z = c_change_dir * rotatechange
#        # start rotation
#        self.publisher_.publish(twist)
#
#        # we will use the c_dir_diff variable to see if we can stop rotating
#        c_dir_diff = c_change_dir
#        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
#        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
#        # becomes -1.0, and vice versa
#        while(c_change_dir * c_dir_diff > 0):
#            # allow the callback functions to run
#            rclpy.spin_once(self)
#            current_yaw = self.yaw
#            # convert the current yaw to complex form
#            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
#            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
#            # get difference in angle between current and target
#            c_change = c_target_yaw / c_yaw
#            # get the sign to see if we can stop
#            c_dir_diff = np.sign(c_change.imag)
#            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
#
#        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
#        # set the rotation speed to 0
#        twist.angular.z = 0.0
#        # stop the rotation
#        self.publisher_.publish(twist)
#        
#    def stopbot(self):
#        self.get_logger().info('In stopbot')
#        # publish to cmd_vel to move TurtleBot
#        twist = Twist()
#        twist.linear.x = 0.0
#        twist.angular.z = 0.0
#        # time.sleep(1)
#        self.publisher_.publish(twist)
#
#
#def main(args=None):
#    rclpy.init(args=args)
#
#    occupy = Occupy()
#
#    # create matplotlib figure
#    plt.ion()
#    plt.show()
#
#    rclpy.spin(occupy)
#
#    # Destroy the node explicitly
#    # (optional - otherwise it will be done automatically
#    # when the garbage collector destroys the node object)
#    occupy.destroy_node()
#    rclpy.shutdown()
#
#
#if __name__ == '__main__':
#    main()
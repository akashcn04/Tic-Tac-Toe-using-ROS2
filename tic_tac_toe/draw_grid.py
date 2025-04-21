import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen
from std_srvs.srv import Empty
import time
import math

class GridDrawer(Node):
    def __init__(self):
        super().__init__('grid_drawer')
        # Publishers and clients
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.clear_client = self.create_client(Empty, '/clear')
        
        # Wait for services
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teleport service...')
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for pen service...')
        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for clear service...')
            
        # Clear the screen first
        self.clear_screen()
        
        # Initialize turtle
        self.stop_turtle()
        self.set_pen(255, 255, 255, 4, 0)  # White pen with width 4
        time.sleep(1.0)
        
        # Draw the grid
        self.draw_grid()
    
    def clear_screen(self):
        req = Empty.Request()
        future = self.clear_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(0.5)
    
    def stop_turtle(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        for _ in range(5):
            self.publisher.publish(msg)
            time.sleep(0.1)
    
    def set_pen(self, r, g, b, width, off):
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        future = self.pen_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(0.1)
    
    def teleport(self, x, y, theta=0.0):
        self.stop_turtle()
        req = TeleportAbsolute.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        future = self.teleport_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(0.5)
    
    def pen_up(self):
        self.set_pen(255, 255, 255, 4, 1)  # Pen up (off=1)
    
    def pen_down(self):
        self.set_pen(255, 255, 255, 4, 0)  # Pen down (off=0)
    
    def draw_straight_line(self, length, speed=1.0):
        """Draw a straight line in the current direction"""
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = 0.0
        
        start_time = time.time()
        duration = length / speed
        
        while time.time() - start_time < duration:
            self.publisher.publish(msg)
            time.sleep(0.01)
        
        self.stop_turtle()
    
    def draw_grid(self):
        # Grid parameters
        grid_size = 6.0      # Size of the grid
        margin = 2.0         # Margin from edges
        cell_count = 3       # 3x3 grid
        
        # Calculate positions
        start_x = margin
        start_y = margin
        end_x = start_x + grid_size
        end_y = start_y + grid_size
        
        # Calculate cell size
        cell_size = grid_size / cell_count
        
        # Draw horizontal lines
        for i in range(cell_count + 1):
            y_pos = start_y + i * cell_size
            
            # Teleport to start position
            self.pen_up()
            self.teleport(start_x, y_pos, 0.0)  # Face right (0 radians)
            
            # Draw line
            self.pen_down()
            self.draw_straight_line(grid_size)
            self.pen_up()
        
        # Draw vertical lines
        for i in range(cell_count + 1):
            x_pos = start_x + i * cell_size
            
            # Teleport to start position
            self.pen_up()
            self.teleport(x_pos, start_y, math.pi/2)  # Face up (π/2 radians)
            
            # Draw line
            self.pen_down()
            self.draw_straight_line(grid_size)
            self.pen_up()
        
        # Move turtle out of the way
        self.pen_up()
        self.teleport(9.0, 9.0, 0.0)
        
        self.get_logger().info("✅ Finished drawing Tic Tac Toe grid!")

def main(args=None):
    rclpy.init(args=args)
    node = GridDrawer()
    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

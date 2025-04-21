# player_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from std_msgs.msg import String
import sys
import termios
import tty
import threading
import time

class PlayerControlNode(Node):
    def __init__(self):
        super().__init__('player_control_node')
        
        # Create publishers
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Create subscribers
        self.pose_subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)
        
        self.game_state_subscriber = self.create_subscription(
            String,
            '/game_state',
            self.game_state_callback,
            10)
        
        # Create service clients
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        
        # Initialize variables
        self.pose = None
        self.game_active = True
        self.speed = 0.5
        
        self.get_logger().info('Player controller initialized!')
        self.get_logger().info('Use WASD keys to move the turtle to a cell and press SPACE to select it')
        
        # Start keyboard input thread
        threading.Thread(target=self.keyboard_input_loop, daemon=True).start()
        
    def pose_callback(self, msg):
        self.pose = msg
        
    def game_state_callback(self, msg):
        if "Game over" in msg.data:
            self.game_active = False
            self.get_logger().info(msg.data)
            
    def teleport_turtle(self, x, y, theta):
        request = TeleportAbsolute.Request()
        request.x = float(x)
        request.y = float(y)
        request.theta = float(theta)
        self.teleport_client.call_async(request)
        
    def get_key(self):
        # Get keyboard input without requiring Enter key
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    def keyboard_input_loop(self):
        while True:
            if not self.game_active:
                time.sleep(0.1)
                continue
                
            key = self.get_key()
            
            twist = Twist()
            
            # Handle movement keys
            if key == 'w':
                twist.linear.x = self.speed
                twist.linear.y = 0.0
            elif key == 's':
                twist.linear.x = -self.speed
                twist.linear.y = 0.0
            elif key == 'a':
                twist.linear.x = 0.0
                twist.linear.y = self.speed
            elif key == 'd':
                twist.linear.x = 0.0
                twist.linear.y = -self.speed
            elif key == ' ':  # Space to select current cell
                # Just sending empty velocity to trigger position check
                pass
            elif key == '\x03':  # Ctrl+C
                break
                
            self.velocity_publisher.publish(twist)
            time.sleep(0.1)  # Small delay to prevent flooding

def main(args=None):
    rclpy.init(args=args)
    node = PlayerControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

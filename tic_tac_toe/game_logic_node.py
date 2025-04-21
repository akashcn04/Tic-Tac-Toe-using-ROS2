#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen, Kill, Spawn
import time
import math
import threading
import sys
import termios
import tty

class TicTacToeGameNode(Node):
    def __init__(self):
        super().__init__('tic_tac_toe_game')
        
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Create subscribers
        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)
        
        # Create service clients
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.kill_client = self.create_client(Kill, '/kill')
        self.spawn_client = self.create_client(Spawn, '/spawn')
        
        # Wait for services to be available
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Teleport service not available, waiting...')
        while not self.set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set pen service not available, waiting...')
        
        # Ask for grid size (default to 3 if invalid)
        self.get_grid_size()
        
        # Initialize game state based on grid size
        self.board = [' ' for _ in range(self.grid_size * self.grid_size)]
        self.current_player = 'X'  # X = human, O = computer
        self.game_active = True
        self.pose = None
        
        # Calculate grid dimensions
        self.grid_start_x = 1.0
        self.grid_start_y = 1.0
        self.grid_end_x = 10.0
        self.grid_end_y = 10.0
        self.cell_size = min((self.grid_end_x - self.grid_start_x) / self.grid_size, 
                             (self.grid_end_y - self.grid_start_y) / self.grid_size)
        
        # Calculate cell positions (center of each cell)
        self.calculate_cell_positions()
        
        # Draw the grid first
        self.draw_grid()
        
        # Configure initial turtle state
        self.set_pen(False)  # Ensure pen is off
        center_x = (self.grid_start_x + self.grid_end_x) / 2
        center_y = (self.grid_start_y + self.grid_end_y) / 2
        self.teleport_turtle(center_x, center_y, 0.0)  # Start position
        
        # Start keyboard input thread
        self.running = True
        self.input_thread = threading.Thread(target=self.get_keyboard_input)
        self.input_thread.daemon = True
        self.input_thread.start()
        
        # Create the main game loop timer
        self.timer = self.create_timer(0.1, self.game_loop)
        
        # Flag for reset request
        self.reset_requested = False
        # Flag for quit request
        self.quit_requested = False
        
        self.get_logger().info(f'Tic Tac Toe game started with {self.grid_size}x{self.grid_size} grid!')
        self.get_logger().info('Use WASD keys to move: w=up, a=left, s=down, d=right')
        self.get_logger().info('Press m to mark a cell with X')
        self.get_logger().info('Press r to reset the game')
        self.get_logger().info('Press q to quit')
    
    def calculate_cell_positions(self):
        """Calculate positions for each cell in n×n grid"""
        self.cell_positions = []
        for row in range(self.grid_size):
            for col in range(self.grid_size):
                x = self.grid_start_x + (col + 0.5) * self.cell_size
                y = self.grid_end_y - (row + 0.5) * self.cell_size
                self.cell_positions.append((x, y))
    
    def get_grid_size(self):
        """Ask the user for grid size and validate it"""
        self.get_logger().info('Enter grid size (n for n×n grid, 3-9 recommended): ')
        
        try:
            # Allow user to input grid size
            size = int(input())
            if size < 2:
                self.get_logger().warn('Grid size must be at least 2. Using default size 3.')
                self.grid_size = 3
            elif size > 10:
                self.get_logger().warn('Grid size larger than 10 may not display well. Using maximum size 10.')
                self.grid_size = 10
            else:
                self.grid_size = size
        except ValueError:
            self.get_logger().warn('Invalid input. Using default grid size 3.')
            self.grid_size = 3
        
        self.get_logger().info(f'Using grid size: {self.grid_size}x{self.grid_size}')
    
    def draw_grid(self):
        # Set pen to white for grid lines
        self.set_pen(True, 255, 255, 255, 2)
        
        # Draw horizontal lines
        for i in range(self.grid_size + 1):
            y = self.grid_end_y - i * self.cell_size
            # Move to start position with pen off
            self.set_pen(False)
            self.teleport_turtle(self.grid_start_x, y, 0.0)
            # Turn pen on for drawing
            self.set_pen(True, 255, 255, 255, 2)
            # Draw line
            self.teleport_turtle(self.grid_start_x + self.cell_size * self.grid_size, y, 0.0)
        
        # Draw vertical lines
        for i in range(self.grid_size + 1):
            x = self.grid_start_x + i * self.cell_size
            # Move to start position with pen off
            self.set_pen(False)
            self.teleport_turtle(x, self.grid_end_y, 0.0)
            # Turn pen on for drawing
            self.set_pen(True, 255, 255, 255, 2)
            # Draw line
            self.teleport_turtle(x, self.grid_end_y - self.cell_size * self.grid_size, 0.0)
        
        # Ensure pen is off after drawing grid
        self.set_pen(False)
    
    def clear_screen_with_background(self):
        # Instead of killing and respawning the turtle, we'll draw a background
        # rectangle that covers the entire game area to "erase" previous marks
        
        # Save current position
        save_x, save_y = 5.5, 5.5
        if self.pose:
            save_x, save_y = self.pose.x, self.pose.y
        
        # Set pen to background color (light blue like turtlesim default)
        self.set_pen(True, 69, 86, 255, width=20)  # Wide pen to cover more area
        
        # Draw a series of horizontal lines to cover the game area
        y_start = 0.5  # Start a bit below the grid
        y_end = 10.5   # End a bit above the grid
        step = 0.5     # Small steps for complete coverage
        
        for y in [y_start + i * step for i in range(int((y_end - y_start) / step) + 1)]:
            # Move to left side with pen off
            self.set_pen(False)
            self.teleport_turtle(0.5, y, 0.0)
            # Turn pen on for erasing
            self.set_pen(True, 69, 86, 255, width=20)
            # Draw line across
            self.teleport_turtle(10.5, y, 0.0)
        
        # Ensure pen is off after erasing
        self.set_pen(False)
        
        # Return to saved position
        self.teleport_turtle(save_x, save_y, 0.0)
    
    def get_keyboard_input(self):
        while self.running:
            key = self.get_key()
            
            if key.lower() == 'r':
                # Set reset flag
                self.reset_requested = True
                continue
            
            if key.lower() == 'q':
                # Set quit flag
                self.quit_requested = True
                continue
                
            if not self.game_active or self.current_player != 'X':
                time.sleep(0.1)
                continue
                
            if key.lower() == 'w':  # up
                self.move_to_adjacent_cell(0, -1)
            elif key.lower() == 's':  # down
                self.move_to_adjacent_cell(0, 1)
            elif key.lower() == 'a':  # left
                self.move_to_adjacent_cell(-1, 0)
            elif key.lower() == 'd':  # right
                self.move_to_adjacent_cell(1, 0)
            elif key.lower() == 'm':  # mark
                self.handle_player_move()
                
            time.sleep(0.1)  # Small delay to prevent flooding
    
    def move_to_adjacent_cell(self, dx, dy):
        """Move turtle to adjacent cell in the specified direction"""
        current_cell = self.get_current_cell()
        if current_cell is None:
            # If not in a specific cell, just do regular movement
            self.move_turtle(dx, 0.0 if dx else dy)
            return
            
        # Convert cell index to row and column
        current_row = current_cell // self.grid_size
        current_col = current_cell % self.grid_size
        
        # Calculate target row and column
        target_row = max(0, min(self.grid_size - 1, current_row + dy))
        target_col = max(0, min(self.grid_size - 1, current_col + dx))
        
        # If no change (already at edge), don't move
        if target_row == current_row and target_col == current_col:
            return
            
        # Calculate target cell index
        target_cell = target_row * self.grid_size + target_col
        
        # Teleport to target cell center
        self.set_pen(False)
        x, y = self.cell_positions[target_cell]
        self.teleport_turtle(x, y, 0.0)
    
    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    def pose_callback(self, msg):
        self.pose = msg
        
    def teleport_turtle(self, x, y, theta):
        request = TeleportAbsolute.Request()
        request.x = float(x)
        request.y = float(y)
        request.theta = float(theta)
        self.teleport_client.call_async(request)
        time.sleep(0.05)  # Reduced wait time for faster operation
        
    def set_pen(self, pen_on, r=255, g=255, b=255, width=2):
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = 0 if pen_on else 1
        self.set_pen_client.call_async(request)
        time.sleep(0.05)  # Reduced wait time for faster operation
        
    def move_turtle(self, x_dir, y_dir):
        if self.pose is None:
            return
        
        # Make sure pen is off during movement
        self.set_pen(False)
        
        # Create velocity command
        vel_msg = Twist()
        vel_msg.linear.x = float(x_dir)
        vel_msg.linear.y = float(y_dir)
        self.cmd_vel_pub.publish(vel_msg)
        
        # Stop the turtle after a short movement
        time.sleep(0.3)
        vel_msg = Twist()
        self.cmd_vel_pub.publish(vel_msg)
        
    def get_current_cell(self):
        if self.pose is None:
            return None
            
        # Find the closest cell to current position
        cell_size_half = self.cell_size / 2
        for i, (x, y) in enumerate(self.cell_positions):
            if abs(self.pose.x - x) < cell_size_half and abs(self.pose.y - y) < cell_size_half:
                return i
        return None
        
    def handle_player_move(self):
        if self.current_player != 'X' or not self.game_active:
            return
            
        cell = self.get_current_cell()
        if cell is not None and self.board[cell] == ' ':
            # Mark the cell with X
            self.board[cell] = 'X'
            self.draw_x(cell)
            
            # Check for win
            if self.check_winner():
                self.get_logger().info('Player X wins!')
                self.game_active = False
                return
                
            # Check for draw
            if ' ' not in self.board:
                self.get_logger().info('Game ended in a draw!')
                self.game_active = False
                return
                
            # Switch to computer turn
            self.current_player = 'O'
            self.get_logger().info("Computer's turn...")
        
    def draw_x(self, cell_idx):
        x, y = self.cell_positions[cell_idx]
        size = self.cell_size * 0.4  # Size of the X relative to cell size
        
        # Turn pen off for positioning
        self.set_pen(False)
        
        # Draw first diagonal
        self.teleport_turtle(x - size/2, y - size/2, 0.0)
        self.set_pen(True, 255, 0, 0)  # Red for X
        self.teleport_turtle(x + size/2, y + size/2, 0.0)
        
        # Turn pen off for repositioning
        self.set_pen(False)
        
        # Draw second diagonal
        self.teleport_turtle(x + size/2, y - size/2, 0.0)
        self.set_pen(True, 255, 0, 0)
        self.teleport_turtle(x - size/2, y + size/2, 0.0)
        
        # Lift pen when done
        self.set_pen(False)
        
    def draw_o(self, cell_idx):
        cx, cy = self.cell_positions[cell_idx]
        radius = self.cell_size * 0.2  # Size of O relative to cell size
        
        # Turn pen off for initial positioning
        self.set_pen(False)
        
        # Faster circle drawing with fewer steps for larger grids
        steps = max(8, min(12, 16 - self.grid_size))  # Fewer steps for larger grids
        
        # Direct teleport to draw circle segments (faster than steps)
        angle_step = 2 * math.pi / steps
        
        # Move to first point
        first_x = cx + radius
        first_y = cy
        self.teleport_turtle(first_x, first_y, 0.0)
        self.set_pen(True, 0, 0, 255)  # Blue for O
        
        # Draw the circle using direct teleporting (faster than small steps)
        for i in range(1, steps + 1):
            angle = i * angle_step
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            self.teleport_turtle(x, y, 0.0)
                
        # Lift pen when done
        self.set_pen(False)
        
    def make_computer_move(self):
        if not self.game_active:
            return
            
        # Simple AI to find best move
        # First check if computer can win
        for i in range(len(self.board)):
            if self.board[i] == ' ':
                self.board[i] = 'O'  # Try move
                if self.check_winner() == 'O':
                    # Winning move found
                    self.draw_o(i)
                    self.get_logger().info('Computer wins!')
                    self.game_active = False
                    return
                self.board[i] = ' '  # Undo move
                
        # Check if player can win and block
        for i in range(len(self.board)):
            if self.board[i] == ' ':
                self.board[i] = 'X'  # Try move
                if self.check_winner() == 'X':
                    # Blocking move
                    self.board[i] = 'O'
                    self.draw_o(i)
                    self.current_player = 'X'
                    return
                self.board[i] = ' '  # Undo move
                
        # Take center if available
        center_idx = (self.grid_size * self.grid_size) // 2
        if self.grid_size % 2 == 1 and self.board[center_idx] == ' ':
            self.board[center_idx] = 'O'
            self.draw_o(center_idx)
            self.current_player = 'X'
            return
            
        # Take a corner if available
        corners = [0, self.grid_size-1, 
                   self.grid_size*(self.grid_size-1), 
                   self.grid_size*self.grid_size-1]
        for i in corners:
            if self.board[i] == ' ':
                self.board[i] = 'O'
                self.draw_o(i)
                self.current_player = 'X'
                return
                
        # Take any available space
        for i in range(len(self.board)):
            if self.board[i] == ' ':
                self.board[i] = 'O'
                self.draw_o(i)
                self.current_player = 'X'
                return
                
        # If board is full
        self.get_logger().info('Game ended in a draw!')
        self.game_active = False
        
    def check_winner(self):
        # For n×n grid, check for n in a row in any direction
        
        # Check rows
        for row in range(self.grid_size):
            row_start = row * self.grid_size
            if self.board[row_start] != ' ' and all(self.board[row_start] == self.board[row_start + i] 
                                                 for i in range(self.grid_size)):
                return self.board[row_start]
                
        # Check columns
        for col in range(self.grid_size):
            if self.board[col] != ' ' and all(self.board[col] == self.board[col + i * self.grid_size] 
                                           for i in range(self.grid_size)):
                return self.board[col]
                
        # Check main diagonal (top-left to bottom-right)
        if self.board[0] != ' ' and all(self.board[0] == self.board[i * (self.grid_size + 1)] 
                                      for i in range(self.grid_size)):
            return self.board[0]
            
        # Check other diagonal (top-right to bottom-left)
        if self.board[self.grid_size - 1] != ' ' and all(self.board[self.grid_size - 1] == self.board[(i + 1) * (self.grid_size - 1)] 
                                                      for i in range(self.grid_size)):
            return self.board[self.grid_size - 1]
            
        return None
        
    def reset_game(self):
        try:
            # Ask for grid size again to allow changing it
            self.get_grid_size()
            
            # Clear the screen to remove all marks
            self.clear_screen_with_background()
            
            # Reset the game state with new grid size
            self.board = [' ' for _ in range(self.grid_size * self.grid_size)]
            self.current_player = 'X'
            self.game_active = True
            
            # Recalculate cell size and positions
            self.cell_size = min((self.grid_end_x - self.grid_start_x) / self.grid_size, 
                                (self.grid_end_y - self.grid_start_y) / self.grid_size)
            
            # Recalculate cell positions
            self.calculate_cell_positions()
            
            # Draw a fresh grid
            self.draw_grid()
            
            # Reset position
            self.set_pen(False)  # Make sure pen is off
            center_x = (self.grid_start_x + self.grid_end_x) / 2
            center_y = (self.grid_start_y + self.grid_end_y) / 2
            self.teleport_turtle(center_x, center_y, 0.0)

            self.get_logger().info(f'Game reset with {self.grid_size}x{self.grid_size} grid! Your turn (X)')
        except Exception as e:
            self.get_logger().error(f"Error during reset: {str(e)}")
        
    def game_loop(self):
        # Check if quit was requested from keyboard input thread
        if self.quit_requested:
            self.get_logger().info("Quitting game...")
            self.running = False
            # Proper shutdown
            rclpy.shutdown()
            sys.exit(0)
            
        # Check if reset was requested from keyboard input thread
        if self.reset_requested:
            self.reset_requested = False
            self.reset_game()
            return
            
        if not self.game_active:
            return
            
        # Handle computer turn
        if self.current_player == 'O':
            time.sleep(0.5)  # Reduced delay before computer moves for faster gameplay
            self.make_computer_move()
            
            # Check for win after computer move
            if self.check_winner() == 'O':
                self.get_logger().info('Computer wins!')
                self.game_active = False
            # Check for draw after computer move
            elif ' ' not in self.board:
                self.get_logger().info('Game ended in a draw!')
                self.game_active = False
            
            if self.game_active:
                self.get_logger().info('Your turn!')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TicTacToeGameNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown if node exists
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

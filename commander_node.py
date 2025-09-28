import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen # <-- FIXED: Missing Service Import
import math
import sys

# M_PI is not automatically available in Python's math module for all platforms, 
# but math.pi is guaranteed.
PI = math.pi 

class CommanderNode(Node):
    def __init__(self, **kwargs):
        super().__init__('commander_node', **kwargs)
        
        # Publishers and Clients
        self.subscription = self.create_subscription(String, 'space_objects', self.command_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        
        # State Variables for Drawing (Asynchronous State Machine)
        self.is_drawing = False
        # self.steps_per_segment is unused, removed for cleanliness
        self.current_step = 0
        self.drawing_queue = [] # List of movement steps to execute
        
        # Primary ROS 2 Timer (runs every 50ms for smooth movement)
        self.timer = self.create_timer(0.05, self.movement_loop)
        
        self.get_logger().info('Commander Node ready to receive commands!')

    # --- TOPIC CALLBACK (Receives command from ShapeNode) ---
    def command_callback(self, msg):
        command = msg.data.lower().strip()
        self.get_logger().info(f'Received command: "{command}"')
        
        # Clear the queue for the new drawing command
        self.drawing_queue = [] 

        if command == "stop":
            self.stop_turtle()
            return
            
        # Parse the message (e.g., "2 planet ufo" or "3 star")
        words = command.split()
        
        planets, stars, ufo = 0, 0, False
        
        # Determine counts based on parsed words
        if "planet" in words:
            try:
                planets = int(words[words.index('planet') - 1])
            except (ValueError, IndexError):
                planets = 1 
        if "star" in words:
            try:
                stars = int(words[words.index('star') - 1])
            except (ValueError, IndexError):
                stars = 1
        if "ufo" in words:
            ufo = True

        self.get_logger().info(f'Parsed: {planets} planets, {stars} stars, UFO: {ufo}')
        self.build_drawing_queue(planets, stars, ufo)


    # --- MOVEMENT LOOP (Non-blocking execution) ---
    def movement_loop(self):
        if not self.is_drawing or not self.drawing_queue:
            self.stop_turtle()
            return

        # Execute the next step in the current drawing action
        action = self.drawing_queue[0] 
        self.execute_action(action)
        
        self.current_step += 1

        if self.current_step >= action['steps']:
            # Action complete, move to the next item in the queue
            self.drawing_queue.pop(0)
            self.current_step = 0
            
            if not self.drawing_queue:
                self.stop_turtle() # Drawing is finished


    def execute_action(self, action):
        """Publishes velocity or sets pen based on the current action state."""
        twist = Twist()
        
        if action['type'] == 'move':
            twist.linear.x = action['speed']
            twist.angular.z = action['angular']
            
            self.cmd_pub.publish(twist)
            
        elif action['type'] == 'pen' and self.current_step == 0:
            # Service call logic (only run once per action block)
            self.set_pen(action['r'], action['g'], action['b'], action['pen_on'])
            
            # Publish zero velocity while waiting for service to complete
            self.cmd_pub.publish(twist) 


    # --- DRAWING QUEUE BUILDERS ---
    def build_drawing_queue(self, planets, stars, ufo):
        self.is_drawing = True
        self.drawing_queue = []

        # --- PLANETS (Circles) ---
        for i in range(planets):
            # Pen: Red/Pink, ON
            self.drawing_queue.append({'type': 'pen', 'r': 255, 'g': 100, 'b': 100, 'pen_on': True, 'steps': 1})
            self.add_circle_actions()
            # Pause/Move for separation
            self.drawing_queue.append({'type': 'pen', 'r': 0, 'g': 0, 'b': 0, 'pen_on': False, 'steps': 1}) 
            self.drawing_queue.append({'type': 'move', 'speed': 0.0, 'angular': -1.0, 'steps': 10}) 
            self.drawing_queue.append({'type': 'move', 'speed': 1.0, 'angular': 0.0, 'steps': 20}) 
            
        # --- STARS (Polygons) ---
        for i in range(stars):
            # Pen: Yellow, ON
            self.drawing_queue.append({'type': 'pen', 'r': 255, 'g': 255, 'b': 0, 'pen_on': True, 'steps': 1})
            self.add_star_actions()
            # Pause/Move for separation
            self.drawing_queue.append({'type': 'pen', 'r': 0, 'g': 0, 'b': 0, 'pen_on': False, 'steps': 1}) 
            self.drawing_queue.append({'type': 'move', 'speed': 0.0, 'angular': 1.0, 'steps': 10}) 
            self.drawing_queue.append({'type': 'move', 'speed': 1.0, 'angular': 0.0, 'steps': 20}) 

        # --- UFO (Wiggle) ---
        if ufo:
            self.drawing_queue.append({'type': 'pen', 'r': 0, 'g': 255, 'b': 0, 'pen_on': True, 'steps': 1})
            self.add_ufo_actions()

        # Final action: Ensure pen is off and turtle is stopped
        self.drawing_queue.append({'type': 'pen', 'r': 0, 'g': 0, 'b': 0, 'pen_on': False, 'steps': 1}) 


    def add_circle_actions(self):
        # Circle: Move forward and turn continuously (V=R*W). 1.0 = 1.0 * 1.0
        steps = 150 
        self.drawing_queue.append({'type': 'move', 'speed': 1.0, 'angular': 1.0, 'steps': steps}) 

    def add_star_actions(self):
        # Star (5-point): 5 outer lines, 5 turns (External angle = 144 degrees = 2.51 rad)
        segment_steps = 40 
        # Angular rate needed to turn 144 degrees in 0.05 seconds: (2.51 rad) / 0.05 s
        angular_rate = 2.51 / 0.05 

        for i in range(5):
            # 1. Move line segment
            self.drawing_queue.append({'type': 'move', 'speed': 2.0, 'angular': 0.0, 'steps': segment_steps}) 
            # 2. Turn 144 degrees (turn in place)
            self.drawing_queue.append({'type': 'move', 'speed': 0.0, 'angular': -angular_rate, 'steps': 1}) # Negative for clockwise turn

    def add_ufo_actions(self):
        # Simple irregular wiggle (Alternating turns with continuous forward motion)
        wiggle_steps = 10 
        for i in range(20):
             # Wiggle right
             self.drawing_queue.append({'type': 'move', 'speed': 1.0, 'angular': 2.0, 'steps': wiggle_steps}) 
             # Wiggle left
             self.drawing_queue.append({'type': 'move', 'speed': 1.0, 'angular': -2.0, 'steps': wiggle_steps})


    # --- HELPER FUNCTIONS ---
    def set_pen(self, r, g, b, pen_on):
        """Set pen color and state using the service client."""
        if not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Pen service not available, skipping pen change.')
            return
        
        request = SetPen.Request(r=r, g=g, b=b, width=3, off=0 if pen_on else 1)
        self.pen_client.call_async(request)

    def stop_turtle(self):
        """Immediately stops movement and sets drawing state to false."""
        self.is_drawing = False
        self.cmd_pub.publish(Twist()) # Publish default empty Twist (all zeros)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CommanderNode()
        rclpy.spin(node)

    except (SystemExit, KeyboardInterrupt):
        pass # Handle clean exit

    finally:
        if 'node' in locals() and node is not None:
             node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': # <-- FIXED: Correct Entry Point
    main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class ShapeNode(Node):
    
    # --- FIXED: Use double underscores for constructor ---
    def __init__(self):
        super().__init__('shape_node')
        self.publisher_ = self.create_publisher(String, 'space_objects', 10)
        self.get_logger().info('ShapeNode Initialized. Gathering input...')
        
        # Gather input and publish the result immediately
        self.publish_choices(self.get_user_choices())
        
        # Shut down immediately after publishing to avoid blocking
        self.get_logger().info('Input published. Shutting down input node.')
        self.destroy_node()
        rclpy.shutdown()

    def get_user_choices(self):
        # Helper function for clean integer input and validation
        def get_int_input(prompt, valid_range):
            while True:
                try:
                    # Using sys.stdin.fileno to check if terminal is interactive
                    # This prevents crashes when run via launch file without a terminal
                    if not sys.stdin.isatty():
                        return 0 # Return default if not running in a terminal
                        
                    choice = int(input(prompt))
                    if choice in valid_range:
                        return choice
                    else:
                        print(f"Invalid input. Please choose from: {list(valid_range)}")
                except ValueError:
                    print("Invalid input. Please enter a number.")
                except EOFError:
                    return 0
        
        # State Variables
        n = 0
        ufo = False 
        p = 0
        s = 0

        print("\nChoose 1 for planets, 2 for stars, or 'stop' to end.")
        # Check if sys.stdin.isatty() is necessary because launch systems often run nodes without an interactive terminal
        choice = input("Enter choice (1 or 2 or 'stop'): ").strip().lower()
        
        if choice == 'stop':
            return "stop"
        
        try:
            n = int(choice)
        except ValueError:
            return "stop"

        ufo_choice = input("If you want a UFO say YES else NO: ").strip().lower()
        ufo = (ufo_choice in ["yes", "y"])

        if n == 1:
            p = get_int_input("Enter number of planets (1, 2, or 3): ", {1, 2, 3})
        elif n == 2:
            s = get_int_input("Enter number of stars (1, 2, or 3): ", {1, 2, 3})

        return {'p': p, 's': s, 'ufo': ufo}

    def publish_choices(self, choices):
        msg = String()
        
        if choices == "stop":
            msg.data = "stop"
        else:
            ufo_status = " ufo" if choices['ufo'] else "" # Added space here for clean parsing
            if choices['p'] > 0:
                content = f"{choices['p']} planet{ufo_status}"
            elif choices['s'] > 0:
                content = f"{choices['s']} star{ufo_status}"
            else:
                content = "stop"

            msg.data = content.strip()
        
        self.get_logger().info(f"Publishing Command: '{msg.data}'")
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        ShapeNode()
    except Exception:
        pass
    finally:
        rclpy.shutdown()

# --- FIXED: Correct Python Entry Point ---
if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from Raspblock import Raspblock
import time

robot = Raspblock()

class RobotControlServer(Node):
    def __init__(self):
        super().__init__('robot_control_server')

        # Movement control services for 4 buttons
        self.button1_service = self.create_service(SetBool, 'button1_service', self.handle_button1)
        self.button2_service = self.create_service(SetBool, 'button2_service', self.handle_button2)
        self.button3_service = self.create_service(SetBool, 'button3_service', self.handle_button3)
        self.button4_service = self.create_service(SetBool, 'button4_service', self.handle_button4)
        self.button5_service = self.create_service(SetBool, 'button5_service', self.handle_button5)
        self.button6_service = self.create_service(SetBool, 'button6_service', self.handle_button6)

        # Movement control subscriber
        self.movement_subscription = self.create_subscription(
            Twist,
            'movement_control',
            self.handle_movement,
            10
        )

        self.get_logger().info("Robot control server is ready.")

    # Button 1: Move forward 1 second
    def handle_button1(self, request, response):
        if request.data:
            robot.Speed_axis_control(0, 15, 0)  # Move forward
            response.success = True
            response.message = "Moving forward 1 second (Button 1)"
            time.sleep(1.0)
            robot.Speed_axis_control(0, 0, 0)  # Stop
        else:
            response.success = True
            response.message = "Button 1 released, no movement"
        return response

    # Button 2: Move backward 1 second
    def handle_button2(self, request, response):
        if request.data:
            robot.Speed_axis_control(15, 0, 0)  # Move backward
            response.success = True
            response.message = "Moving backward 1 second (Button 2)"
            time.sleep(1.0)
            robot.Speed_axis_control(0, 0, 0)
        else:
            response.success = True
            response.message = "Button 2 released, no movement"
        return response

    # Button 3: Move left 1 second
    def handle_button3(self, request, response):
        if request.data:
            robot.Speed_axis_control(0, -15, 0)  # Move left
            response.success = True
            response.message = "Moving left 1 second (Button 3)"
            time.sleep(1.0)
            robot.Speed_axis_control(0, 0, 0)
        else:
            response.success = True
            response.message = "Button 3 released, no movement"
        return response

    # Button 4: Move right 1 second
    def handle_button4(self, request, response):
        if request.data:
            robot.Speed_axis_control(-15, 0, 0)  # Move right
            response.success = True
            response.message = "Moving right 1 second (Button 4)"
            time.sleep(1.0)
            robot.Speed_axis_control(0, 0, 0)
        else:
            response.success = True
            response.message = "Button 4 released, no movement"
        return response
#------------------------------------------------------------------------    
        # Button 5: Move right 1 second
    def handle_button5(self, request, response):
        if request.data:
            robot.Speed_axis_control(0, 0, 15)  # Move right
            response.success = True
            response.message = "Moving right 1 second (Button 5)"
            time.sleep(1.0)
            robot.Speed_axis_control(0, 0, 0)
        else:
            response.success = True
            response.message = "Button 5 released, no movement"
        return response
        # Button 6: Move right 1 second
    def handle_button6(self, request, response):
        if request.data:
            robot.Speed_axis_control(0, 0, -15)  # Move right
            response.success = True
            response.message = "Moving right 1 second (Button 6)"
            time.sleep(1.0)
            robot.Speed_axis_control(0, 0, 0)
        else:
            response.success = True
            response.message = "Button 6 released, no movement"
        return response

    # Handle normal movement from joystick axes
    def handle_movement(self, data):
        Speed_axis_X = int(data.linear.x * 15)
        Speed_axis_Y = int(data.linear.y * 15)
        Speed_axis_Z = int(data.angular.z * 15)
        robot.Speed_axis_control(Speed_axis_X, Speed_axis_Y, Speed_axis_Z)

def main(args=None):
    rclpy.init(args=args)
    robot_control_server = RobotControlServer()
    try:
        rclpy.spin(robot_control_server)
    except KeyboardInterrupt:
        robot_control_server.get_logger().info("Shutting down robot control server.")
    finally:
        robot_control_server.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
import pygame

class JoystickTwistPublisher(Node):
    def __init__(self):
        super().__init__('joystick_twist_publisher')

        # Publisher
        self.publisher_ = self.create_publisher(Twist, '/movement_control', 1)
        self.timer = self.create_timer(0.1, self.publish_twist)  # 10 Hz

        # Initialize pygame
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joystick detected! Please connect a joystick.")
            raise RuntimeError("No joystick connected.")

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"Joystick initialized: {self.joystick.get_name()}")

        # Twist msg
        self.twist_msg = Twist()

        # Create clients for the 4 services
        self.client1 = self.create_client(SetBool, 'button1_service')
        self.client2 = self.create_client(SetBool, 'button2_service')
        self.client3 = self.create_client(SetBool, 'button3_service')
        self.client4 = self.create_client(SetBool, 'button4_service')

        # For now, button 9 triggers the movement to the right; 
        # it is planned that this button will be assigned to Update Memory (UM)
        self.client9 = self.create_client(SetBool, 'button9_service')

        # Wait for the services to be available
        while not self.client1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for button1_service...")
        while not self.client2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for button2_service...")
        while not self.client3.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for button3_service...")
        while not self.client4.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for button4_service...")
        # Button 9 corresponds to the "Start" button on the controller
        while not self.client9.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for button9_service...")

    def publish_twist(self):
        # Process events
        pygame.event.pump()

        # Axes
        self.twist_msg.linear.x = self.joystick.get_axis(0)    # Adelante/atrás
        self.twist_msg.linear.y = -self.joystick.get_axis(1)   # Izquierda/derecha
        self.twist_msg.angular.z = self.joystick.get_axis(2)   # Giro

        # Publish
        self.publisher_.publish(self.twist_msg)

        # Detect buttons (0,1,2,3 on the joystick correspond to 1,2,3,4)
        if self.joystick.get_button(0):
            self.call_service(self.client1)
        if self.joystick.get_button(1):
            self.call_service(self.client2)
        if self.joystick.get_button(2):
            self.call_service(self.client3)
        if self.joystick.get_button(3):
            self.call_service(self.client4)
        #Boton 9
        if self.joystick.get_button(9):
            self.call_service(self.client9)

    # ---- Method to invoke services ----
    def call_service(self, client):
        req = SetBool.Request()
        req.data = True
        future = client.call_async(req)
        # You can optionally add a callback to receive the response
        # future.add_done_callback(lambda f: self.get_logger().info(f.result().message))
        self.get_logger().info(f"Servicio {client.srv_name} llamado")

def main(args=None):
    rclpy.init(args=args)

    try:
        node = JoystickTwistPublisher()
        rclpy.spin(node)
    except RuntimeError as e:
        print(e)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__':
    main()


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
import pygame
import time

class JoystickTwistPublisher(Node):
    def __init__(self):
        super().__init__('joystick_twist_publisher_butter')

        # Publicador del movimiento
        self.publisher_ = self.create_publisher(Twist, '/movement_control', 1)
        self.timer = self.create_timer(0.1, self.publish_twist)

        # Inicializar joystick
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("❌ No se detectó joystick.")
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        # 🎮 Servicios
        self.client_map = {
            0: self.create_client(SetBool, 'button1_service'),  # Adelante
            1: self.create_client(SetBool, 'button2_service'),  # Derecha
            2: self.create_client(SetBool, 'button3_service'),  # Atrás
            3: self.create_client(SetBool, 'button4_service'),  # Izquierda
        }
        self.image_client = self.create_client(SetBool, 'toggle_image_saving')
        self.saving_images = False

        # Estados
        self.prev_buttons = [0] * 10
        self.last_sent_time = [0] * 10
        self.cooldown = 0.2

        self.get_logger().info("✅ Joystick Butter en servicio 🧈")

    def publish_twist(self):
        pygame.event.pump()
        twist = Twist()
        twist.linear.x = self.joystick.get_axis(0)
        twist.linear.y = -self.joystick.get_axis(1)
        twist.angular.z = self.joystick.get_axis(2)
        self.publisher_.publish(twist)

        buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
        now = time.time()

        # ⚠️ Detección de varios botones presionados a la vez
        if sum(buttons[:4]) > 1:
            if sum(self.prev_buttons[:4]) <= 1:
                self.get_logger().warn("⚠️ Dos botones presionados, ignorando acción por seguridad.")
            self.prev_buttons = buttons.copy()
            return

        # --- Botones normales (0–3) ---
        for i in range(4):
            if buttons[i] == 1 and now - self.last_sent_time[i] > self.cooldown:
                self._send_button(i)
                self.last_sent_time[i] = now

        # --- Start (botón 9) para activar/desactivar guardado ---
        if buttons[9] == 1 and self.prev_buttons[9] == 0:
            self.saving_images = not self.saving_images
            self._toggle_image_saving(self.saving_images)

        # Mostrar qué botón presionas
        for i, val in enumerate(buttons):
            if val == 1 and self.prev_buttons[i] == 0:
                self.get_logger().info(f"Botón {i} presionado")

        self.prev_buttons = buttons.copy()

    def _send_button(self, idx):
        client = self.client_map[idx]
        if not client.service_is_ready():
            self.get_logger().warn(f"⚠️ Servicio {idx} no listo.")
            return
        req = SetBool.Request()
        req.data = True
        client.call_async(req)
        self.get_logger().info(f"Acción → Botón {idx}")

    def _toggle_image_saving(self, active):
        if not self.image_client.service_is_ready():
            self.get_logger().warn("⚠️ Servicio toggle_image_saving no listo.")
            return
        req = SetBool.Request()
        req.data = active
        self.image_client.call_async(req)
        msg = "🎥 Guardado ACTIVADO" if active else "🧹 Guardado DETENIDO"
        self.get_logger().info(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickTwistPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

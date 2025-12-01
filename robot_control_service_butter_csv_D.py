import os
import csv
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from Raspblock import Raspblock

import pyrealsense2 as rs  

robot = Raspblock()


# ======================= CLASE PRINCIPAL ========================
class RobotControlServer(Node):

    def __init__(self):
        super().__init__('robot_control_server_mantequilla_realsense_csv')
        # ======================== MOVIMIENTO ===========================
        
        self.move_duration = 1.0    # Duración fija de cada movimiento (segundos)
        self.empalme = 0.2  # Ventana para empalmar comandos 
        self.timer_period = 0.05 # Periodo entre cada comando PWM (50 ms)
        self.coast_mode = True  # Si True, el robot no frena en seco
        self.move_time_left = 0.0   # Tiempo que le queda al comando actual
        self.current_speed = (0, 0, 0)  # Velocidad actual enviada al robot (ejes x,y,z)
        self.next_command = None   # Comando esperando empalme
        self.busy = False   # Si el robot está ocupado ejecutando algo
        self.empalming = False  # Indica si está en la ventana de empalme
        self.timer = None # Timer que manda comandos PWM al robot
        self.last_command = "En reposo" # Último comando ejecutado (para guardar en CSV)

        # ======================= REALSENSE + PATHS =====================

        self.bridge = CvBridge()

        self.save_images = False    # Flag general de guardado de imágenes activado / desactivado
        self.frame_skip = 3 # Guardar solo 1 de cada N frames (para no saturar)
        self.frame_count = 0 # Contador de frames recibidos
        self.image_index = 0    # Índice para nombrar imágenes rgb_0000.png

        # Carpetas en ~/Pictures
        base_path = os.path.expanduser("~/Pictures")
        self.rgb_path = os.path.join(base_path, "rgb")
        self.depth_path = os.path.join(base_path, "depth")

        os.makedirs(self.rgb_path, exist_ok=True)
        os.makedirs(self.depth_path, exist_ok=True)

        # Buscar último número guardado para continuar secuencia
        existing_rgb = [
            f for f in os.listdir(self.rgb_path)
            if f.startswith("rgb_") and f.endswith(".png")
        ]

        if existing_rgb:
            try:
                indices = [int(f[4:8]) for f in existing_rgb if f[4:8].isdigit()]
                if indices:
                    self.image_index = max(indices) + 1
                    self.get_logger().info(f"Continuando desde la imagen {self.image_index:04d}")
            except Exception as e:
                self.get_logger().warn(f"No se pudo leer índices previos: {e}")
        else:
            self.get_logger().info("No se encontraron imágenes previas, comenzando desde 0.")

        # ======================== REALSENSE INIT =======================

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.realsense_ok = False

        try:
            ctx = rs.context()
            devices = ctx.query_devices()

            # Verificar que sí existe cámara conectada
            if len(devices) == 0:
                raise RuntimeError("No se detectó ninguna cámara RealSense conectada.")

            # Habilitar color y profundidad a 15 Hz (más ligero)
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)

            self.pipeline.start(self.config)
            self.realsense_ok = True

            self.get_logger().info("📷 RealSense D415 inicializada correctamente (RGB + Depth a 15 Hz)")

        except Exception as e:
            self.get_logger().warn(f"RealSense no disponible: {e}")
            self.realsense_ok = False

        # Timer que publica imágenes a ROS
        if self.realsense_ok:
            self.img_timer = self.create_timer(0.1, self.publish_image)
        else:
            self.get_logger().warn("No se inició el temporizador de imágenes.")

        # ============================ CSV ==============================

        self.csv_path = os.path.join(base_path, "registro_robot.csv")

        # Crear archivo si no existe
        if not os.path.isfile(self.csv_path):
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp_ms', 'x', 'y', 'z',
                    'rgb_filename', 'depth_filename', 'depth_vis_filename',
                    'datetime_iso', 'accion'
                ])

        # ========================= ROS2 SETUP ==========================

        # Botones virtuales 1–4
        self.button_services = [
            self.create_service(SetBool, f'button{i + 1}_service',
                                getattr(self, f'handle_button{i + 1}'))
            for i in range(4)
        ]

        # Servicio para ver si está ocupado
        self.status_service = self.create_service(
            Trigger, 'get_robot_status', self.get_status_callback)

        # Control por tópico /joystick
        self.movement_subscription = self.create_subscription(
            Twist, 'movement_control', self.handle_movement, 10)

        # Activar/desactivar guardado de imágenes
        self.save_toggle_service = self.create_service(
            SetBool, 'toggle_image_saving', self.toggle_image_saving)

        # Publicación en /camera/image_raw
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)

        self.get_logger().info("🧈 Servidor mantequilloso listo (RGB/Depth + CSV + RealSense)")

    # ================= PUBLICACIÓN Y GUARDADO ======================

    def publish_image(self):
        """Captura frame de RealSense, publica RGB y guarda RGB+Depth si está activado."""
        if not self.realsense_ok:
            return

        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                return

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # Publicar a ROS
            msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
            self.image_pub.publish(msg)

            # Solo guardar si está activado
            if self.save_images:

                self.frame_count += 1

                if self.frame_count % self.frame_skip == 0:

                    rgb_filename = f"rgb_{self.image_index:04d}.png"
                    depth_filename = f"depth_{self.image_index:04d}.png"
                    depth_vis_filename = f"depth_vis_{self.image_index:04d}.png"

                    # Paths completos
                    rgb_path = os.path.join(self.rgb_path, rgb_filename)
                    depth_path = os.path.join(self.depth_path, depth_filename)
                    depth_vis_path = os.path.join(self.depth_path, depth_vis_filename)

                    # Guardar imágenes
                    cv2.imwrite(rgb_path, color_image)
                    cv2.imwrite(depth_path, depth_image)

                    # Crear versión visual en color utilizando un colormap
                    depth_vis = cv2.convertScaleAbs(depth_image, alpha=0.03)
                    depth_vis_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                    cv2.imwrite(depth_vis_path, depth_vis_colored)

                    # Guardar en CSV
                    timestamp_ms = int(time.time() * 1000)
                    datetime_iso = datetime.now().isoformat(timespec='seconds')
                    x, y, z = self.current_speed

                    with open(self.csv_path, 'a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow([
                            timestamp_ms, x, y, z,
                            rgb_filename, depth_filename, depth_vis_filename,
                            datetime_iso, self.last_command
                        ])

                    self.get_logger().info(f" Guardado #{self.image_index:04d} ({self.last_command})")
                    self.image_index += 1

        except Exception as e:
            self.get_logger().error(f"Error al capturar imagen: {e}")

    # ========================= SERVICIOS ===========================

    def toggle_image_saving(self, req, res):
        """Activa/desactiva el guardado de imágenes."""
        self.save_images = req.data
        res.success = True
        res.message = f"Guardado {'activado' if req.data else 'detenido'}"
        self.get_logger().info(res.message)
        return res

    def get_status_callback(self, req, res):
        """Regresa si el robot está libre u ocupado."""
        res.success = not self.busy
        res.message = "Robot libre" if not self.busy else "Robot ocupado"
        return res
    
    # ================= MOVIMIENTO SUAVE MANTEQUILLA ================

    def _start_movement(self, spd, name):
        """Inicia un movimiento con empalme."""
        if not self.busy:
            self.busy = True
            self.empalming = False
            self.current_speed = spd
            self.move_time_left = float(self.move_duration)
            self.last_command = name

            if self.timer:
                self.timer.cancel()

            self.timer = self.create_timer(self.timer_period, self._timer_cb)

            self.get_logger().info(f"▶️ {name} {spd}")

        elif self.empalming and not self.next_command:
            self.next_command = (spd, name)
            self.get_logger().info(f"{name} programado (empalme)")

        else:
            self.get_logger().warn(f"{name} ignorado (ocupado)")

    def _timer_cb(self):
        """Callback periódico para enviar velocidades mantequillosas."""
        if self.move_time_left > 0:
            x, y, z = self.current_speed
            robot.Speed_axis_control(int(x), int(y), int(z))
            self.move_time_left -= self.timer_period

            if self.move_time_left <= self.empalme and not self.empalming:
                self.empalming = True

        else:
            if self.next_command:
                nxt, nm = self.next_command
                self.next_command = None
                self.empalming = False
                self.current_speed = nxt
                self.move_time_left = float(self.move_duration)
                self.last_command = nm
            else:
                self._stop_robot()

    def _stop_robot(self):
        """Detiene el robot (a menos que esté en modo coast)."""
        if not self.coast_mode:
            robot.Speed_axis_control(0, 0, 0)

        self.current_speed = (0, 0, 0)
        self.busy = False
        self.empalming = False
        self.last_command = "En reposo"

        if self.timer:
            self.timer.cancel()

    # ==================== HANDLERS DE BOTONES ======================

    def handle_movement(self, msg):
        """Control directo por joystick."""
        if not self.busy:
            x = int(msg.linear.x * 15)
            y = int(msg.linear.y * 15)
            z = int(msg.angular.z * 15)
            robot.Speed_axis_control(x, y, z)

    def handle_button1(self, req, res): return self._handle_button(req, res, "Adelante", (0, 20, 0))
    def handle_button2(self, req, res): return self._handle_button(req, res, "Derecha",  (0, 0, 20))
    def handle_button3(self, req, res): return self._handle_button(req, res, "Atrás",    (0, -20, 0))
    def handle_button4(self, req, res): return self._handle_button(req, res, "Izquierda",(0, 0, -20))

    def _handle_button(self, req, res, name, spd):
        """Handler genérico para botones 1–4."""
        if req.data:
            self._start_movement(spd, name)

        res.success = True
        res.message = f"{name} ejecutado"
        return res

# ============================== MAIN ============================

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.realsense_ok:
            try:
                node.pipeline.stop()
                node.get_logger().info("RealSense detenida correctamente.")
            except Exception as e:
                node.get_logger().warn(f"No se pudo detener RealSense: {e}")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

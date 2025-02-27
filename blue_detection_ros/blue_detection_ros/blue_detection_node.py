import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class BlueDetectionNode(Node):
    def __init__(self):
        super().__init__('blue_detection_node')
        # Suscribirse al tópico de la cámara
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.latest_frame = None

        # Timer para actualizar la ventana a una tasa fija (aprox. 30 FPS)
        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info("Nodo de detección de azul inicializado.")

    def image_callback(self, msg):
        try:
            # Convertir el mensaje ROS a imagen OpenCV (BGR)
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"Error al convertir la imagen: {e}")
            return

        # Procesamiento para detectar color azul
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Buscar contornos en la máscara
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) > 500:  # Umbral para ignorar pequeños ruidos
                x, y, w, h = cv2.boundingRect(cnt)
                # Dibuja un rectángulo verde sobre las áreas azules detectadas
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Almacena el frame procesado para su actualización en la ventana
        self.latest_frame = frame

    def timer_callback(self):
        # Si hay un frame disponible, se actualiza la ventana existente
        if self.latest_frame is not None:
            cv2.imshow("Video Feed", self.latest_frame)
            # waitKey(1) es suficiente para refrescar la ventana sin bloquear
            cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    # Crear la ventana **única** antes de iniciar el nodo
    cv2.namedWindow("Video Feed", cv2.WINDOW_NORMAL)
    rclpy.init(args=args)
    node = BlueDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



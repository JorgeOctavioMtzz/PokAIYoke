import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class IriumPublisherNode(Node):
    def __init__(self):
        super().__init__('irium_publisher_node')
        # Publicador para el tópico de la cámara
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        # Abre la cámara USB (o virtual, como la que crea iRium en modo cable)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara (iRium).")
        # Timer para publicar frames periódicamente (cada 0.1 segundos)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("No se pudo obtener frame de la cámara iRium.")
            return
        # Convertir el frame a mensaje ROS
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher.publish(msg)
        self.get_logger().info("Frame publicado en /camera/image_raw.")

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = IriumPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


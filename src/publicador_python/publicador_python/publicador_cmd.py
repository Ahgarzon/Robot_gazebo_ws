# publicador_python/publicador_cmd.py
#
# Publica comandos de velocidad (geometry_msgs/Twist) en /cmd_vel
# ──────────────────────────────────────────────────────────────────
#  • Linear  : +0.10 m/s en X
#  • Angular :   0.00 rad/s (sin giro)
#  • Frecuencia de envío: 100 Hz   (timer = 0.01 s)
#
# Para ejecutar:
#   ros2 run publicador_python publicador_cmd
#

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class PublicadorCMD(Node):
    """Nodo que publica constantemente un mensaje Twist en /cmd_vel."""

    def __init__(self) -> None:
        super().__init__("publicador_cmd")

        # Publicador de geometry_msgs/Twist                       topic  QoS depth
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)

        self.i = 0  # contador solo para el log
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz

    # -------------------------------------------------------------------------
    # Call-back periódico → genera y envía el Twist
    # -------------------------------------------------------------------------
    def timer_callback(self) -> None:
        msg = Twist()
        msg.linear.x = 0.10          # m/s  → avanza 10 cm/s
        msg.angular.z = 0.00         # rad/s → sin rotación

        self.publisher_.publish(msg)

        # Registro legible en consola (NO usar campos inexistentes)
        self.get_logger().info(
            f"[{self.i}]   linear=({msg.linear.x:.3f}, {msg.linear.y:.3f}, {msg.linear.z:.3f})  "
            f"angular=({msg.angular.x:.3f}, {msg.angular.y:.3f}, {msg.angular.z:.3f})"
        )
        self.i += 1


# -----------------------------------------------------------------------------
# Punto de entrada
# -----------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = PublicadorCMD()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

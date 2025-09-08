#!/usr/bin/env python3
from __future__ import annotations

import math
import random
from enum import Enum, auto
from typing import Final

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# ───────── parámetros globales ─────────
RADIO_MAX:      Final[float] = 80.0      # m
DIST_OBJ_START: Final[float] = 5.0       # m
DIST_OBJ_GOAL:  Final[float] = 0.60      # m

V_EXPLORE:      Final[float] = 0.20      # m/s
W_EXPLORE_MAX:  Final[float] = 0.30      # rad/s  (rumbo aleatorio)

V_CHASE:        Final[float] = 0.35      # m/s
W_TURN:         Final[float] = 1.5       # rad/s   (giro 90°)

CTRL_DT:        Final[float] = 0.05      # 20 Hz

class Estado(Enum):
    EXPLORA   = auto()
    PERSECUCION = auto()
    ESQUIVA   = auto()
    RETORNO   = auto()

class Autonomia(Node):
    def __init__(self) -> None:
        super().__init__("autonomia_wander")

        # pubs / subs
        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(LaserScan, "laser_scan",     self.cb_scan, 10)
        self.create_subscription(Odometry,  "wheel/odometry", self.cb_odom, 10)

        # estado sensores
        self.laser: list[float] = []
        self.x = self.y = 0.0
        self.x0 = self.y0 = None

        # estado de control
        self.estado: Estado = Estado.EXPLORA

        # para el modo EXPLORA
        self._rumbo_expira = 0.0          # instante en que cambiamos de rumbo
        self._w_random     = 0.0

        # para ESQUIVA
        self._giro_fin = None
        self._girar_derecha = True

        # temporizador
        self.create_timer(CTRL_DT, self.loop)
        self.get_logger().info("Autonomía wander activa ✅")

    # ───────── callbacks ─────────
    def cb_scan(self, msg: LaserScan) -> None:
        self.laser = list(msg.ranges)

    def cb_odom(self, msg: Odometry) -> None:
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        if self.x0 is None:
            self.x0, self.y0 = self.x, self.y
            self.get_logger().info(f"Origen ({self.x0:.2f}, {self.y0:.2f}) m")

    # ───────── bucle principal ─────────
    def loop(self) -> None:
        if self.x0 is None or not self.laser:
            return

        now_sec = self.get_clock().now().seconds_nanoseconds()[0]
        dist_radio = math.hypot(self.x - self.x0, self.y - self.y0)
        dist_min   = min(self.laser)

        # ----- gestión de transiciones ------------------------------------
        if dist_radio > RADIO_MAX:
            self.estado = Estado.RETORNO
        elif self.estado == Estado.RETORNO and dist_radio < RADIO_MAX * 0.9:
            self.estado = Estado.EXPLORA

        if self.estado == Estado.EXPLORA and dist_min < DIST_OBJ_START:
            self.estado = Estado.PERSECUCION

        if self.estado == Estado.PERSECUCION and dist_min <= DIST_OBJ_GOAL:
            self._decidir_giro()          # fija lado y _giro_fin
            self.estado = Estado.ESQUIVA

        if self.estado == Estado.ESQUIVA and now_sec >= self._giro_fin:
            self.estado = Estado.EXPLORA

        # ----- generar Twist ----------------------------------------------
        twist = Twist()

        if self.estado == Estado.EXPLORA:
            twist.linear.x = V_EXPLORE
            # cambiar rumbo cada 2-5 s
            if now_sec >= self._rumbo_expira:
                self._w_random = random.uniform(-W_EXPLORE_MAX, W_EXPLORE_MAX)
                self._rumbo_expira = now_sec + random.uniform(2.0, 5.0)
            twist.angular.z = self._w_random

        elif self.estado == Estado.PERSECUCION:
            twist.linear.x  = V_CHASE
            twist.angular.z = 0.0

        elif self.estado == Estado.ESQUIVA:
            twist.linear.x  = 0.0
            twist.angular.z = -W_TURN if self._girar_derecha else W_TURN

        elif self.estado == Estado.RETORNO:
            twist.linear.x  = 0.0
            twist.angular.z = W_TURN

        # publicar y log
        self.pub_cmd.publish(twist)
        self.get_logger().debug(
            f"{self.estado.name:<10}  r={dist_radio:5.1f}  "
            f"min={dist_min:4.2f}  v={twist.linear.x:.2f}  w={twist.angular.z:.2f}"
        )

    # ----- helper: decide giro de 90° ------------------------------------
    def _decidir_giro(self) -> None:
        n = len(self.laser)
        derecha  = sum(self.laser[: n // 3])     / (n // 3)
        izquierda= sum(self.laser[-n // 3 :])    / (n // 3)
        self._girar_derecha = derecha > izquierda
        dur_giro = (math.pi / 2) / W_TURN        # tiempo para 90°
        self._giro_fin = self.get_clock().now().seconds_nanoseconds()[0] + dur_giro


# ───────── main ─────────
def main(args=None):
    rclpy.init(args=args)
    node = Autonomia()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

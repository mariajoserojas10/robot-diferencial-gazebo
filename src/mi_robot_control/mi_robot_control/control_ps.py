#!/usr/bin/env python3
"""
control_ps.py - Manejo tipo videojuego con un control de PS5 (DualSense).

    R2 (gatillo derecho)   acelerar   (proporcional a cuánto se aprieta)
    L2 (gatillo izquierdo) frenar / reversa
    Stick izquierdo X      girar      (curva exponencial: fino al centro)

Los gatillos valen +1 en reposo y -1 apretados a fondo:
    apriete = (1 - eje) / 2  ->  0 (suelto) ... 1 (a fondo)
    v     = (apriete_R2 - apriete_L2) * v_max
    omega = signo(x) |x|^expo * w_max

Publica en /cmd_vel_teleop (mux_aeb lo lleva a /cmd_vel pasando por el AEB).
Si deja de llegar /joy (control desconectado) manda velocidad cero.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class ControlPS(Node):
    def __init__(self):
        super().__init__('control_ps')
        p = self.declare_parameter
        self.eje_r2 = p('eje_r2', 5).value
        self.eje_l2 = p('eje_l2', 2).value
        self.eje_giro = p('eje_giro', 0).value
        self.boton_turbo = p('boton_turbo', -1).value
        self.v_normal = p('v_normal', 1.0).value
        self.v_turbo = p('v_turbo', 1.0).value
        self.w_normal = p('w_normal', 3.0).value
        self.w_turbo = p('w_turbo', 3.5).value
        self.expo = p('expo', 1.6).value
        self.timeout = p('timeout', 0.5).value

        self.joy = None
        self.t_joy = None
        self.gatillos_vistos = False
        self.create_subscription(Joy, '/joy', self.cb_joy, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.create_timer(0.05, self.lazo)
        self.get_logger().info(
            f'control_ps | R2=eje {self.eje_r2} acelera, L2=eje {self.eje_l2} reversa, '
            f'stick=eje {self.eje_giro} gira | v_max={self.v_normal} m/s, w_max={self.w_normal} rad/s')

    def cb_joy(self, msg):
        self.joy = msg
        self.t_joy = self.get_clock().now().nanoseconds * 1e-9
        if abs(msg.axes[self.eje_r2]) > 0.5 or abs(msg.axes[self.eje_l2]) > 0.5:
            self.gatillos_vistos = True

    @staticmethod
    def apriete(eje):
        return max(0.0, min(1.0, (1.0 - eje) / 2.0))

    def lazo(self):
        cmd = Twist()
        ahora = self.get_clock().now().nanoseconds * 1e-9
        if self.joy is not None and ahora - self.t_joy < self.timeout and self.gatillos_vistos:
            ax, bt = self.joy.axes, self.joy.buttons
            turbo = 0 <= self.boton_turbo < len(bt) and bt[self.boton_turbo] == 1
            v_max = self.v_turbo if turbo else self.v_normal
            w_max = self.w_turbo if turbo else self.w_normal

            acel = self.apriete(ax[self.eje_r2]) - self.apriete(ax[self.eje_l2])
            x = ax[self.eje_giro]
            cmd.linear.x = acel * v_max
            cmd.angular.z = math.copysign(abs(x) ** self.expo, x) * w_max
        self.pub.publish(cmd)


def main():
    rclpy.init()
    nodo = ControlPS()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            nodo.pub.publish(Twist())
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
seguidor_pared.py - Seguimiento de pared con PID (modo autónomo, reactivo).

Circuito antihorario -> se sigue la pared IZQUIERDA (la interior).
Para cada pared se usan dos rayos del LiDAR:
    b : perpendicular a la pared (90° a la izquierda)
    a : a un ángulo theta de b, hacia adelante (90° - theta)

    alpha  = atan( (a cos(theta) - b) / (a sin(theta)) )
    D_t    = b cos(alpha)
    D_t+1  = D_t + l sin(alpha)
    e      = D_des - D_t+1
    omega  = -(Kp e + Ki ∫e + Kd de/dt)

D_des adaptativa: mitad del ancho local del pasillo (D_t izq + D_t der)/2.

Velocidad según la curvatura y el espacio libre al frente:
    v_curva  = v_max / (1 + k_curva |omega|)
    v_frente = sqrt(2 a_fren (d_frente - d_parada))
    v = clip(min(v_curva, v_frente), v_min, v_max)

Si el AEB queda activo más de t_rescate s, gira en el sitio hacia el lado
más despejado para no quedarse detenido.
Publica en /cmd_vel_auto (mux_aeb lo lleva a /cmd_vel pasando por el AEB).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64MultiArray


class SeguidorPared(Node):
    def __init__(self):
        super().__init__('seguidor_pared')
        p = self.declare_parameter
        self.kp = p('kp', 6.0).value
        self.ki = p('ki', 0.0).value
        self.kd = p('kd', 1.0).value
        self.theta = math.radians(p('theta_deg', 45.0).value)
        self.l = p('l', 0.30).value
        self.centrar = p('centrar', True).value
        self.d_des_fija = p('d_des', 0.40).value
        self.v_max = p('v_max', 1.0).value
        self.v_min = p('v_min', 0.25).value
        self.k_curva = p('k_curva', 0.15).value
        self.a_fren = p('a_fren', 1.6).value
        self.d_parada = p('d_parada', 0.25).value
        self.w_max = p('w_max', 4.0).value
        self.t_rescate = p('t_rescate', 0.8).value

        self.e_ant = None
        self.ie = 0.0
        self.t_ant = None
        self.aeb = False
        self.t_aeb = None

        self.create_subscription(LaserScan, '/scan', self.cb_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, '/aeb/activo', self.cb_aeb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel_auto', 10)
        self.pub_dbg = self.create_publisher(Float64MultiArray, '/seguidor/debug', 10)
        self.get_logger().info(
            f'seguidor_pared | Kp={self.kp} Ki={self.ki} Kd={self.kd} | theta={math.degrees(self.theta):.0f}° '
            f'l={self.l} m | v_max={self.v_max} m/s | centrar={self.centrar}')

    def cb_aeb(self, msg):
        if msg.data and not self.aeb:
            self.t_aeb = self.get_clock().now().nanoseconds * 1e-9
        self.aeb = msg.data

    def rayo(self, sc, deg, ventana=2):
        """Mediana de los rayos alrededor de 'deg' (filtra el ruido de 1 cm)."""
        n = len(sc.ranges)
        i = int(round((math.radians(deg) - sc.angle_min) / sc.angle_increment))
        vals = [sc.ranges[(i + k) % n] for k in range(-ventana, ventana + 1)]
        vals = sorted(v for v in vals if math.isfinite(v) and v >= sc.range_min)
        return vals[len(vals) // 2] if vals else sc.range_max

    def pared(self, sc, lado):
        """lado = +1 izquierda, -1 derecha. Devuelve (alpha, D_t)."""
        b = self.rayo(sc, 90.0 * lado)
        a = self.rayo(sc, lado * (90.0 - math.degrees(self.theta)))
        alpha = math.atan2(a * math.cos(self.theta) - b, a * math.sin(self.theta))
        return alpha, b * math.cos(alpha)

    def cb_scan(self, sc):
        t = sc.header.stamp.sec + sc.header.stamp.nanosec * 1e-9
        dt = 0.1 if self.t_ant is None else max(t - self.t_ant, 1e-3)
        self.t_ant = t
        cmd = Twist()

        ahora = self.get_clock().now().nanoseconds * 1e-9
        if self.aeb and self.t_aeb is not None and ahora - self.t_aeb > self.t_rescate:
            izq = self.rayo(sc, 60.0)
            der = self.rayo(sc, -60.0)
            cmd.angular.z = 1.5 if izq > der else -1.5
            self.pub.publish(cmd)
            return

        al_i, Dt_i = self.pared(sc, +1)
        al_d, Dt_d = self.pared(sc, -1)
        D1 = Dt_i + self.l * math.sin(al_i)

        ancho = Dt_i + Dt_d
        d_des = ancho / 2.0 if (self.centrar and ancho < 1.6) else self.d_des_fija

        e = d_des - D1
        de = 0.0 if self.e_ant is None else (e - self.e_ant) / dt
        self.e_ant = e
        self.ie = max(-1.0, min(1.0, self.ie + e * dt))
        w = -(self.kp * e + self.ki * self.ie + self.kd * de)
        w = max(-self.w_max, min(self.w_max, w))

        frente = min(self.rayo(sc, d, 1) for d in (-8, -4, 0, 4, 8)) - 0.1
        v_frente = math.sqrt(max(0.0, 2.0 * self.a_fren * (frente - self.d_parada)))
        v = min(self.v_max / (1.0 + self.k_curva * abs(w)), v_frente)
        v = max(self.v_min, min(self.v_max, v))

        cmd.linear.x = v
        cmd.angular.z = w
        self.pub.publish(cmd)
        self.pub_dbg.publish(Float64MultiArray(data=[e, d_des, D1, v, w, frente]))


def main():
    rclpy.init()
    nodo = SeguidorPared()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.pub.publish(Twist())
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

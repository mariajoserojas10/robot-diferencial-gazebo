#!/usr/bin/env python3
"""
mux_aeb.py - Selector de comandos + Frenado Autónomo de Emergencia (AEB).
 
Cadena de comandos:
    teleop  -> /cmd_vel_teleop --+
                                 +--> [ mux_aeb ] --> /cmd_vel --> DiffDrive
    autónomo-> /cmd_vel_auto  ---+        ^
                                 /scan, /odom
 
1) Mux: según el parámetro 'modo' (teleop | auto) toma uno de los dos topics.
   Si no llega comando en 'timeout_cmd' s, manda velocidad cero.
 
2) AEB por tiempo a la colisión (TTC), para cada rayo i del LiDAR:
        r_dot_i = -v cos(theta_i)
        TTC_i   = d_i / max(-r_dot_i, 0)
   d_i = r_i - borde(theta_i): distancia libre desde el borde del chasis.
 
   Umbral dinámico (tiempo que necesita el robot para detenerse):
        TTC_umbral(v) = t_reaccion + |v| / (2 a_frenado)
 
   Filtro de franja: solo cuentan los rayos que golpean dentro de la franja
   que barre el robot al avanzar, |r_i sin(theta_i)| <= A/2 + margen_lateral.
   Así no frena en falso por muros paralelos (rectas, estrechamiento).
 
   v = la mayor entre la velocidad medida (/odom) y la pedida (comando).
   El giro (angular.z) se deja pasar para poder salir del frenado.
"""
import math
 
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64
 
 
class MuxAEB(Node):
    def __init__(self):
        super().__init__('mux_aeb')
 
        p = self.declare_parameter
        self.modo = p('modo', 'teleop').value
        self.t_reac = p('t_reaccion', 0.20).value
        self.a_fren = p('a_frenado', 2.0).value
        self.margen = p('margen', 0.03).value
        self.margen_lat = p('margen_lateral', 0.02).value
        self.medio_largo = p('medio_largo', 0.1986 / 2).value
        self.medio_ancho = p('medio_ancho', 0.153 / 2).value
        self.timeout = p('timeout_cmd', 0.5).value
        self.v_max = p('v_max', 1.0).value
        self.w_max = p('w_max', 4.0).value
        self.aeb_on = p('aeb_activo', True).value
 
        self.cmd = Twist()
        self.t_cmd = None
        self.v_odom = 0.0
        self.scan = None
        self.frenando = False
 
        topic_in = '/cmd_vel_teleop' if self.modo == 'teleop' else '/cmd_vel_auto'
        self.create_subscription(Twist, topic_in, self.cb_cmd, 10)
        self.create_subscription(LaserScan, '/scan', self.cb_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_activo = self.create_publisher(Bool, '/aeb/activo', 10)
        self.pub_ttc = self.create_publisher(Float64, '/aeb/ttc_min', 10)
        self.create_timer(0.02, self.lazo)
 
        self.get_logger().info(
            f'mux_aeb | modo={self.modo} (entrada {topic_in}) | AEB={"ON" if self.aeb_on else "OFF"} | '
            f'TTC_umbral = {self.t_reac} s + |v|/(2*{self.a_fren})')
 
    def cb_cmd(self, msg):
        self.cmd = msg
        self.t_cmd = self.ahora()
 
    def cb_scan(self, msg):
        self.scan = msg
 
    def cb_odom(self, msg):
        self.v_odom = msg.twist.twist.linear.x
 
    def ahora(self):
        return self.get_clock().now().nanoseconds * 1e-9
 
    def borde(self, th):
        """Distancia del centro del LiDAR al borde del chasis en la dirección th."""
        c, s = abs(math.cos(th)), abs(math.sin(th))
        dx = self.medio_largo / c if c > 1e-6 else math.inf
        dy = self.medio_ancho / s if s > 1e-6 else math.inf
        return min(dx, dy)
 
    def ttc_minimo(self, v):
        """min_i TTC_i para la velocidad v."""
        if self.scan is None or abs(v) < 1e-3:
            return math.inf
        sc = self.scan
        ttc_min = math.inf
        for i, r in enumerate(sc.ranges):
            if not math.isfinite(r) or r < sc.range_min:
                continue
            th = sc.angle_min + i * sc.angle_increment
            if abs(r * math.sin(th)) > self.medio_ancho + self.margen_lat:
                continue
            r_dot = -v * math.cos(th)
            if r_dot >= 0.0:
                continue
            d = max(r - self.borde(th) - self.margen, 0.0)
            ttc = d / (-r_dot)
            if ttc < ttc_min:
                ttc_min = ttc
        return ttc_min
 
    def lazo(self):
        out = Twist()
        if self.t_cmd is not None and self.ahora() - self.t_cmd < self.timeout:
            out.linear.x = max(-self.v_max, min(self.v_max, self.cmd.linear.x))
            out.angular.z = max(-self.w_max, min(self.w_max, self.cmd.angular.z))
 
        v = out.linear.x if abs(out.linear.x) > abs(self.v_odom) else self.v_odom
        if out.linear.x * self.v_odom < 0:
            v = out.linear.x
        ttc = self.ttc_minimo(v)
        umbral = self.t_reac + abs(v) / (2.0 * self.a_fren)
 
        frenar = self.aeb_on and ttc < umbral
        if frenar:
            out.linear.x = 0.0
        if frenar != self.frenando:
            self.frenando = frenar
            if frenar:
                self.get_logger().warn(f'AEB: FRENADO  TTC={ttc:.2f} s < {umbral:.2f} s (v={v:.2f} m/s)')
            else:
                self.get_logger().info('AEB: liberado')
 
        self.pub_cmd.publish(out)
        self.pub_activo.publish(Bool(data=frenar))
        self.pub_ttc.publish(Float64(data=ttc if math.isfinite(ttc) else 99.0))
 
 
def main():
    rclpy.init()
    nodo = MuxAEB()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.pub_cmd.publish(Twist())
        nodo.destroy_node()
        rclpy.try_shutdown()
 
 
if __name__ == '__main__':
    main()

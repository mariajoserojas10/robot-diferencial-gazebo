#!/usr/bin/env python3
"""
vueltas.py - Cronómetro de vueltas y error de cierre de la odometría.

Compara tres poses del robot:
    /pose_real    pose verdadera que publica Gazebo (OdometryPublisher)
    /odom_ruedas  odometría propia (cinemática con el promedio de las ruedas de cada lado)
    /odom         odometría del plugin DiffDrive

Las dos odometrías arrancan en (0, 0, 0) en el punto de salida; se llevan al
marco del mundo con la primera pose real (pose de spawn):  q_mundo = q0 (+) q_odom.

Vuelta: cada vez que la pose real cruza la línea de meta (x = linea_x,
y en [linea_ymin, linea_ymax]) avanzando en +x. En cada cruce:
    tiempo de vuelta   (tiempo de simulación, /clock)
    error de cierre    e = || p_estimada - p_real ||   y  error de orientación
Al completar 'n_vueltas' imprime la tabla y guarda:
    ~/vueltas_resumen.csv      una fila por vuelta
    ~/vueltas_trayectoria.csv  trayectorias (para graficar con graficar_vueltas.py)
También calcula la vía efectiva del skid-steer:  L_ef = L * (giro estimado / giro real).
"""
import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def envolver(a):
    return math.atan2(math.sin(a), math.cos(a))


def al_odom(p0, p):
    """Inversa de componer: lleva una pose del mundo al marco odom (para RViz)."""
    x0, y0, t0 = p0
    dx, dy = p[0] - x0, p[1] - y0
    return (dx * math.cos(t0) + dy * math.sin(t0), -dx * math.sin(t0) + dy * math.cos(t0))


def componer(p0, q):
    """p0 (+) q : lleva una pose del marco odom al marco del mundo."""
    x0, y0, t0 = p0
    x, y, t = q
    return (x0 + x * math.cos(t0) - y * math.sin(t0),
            y0 + x * math.sin(t0) + y * math.cos(t0),
            envolver(t0 + t))


class Vueltas(Node):
    def __init__(self):
        super().__init__('vueltas')
        p = self.declare_parameter
        self.lx = p('linea_x', 0.0).value
        self.ly0 = p('linea_ymin', -0.4).value
        self.ly1 = p('linea_ymax', 0.4).value
        self.n_vueltas = p('n_vueltas', 3).value
        self.L = p('separacion_ruedas', 0.134).value

        self.real = None
        self.q0 = None
        self.est = {'ruedas': None, 'plugin': None}
        self.giro_real = 0.0
        self.giro_est = 0.0
        self.th_real_ant = None
        self.th_est_ant = None
        self.x_ant = None
        self.t_salida = None
        self.t_ult = None
        self.vueltas = []

        self.create_subscription(Odometry, '/pose_real', self.cb_real, 50)
        self.create_subscription(Odometry, '/odom_ruedas', self.cb_ruedas, 50)
        self.create_subscription(Odometry, '/odom', self.cb_plugin, 50)
        self.pub_real = self.create_publisher(Path, '/path_real', 1)
        self.pub_est = self.create_publisher(Path, '/path_ruedas', 1)
        self.path_real = Path()
        self.path_est = Path()
        self.path_real.header.frame_id = self.path_est.header.frame_id = 'odom'

        home = os.path.expanduser('~')
        self.f_tray = open(os.path.join(home, 'vueltas_trayectoria.csv'), 'w')
        self.f_tray.write('t,vuelta,x_real,y_real,th_real,x_ruedas,y_ruedas,th_ruedas,x_plugin,y_plugin,th_plugin\n')
        self.ruta_res = os.path.join(home, 'vueltas_resumen.csv')
        self.k = 0
        self.get_logger().info(
            f'Cronómetro listo: línea x={self.lx}, y∈[{self.ly0}, {self.ly1}], {self.n_vueltas} vueltas. '
            f'CSV en {home}/vueltas_*.csv')

    def cb_ruedas(self, m):
        self.est['ruedas'] = (m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation))
        th = self.est['ruedas'][2]
        if self.th_est_ant is not None:
            self.giro_est += envolver(th - self.th_est_ant)
        self.th_est_ant = th

    def cb_plugin(self, m):
        self.est['plugin'] = (m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation))

    def en_mundo(self, nombre):
        q = self.est[nombre]
        return None if (q is None or self.q0 is None) else componer(self.q0, q)

    def cb_real(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        x, y, th = m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation)
        if self.q0 is None:
            self.q0 = (x, y, th)
        if self.th_real_ant is not None:
            self.giro_real += envolver(th - self.th_real_ant)
        self.th_real_ant = th
        self.real = (x, y, th)

        if self.x_ant is not None and self.x_ant < self.lx <= x and self.ly0 <= y <= self.ly1:
            if self.t_ult is None or t - self.t_ult > 5.0:
                self.cruce(t)
        self.x_ant = x

        self.k += 1
        if self.k % 10 == 0:
            er = self.en_mundo('ruedas') or (math.nan,) * 3
            ep = self.en_mundo('plugin') or (math.nan,) * 3
            self.f_tray.write(f'{t:.3f},{len(self.vueltas)},{x:.4f},{y:.4f},{th:.4f},'
                              f'{er[0]:.4f},{er[1]:.4f},{er[2]:.4f},{ep[0]:.4f},{ep[1]:.4f},{ep[2]:.4f}\n')
            q_r = self.est['ruedas']
            for path, pose in ((self.path_real, al_odom(self.q0, (x, y))),
                               (self.path_est, q_r[:2] if q_r else (math.nan, math.nan))):
                if math.isnan(pose[0]):
                    continue
                ps = PoseStamped()
                ps.header.frame_id = 'odom'
                ps.pose.position.x, ps.pose.position.y = float(pose[0]), float(pose[1])
                path.poses.append(ps)
            self.pub_real.publish(self.path_real)
            self.pub_est.publish(self.path_est)

    def cruce(self, t):
        self.t_ult = t
        if self.t_salida is None:
            self.t_salida = t
            self.t_inicio_vuelta = t
            self.get_logger().info('¡Salida! Cronómetro en marcha.')
            return
        n = len(self.vueltas) + 1
        t_v = t - self.t_inicio_vuelta
        self.t_inicio_vuelta = t
        x, y, th = self.real
        fila = {'vuelta': n, 't_vuelta': t_v, 't_total': t - self.t_salida}
        for nombre in ('ruedas', 'plugin'):
            e = self.en_mundo(nombre)
            if e is None:
                fila[f'e_{nombre}'] = fila[f'eth_{nombre}'] = math.nan
                continue
            fila[f'e_{nombre}'] = math.hypot(e[0] - x, e[1] - y)
            fila[f'eth_{nombre}'] = math.degrees(envolver(e[2] - th))
        fila['L_ef'] = self.L * self.giro_est / self.giro_real if abs(self.giro_real) > 1 else math.nan
        self.vueltas.append(fila)
        self.get_logger().info(
            f'VUELTA {n}: {t_v:.2f} s (total {fila["t_total"]:.2f} s) | error de cierre: '
            f'ruedas {fila["e_ruedas"]*100:.1f} cm / {fila["eth_ruedas"]:.1f}°, '
            f'plugin {fila["e_plugin"]*100:.1f} cm / {fila["eth_plugin"]:.1f}°')
        self.guardar()
        if n == self.n_vueltas:
            self.tabla()

    def guardar(self):
        with open(self.ruta_res, 'w') as f:
            f.write('vuelta,t_vuelta_s,t_total_s,e_cierre_ruedas_m,e_th_ruedas_deg,'
                    'e_cierre_plugin_m,e_th_plugin_deg,L_efectiva_m\n')
            for v in self.vueltas:
                f.write(f'{v["vuelta"]},{v["t_vuelta"]:.3f},{v["t_total"]:.3f},{v["e_ruedas"]:.4f},'
                        f'{v["eth_ruedas"]:.2f},{v["e_plugin"]:.4f},{v["eth_plugin"]:.2f},{v["L_ef"]:.4f}\n')
        self.f_tray.flush()

    def tabla(self):
        lin = ['', '=' * 72, f'  RESULTADO {self.n_vueltas} VUELTAS',
               '  Vuelta | Tiempo [s] | Error cierre ruedas | Error cierre plugin',
               '  -------+------------+---------------------+--------------------']
        for v in self.vueltas:
            lin.append(f'    {v["vuelta"]}    |   {v["t_vuelta"]:6.2f}   |  {v["e_ruedas"]*100:6.1f} cm {v["eth_ruedas"]:6.1f}° '
                       f'|  {v["e_plugin"]*100:6.1f} cm {v["eth_plugin"]:6.1f}°')
        lin.append(f'  TOTAL: {self.vueltas[-1]["t_total"]:.2f} s   |   vía efectiva skid-steer: '
                   f'{self.vueltas[-1]["L_ef"]*1000:.0f} mm (geométrica {self.L*1000:.0f} mm)')
        lin.append('=' * 72)
        self.get_logger().info('\n'.join(lin))

    def destroy_node(self):
        self.f_tray.close()
        super().destroy_node()


def main():
    rclpy.init()
    nodo = Vueltas()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

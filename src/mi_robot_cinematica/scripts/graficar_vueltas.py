#!/usr/bin/env python3
"""
graficar_vueltas.py - Gráficas de odometría para el pitch.

    python3 ~/ros2_ws_dif/src/mi_robot_cinematica/scripts/graficar_vueltas.py

Lee ~/vueltas_trayectoria.csv y ~/vueltas_resumen.csv (los escribe el nodo 'vueltas')
y guarda ~/vueltas_odometria.png con:
  (1) trayectoria real vs estimada (ruedas y plugin)
  (2) error de posición a lo largo del tiempo, con las vueltas marcadas
  (3) error de cierre por vuelta (barras)
"""
import csv
import math
import os

import matplotlib.pyplot as plt

home = os.path.expanduser('~')
T = list(csv.DictReader(open(os.path.join(home, 'vueltas_trayectoria.csv'))))
R = list(csv.DictReader(open(os.path.join(home, 'vueltas_resumen.csv'))))
N = int(os.environ.get('VUELTAS', '3'))
T = [r for r in T if int(r['vuelta']) <= N]
R = R[:N]
f = lambda col: [float(r[col]) for r in T]

t = f('t')
t0 = t[0]
t = [ti - t0 for ti in t]
xr, yr = f('x_real'), f('y_real')
xe, ye = f('x_ruedas'), f('y_ruedas')
xp, yp = f('x_plugin'), f('y_plugin')
err = [math.hypot(a - c, b - d) for a, b, c, d in zip(xe, ye, xr, yr)]

fig = plt.figure(figsize=(14, 9))
ax1 = fig.add_subplot(2, 1, 1)
ax1.plot(xr, yr, 'k-', lw=2, label='Real (Gazebo)')
ax1.plot(xe, ye, 'm-', lw=1, label='Estimada: cinemática de ruedas')
ax1.plot(xp, yp, 'g--', lw=1, label='Estimada: plugin DiffDrive')
ax1.plot([0, 0], [-0.4, 0.4], 'b-', lw=3, label='Línea de meta')
ax1.set_aspect('equal')
ax1.grid(True, alpha=0.3)
ax1.legend(loc='upper left', fontsize=9)
ax1.set_title(f'Trayectoria en {N} vueltas: real vs odometría')
ax1.set_xlabel('x [m]')
ax1.set_ylabel('y [m]')

ax2 = fig.add_subplot(2, 2, 3)
ax2.plot(t, [e * 100 for e in err], 'm-')
v = [int(r['vuelta']) for r in T]
for i in range(1, len(v)):
    if v[i] != v[i - 1]:
        ax2.axvline(t[i], color='b', ls=':', alpha=0.6)
        ax2.text(t[i], max(err) * 100, f' V{v[i]}', color='b', va='top', fontsize=8)
ax2.set_xlabel('tiempo de simulación [s]')
ax2.set_ylabel('error de posición [cm]')
ax2.set_title('Deriva de la odometría de ruedas')
ax2.grid(True, alpha=0.3)

ax3 = fig.add_subplot(2, 2, 4)
n = [int(r['vuelta']) for r in R]
ax3.bar([i - 0.2 for i in n], [float(r['e_cierre_ruedas_m']) * 100 for r in R], 0.4, label='ruedas', color='m')
ax3.bar([i + 0.2 for i in n], [float(r['e_cierre_plugin_m']) * 100 for r in R], 0.4, label='plugin', color='g')
for r in R:
    ax3.text(int(r['vuelta']) - 0.2, float(r['e_cierre_ruedas_m']) * 100,
             f"{float(r['e_cierre_ruedas_m'])*100:.0f}", ha='center', va='bottom', fontsize=9)
ax3.set_xticks(n)
ax3.set_xlabel('vuelta')
ax3.set_ylabel('error de cierre [cm]')
ax3.set_title('Error de cierre por vuelta')
ax3.legend()
ax3.grid(True, axis='y', alpha=0.3)

plt.tight_layout()
salida = os.path.join(home, 'vueltas_odometria.png')
plt.savefig(salida, dpi=120)
print(f'Gráfica guardada en {salida}')
print('Vuelta | tiempo [s] | error cierre ruedas [cm] | e_theta [°] | L efectiva [mm]')
for r in R:
    print(f"  {r['vuelta']}    | {float(r['t_vuelta_s']):7.2f}    | {float(r['e_cierre_ruedas_m'])*100:8.1f}"
          f"                 | {float(r['e_th_ruedas_deg']):6.1f}     | {float(r['L_efectiva_m'])*1000:6.0f}")

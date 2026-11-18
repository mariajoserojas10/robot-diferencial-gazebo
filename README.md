# Robot diferencial MicroROS en Gazebo: AEB, seguimiento de pared y competencia

Universidad EIA – Ingeniería Mecatrónica – Robótica Móvil (Entrega 2)

Robot **MicroROS de Yahboom** (4 ruedas, skid-steer) simulado en **ROS 2 Jazzy + Gazebo Harmonic**, con dos modos de operación:
- **Teleoperado:** control de PS5 o teclado.
- **Autónomo:** seguimiento de pared con PID.

En ambos modos actúa un **frenado autónomo de emergencia (AEB)** por tiempo a la colisión (TTC), con prioridad sobre cualquier comando.

Documentación de la entrega 1: https://somber-nutria-119.notion.site/Robot-Diferencial-en-Gazebo-Entrega-1-Rob-tica-M-vil-3f7364a67f0680d2a9b3c00dd1599304

## Paquetes

| Paquete | Contenido |
|---|---|
| `mi_robot_diferencial` | Robot (`urdf/`), pistas (`worlds/`, `models/`), diseño CAD de la pista propia (`cad/`), bridge y control PS5 (`config/`), launch, RViz |
| `mi_robot_control` | `mux_aeb` (selector de comandos + AEB), `seguidor_pared` (PID), `control_ps` (control PS5) |
| `mi_robot_cinematica` | `odom_ruedas` (odometría por cinemática de llantas), `vueltas` (cronómetro y error de cierre), `scripts/graficar_vueltas.py` |

## Dependencias

    sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
      ros-jazzy-teleop-twist-keyboard ros-jazzy-joy ros-jazzy-rviz2 \
      python3-matplotlib xterm openscad

## Compilar

    cd ~/ros2_ws_dif
    colcon build
    source install/setup.bash

## Ejecutar (un solo launch)

    ros2 launch mi_robot_diferencial mi_robot.launch.py modo:=<teleop|auto> pista:=<competencia|propia>

| Argumento | Opciones | Por defecto | Descripción |
|---|---|---|---|
| `modo` | `teleop`, `auto` | `teleop` | Teleoperado o seguimiento de pared |
| `pista` | `competencia`, `propia` | `competencia` | Pista del profesor o pista propia (CAD) |
| `control` | `ps`, `teclado` | `ps` | Solo en `teleop`. El teclado se abre en su propia ventana |
| `v_max` | número [m/s] | `1.0` | Velocidad máxima del modo autónomo |
| `obstaculo` | `true`, `false` | `false` | Pone una caja en la recta principal para demostrar el AEB |

Ejemplos:

    ros2 launch mi_robot_diferencial mi_robot.launch.py modo:=auto                       # autónomo, competencia
    ros2 launch mi_robot_diferencial mi_robot.launch.py modo:=auto pista:=propia         # autónomo, pista propia
    ros2 launch mi_robot_diferencial mi_robot.launch.py modo:=teleop                     # control PS5, competencia
    ros2 launch mi_robot_diferencial mi_robot.launch.py modo:=teleop control:=teclado    # teclado
    ros2 launch mi_robot_diferencial mi_robot.launch.py modo:=auto obstaculo:=true       # prueba del AEB

**Control PS5:** R2 acelera (proporcional), L2 frena o retrocede, stick izquierdo gira.
**Teclado:** `i` avanzar, `,` retroceder, `j`/`l` girar, `k` frenar.

## Cadena de comandos

    control_ps / teclado -> /cmd_vel_teleop -+
                                             +-> mux_aeb (AEB por TTC) -> /cmd_vel -> DiffDrive (Gazebo)
    seguidor_pared -----> /cmd_vel_auto  ----+          ^
                                                   /scan, /odom

## Robot (MicroROS)

Vía 134 mm, distancia entre ejes 96 mm, rueda de 49 mm, masa 0.906 kg, 4 ruedas motrices (2 juntas por lado en el DiffDrive).
LiDAR MS200: 450 muestras en 360°, 0.03–12 m, 10 Hz, σ = 0.01 m, a 108 mm del suelo.
Límites: 1.0 m/s, 4.0 rad/s, 2.0 m/s².

## AEB (`mux_aeb`)

Para cada rayo: `ṙ_i = −v cos θ_i`, `TTC_i = d_i / max(−ṙ_i, 0)`, con `d_i` medida desde el borde del chasis.
Umbral dinámico: `TTC_umbral = t_reacción + |v| / (2 a_frenado)`, con 0.2 s y 2.0 m/s² (a 1 m/s: 0.45 s).
Solo cuentan los rayos dentro de la **franja que barre el robot**, curvada según su giro (recto, girando, adelante o en reversa), para no frenar en falso en curvas ni en el estrechamiento.

## Seguimiento de pared (`seguidor_pared`)

Sigue la pared izquierda (circuito antihorario) con los rayos b = 90° y a = 45°:
`α = atan((a cosθ − b)/(a sinθ))`, `D_t = b cos α`, `D_t+1 = D_t + l sin α`, `e = D_des − D_t+1`, `ω = −(Kp e + Ki ∫e + Kd ė)`.
`D_des` = mitad del ancho local del pasillo (centrado). Kp = 6.0, Ki = 0, Kd = 1.0, l = 0.30 m.
Velocidad: `v = min(v_max / (1 + 0.15|ω|), √(2·1.6·(d_frente − 0.25)))`.

## Odometría en varias vueltas (`odom_ruedas`, `vueltas`)

`ω_D` y `ω_I` son el promedio de las dos ruedas de cada lado. El nodo `vueltas` cronometra cada vuelta al cruzar la meta y calcula el error de cierre contra la pose real de Gazebo (`/pose_real`).
**Vía efectiva del skid-steer:** con la vía geométrica (134 mm) el error de cierre llega a metros y es errático; calibrada con la vía efectiva medida, **L_ef = 181 mm**, crece de forma lineal (unos 30–60 cm por vuelta).

Gráficas (después de 3 vueltas): `python3 ~/ros2_ws_dif/src/mi_robot_cinematica/scripts/graficar_vueltas.py`

## Pista propia (`cad/pista_propia.scad`)

Diseñada en **OpenSCAD**, en metros, a partir de su línea central: recta principal (6.5 m), curva amplia de 180° (R = 1.6 m), estrechamiento a 0.45 m, chicana (R = 1.0 m), curva cerrada de 90° (R = 0.8 m) y curva media de 90° (R = 1.4 m). Pasillo de 0.80 m, muros de 0.30 m, 24.1 m por vuelta.

    openscad -o src/mi_robot_diferencial/models/pista_propia/meshes/pista_propia.stl src/mi_robot_diferencial/cad/pista_propia.scad

## Resultados (modo autónomo, 3 vueltas, tiempo de simulación)

| Pista | Vuelta 1 | Vuelta 2 | Vuelta 3 | Total | Choques | Frenados falsos |
|---|---|---|---|---|---|---|
| Competencia | 39.76 s | 39.76 s | 39.72 s | 119.24 s | 0 | 0 |
| Propia | 28.04 s | 28.08 s | 28.06 s | 84.18 s | 0 | 0 |

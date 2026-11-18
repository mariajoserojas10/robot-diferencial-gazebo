// =====================================================================
//  Pista propia - Robot diferencial MicroROS (ROS 2 Jazzy + Gazebo)
//  Unidades: METROS (el STL exportado queda en metros).
//
//  El circuito se define por su LÍNEA CENTRAL, como una secuencia de
//  tramos rectos ("S", largo) y arcos ("A", radio, ángulo; + = izquierda).
//  Los muros se generan desplazando esa línea ±(ANCHO/2).
//  Sentido de recorrido: antihorario. Salida/meta en x = 0, y en [-0.4, 0.4].
//
//  Secciones:
//   1. Recta principal (6.5 m) con la línea de salida/meta
//   2. Curva amplia de 180° (R = 1.6 m, dos arcos de 90° con 0.8 m de recta)
//   3. Recta superior (3.2 m) con ESTRECHAMIENTO a 0.45 m
//      (1.2 m de largo + rampas de 0.7 m)
//   4. Chicana (arcos R = 1.0 m: +45°, -90°, +45°)
//   5. Curva cerrada de 90° (R = 0.8 m)
//   6. Recta de bajada (1.8 m) y curva media de 90° (R = 1.4 m)
//  Largo de la línea central: 24.1 m por vuelta
//
//  Exportar:  openscad -o pista_propia.stl pista_propia.scad
// =====================================================================

ANCHO   = 0.80;   // ancho nominal del pasillo [m]
ESPESOR = 0.05;   // espesor de los muros [m]
ALTO    = 0.30;   // alto de los muros [m] (mínimo pedido: 0.25)

// Estrechamiento en la recta superior
EST_ANCHO = 0.45;  // ancho libre en el estrechamiento [m] (mínimo pedido: 0.40)
EST_X0    = 2.30;  // inicio (x) [m]
EST_X1    = 3.50;  // fin (x) [m]
EST_RAMPA = 0.70;  // largo de la rampa de entrada/salida [m]

INICIO = [-2.0, 0.0];   // punto de partida de la línea central (rumbo 0°)

TRAMOS = [
    ["S", 6.5],             // 1. recta principal (salida/meta)
    ["A", 1.6,  90],        // 2. curva amplia de 180° (1/2)
    ["S", 0.8],
    ["A", 1.6,  90],        //    curva amplia de 180° (2/2)
    ["S", 3.2],             // 3. recta superior con estrechamiento
    ["A", 1.0,  45],        // 4. chicana
    ["A", 1.0, -90],
    ["A", 1.0,  45],
    ["S", 1.4],
    ["A", 0.8,  90],        // 5. curva cerrada de 90°
    ["S", 1.8],             // 6. recta de bajada
    ["A", 1.4,  90],        //    curva media de 90°
    ["S", 0.328]            //    cierra el circuito en el punto de partida
];

$fa = 1;
$fs = 0.01;

// ---------- Línea central (tortuga) ----------
function tramo(p, h, s) =
    s[0] == "S"
        ? [[p + s[1] * [cos(h), sin(h)]], h]
        : let(R = s[1], a = s[2], n = max(4, ceil(abs(a) / 2)),
              c = p + R * sign(a) * [-sin(h), cos(h)],
              a0 = atan2(p[1] - c[1], p[0] - c[0]))
          [[for (k = [1:n]) c + R * [cos(a0 + a * k / n), sin(a0 + a * k / n)]], h + a];

function tortuga(i, p, h) =
    i >= len(TRAMOS) ? [] :
    let(r = tramo(p, h, TRAMOS[i]), q = r[0])
    concat(q, tortuga(i + 1, q[len(q) - 1], r[1]));

PUNTOS  = tortuga(0, INICIO, 0);
CENTRAL = [for (i = [0:len(PUNTOS) - 2]) PUNTOS[i]];

// ---------- Muros ----------
module muro_exterior() {
    difference() {
        offset(r =  ANCHO / 2 + ESPESOR) polygon(CENTRAL);
        offset(r =  ANCHO / 2)           polygon(CENTRAL);
    }
}

module muro_interior() {
    difference() {
        offset(r = -ANCHO / 2)             polygon(CENTRAL);
        offset(r = -(ANCHO / 2 + ESPESOR)) polygon(CENTRAL);
    }
}

// Bloque del estrechamiento pegado a un muro (con rampas para no frenar en seco)
module bloque_estrechamiento(y_muro, dir) {
    d = (ANCHO - EST_ANCHO) / 2;
    polygon([
        [EST_X0 - EST_RAMPA, y_muro],
        [EST_X0,             y_muro + dir * d],
        [EST_X1,             y_muro + dir * d],
        [EST_X1 + EST_RAMPA, y_muro],
        [EST_X1 + EST_RAMPA, y_muro - dir * 0.01],
        [EST_X0 - EST_RAMPA, y_muro - dir * 0.01]
    ]);
}

module estrechamiento() {
    y_c = 4.0;
    bloque_estrechamiento(y_c + ANCHO / 2, -1);
    bloque_estrechamiento(y_c - ANCHO / 2, +1);
}

linear_extrude(height = ALTO)
    union() {
        muro_exterior();
        muro_interior();
        estrechamiento();
    }

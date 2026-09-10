# Reporte Técnico: Hito 1 - ABB IRB 120

## Hoja de Definición de Juntas y Justificación Física

**Nota sobre los límites articulares:** 
Los rangos máximos y mínimos han sido acotados respecto al datasheet oficial para restringir el modelo geométrico a su **Espacio de Configuración Libre**. Dado que RViz es un visualizador cinemático puro carente de un motor de físicas, se limitaron matemáticamente las articulaciones críticas (J2, J3, J5 y J6) para prevenir autocolisiones tridimensionales entre las mallas CAD y evitar enredos de cableado en la simulación, manteniendo intacta la precisión de la Cinemática Directa.

| Joint | Parent | Child | `xyz` (m) | `rpy` (rad) | `axis` | Límites (rad) | Justificación Física |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **J1** | `base_link` | `link_1` | `0.000 0.000 0.000` | `0 0 0` | `0 0 1` | `[-2.879, 2.879]` | Giro horizontal de la cintura. Se mantiene el límite íntegro de fábrica. |
| **J2** | `link_1` | `link_2` | `0.000 0.000 0.290` | `0 0 0` | `0 1 0` | `[-1.221, 1.221]` | Elevación del hombro. Acotado a ±70° para evitar que la carcasa colisione con la base. |
| **J3** | `link_2` | `link_3` | `0.000 0.000 0.270` | `0 0 0` | `0 1 0` | `[-1.745, 0.785]` | Articulación del codo. Acotado a +45° en flexión trasera para evitar impacto contra el hombro. |
| **J4** | `link_3` | `link_4` | `0.070 0.000 0.000` | `0 0 0` | `1 0 0` | `[-2.792, 2.792]` | Torsión longitudinal del antebrazo. Se mantiene el límite íntegro de fábrica. |
| **J5** | `link_4` | `link_5` | `0.302 0.000 0.000` | `0 0 0` | `0 1 0` | `[-1.570, 1.570]` | Flexión de muñeca. Acotado a ±90° impidiendo que el efector final golpee el antebrazo. |
| **J6** | `link_5` | `link_6` | `0.072 0.000 0.000` | `0 0 0` | `1 0 0` | `[-3.141, 3.141]` | Giro de brida. Acotado a ±180° (1 vuelta) para prevenir enredo de cables en la realidad. |

## Validación Cruzada FK ↔ URDF

| Conf. | Valores Articulares $q$ (rad) | Posición FK Calculada $\mathbf{p}_{\text{analítico}}$ (m) | Observación en URDF / RViz $\mathbf{p}_{\text{TF2}}$ (m) | ¿Coinciden? |
| :--- | :--- | :--- | :--- | :--- |
| **A** | $q^{(A)} = [0, 0, 0, 0, 0, 0]^T$ *(Home)* | $X = 0.374000$<br>$Y = 0.000000$<br>$Z = 0.630000$ | $X = 0.374000$<br>$Y = 0.000000$<br>$Z = 0.630000$ | **SÍ**<br>($\Delta E < 10^{-6}\text{ m}$) |
| **B** | $q^{(B)} = [0.5, 0, 0, 0, 0, 0]^T$ | $X = 0.328216$<br>$Y = 0.179305$<br>$Z = 0.630000$ | $X = 0.328216$<br>$Y = 0.179305$<br>$Z = 0.630000$ | **SÍ**<br>($\Delta E < 10^{-6}\text{ m}$) |
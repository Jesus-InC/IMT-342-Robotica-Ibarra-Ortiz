#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from tf2_ros import TransformListener, Buffer
import numpy as np

class CinematicaDirectaNode(Node):
    def __init__(self):
        super().__init__('cinematica_directa_node')
        
        # Suscriptor al tópico de articulaciones
        self.sub_joints = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        
        # Búfer y oyente para el árbol TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Variables de estado
        self.current_q = None
        self.last_moving_q = None
        self.last_joint_msg_time = None
        self.last_print_time = self.get_clock().now()
        self.last_warn_time = self.get_clock().now()
        
        self.is_moving = False
        self.pending_settled_print = False
        
        # Parámetros de control
        self.settling_delay_sec = 0.30  # Espera 300 ms de quietud para considerar la posición asentada
        self.periodic_delay_sec = 3.0   # Periodo de reporte continuo en reposo
        self.movement_threshold = 0.0005 # Umbral para filtrar ruido numérico (rad)

        # Temporizador supervisor a 10 Hz (cada 100 ms)
        self.supervisor_timer = self.create_timer(0.1, self.supervisor_callback)

        self.get_logger().info("Nodo de Cinemática Directa Iniciado correctamente.")

    def joint_callback(self, msg: JointState):
        if len(msg.position) < 6:
            return

        q_incoming = np.array(msg.position[:6])
        now = self.get_clock().now()

        # Primer mensaje recibido
        if self.current_q is None:
            self.current_q = q_incoming
            self.last_moving_q = q_incoming
            self.last_joint_msg_time = now
            return

        # Verificar si realmente hubo un cambio físico relevante en los sliders
        if np.linalg.norm(q_incoming - self.last_moving_q) > self.movement_threshold:
            self.current_q = q_incoming
            self.last_moving_q = q_incoming
            self.last_joint_msg_time = now
            self.is_moving = True
            self.pending_settled_print = True

    def supervisor_callback(self):
        now = self.get_clock().now()

        # 1. DIAGNÓSTICO: Espera de datos iniciales
        if self.current_q is None:
            if (now - self.last_warn_time).nanoseconds / 1e9 >= 3.0:
                self.get_logger().warn(
                    "Esperando datos en /joint_states... Verifica que RViz o la GUI estén activas."
                )
                self.last_warn_time = now
            return

        time_since_last_msg = (now - self.last_joint_msg_time).nanoseconds / 1e9
        time_since_last_print = (now - self.last_print_time).nanoseconds / 1e9

        # 2. EVALUACIÓN DE ASENTAMIENTO (El movimiento se detuvo y transcurrieron 300 ms)
        if self.pending_settled_print and (time_since_last_msg >= self.settling_delay_sec):
            self.pending_settled_print = False
            self.is_moving = False
            self.last_print_time = now
            self.evaluar_cinematica(causa="POSICIÓN ASENTADA 🎯")
            return

        # 3. EVALUACIÓN PERIÓDICA (Robot estático y transcurrieron 3 segundos sin impresiones)
        if not self.is_moving and not self.pending_settled_print and (time_since_last_print >= self.periodic_delay_sec):
            self.last_print_time = now
            self.evaluar_cinematica(causa="LECTURA PERIÓDICA EN REPOSO (3s) ⏱️")

    # --- MATRICES DE TRANSFORMACIÓN HOMOGÉNEA (4x4) ---
    def rot_x(self, q):
        return np.array([
            [1, 0, 0, 0],
            [0, np.cos(q), -np.sin(q), 0],
            [0, np.sin(q), np.cos(q), 0],
            [0, 0, 0, 1]
        ])

    def rot_y(self, q):
        return np.array([
            [np.cos(q), 0, np.sin(q), 0],
            [0, 1, 0, 0],
            [-np.sin(q), 0, np.cos(q), 0],
            [0, 0, 0, 1]
        ])

    def rot_z(self, q):
        return np.array([
            [np.cos(q), -np.sin(q), 0, 0],
            [np.sin(q), np.cos(q), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

    def trans(self, x, y, z):
        return np.array([
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1]
        ])

    def evaluar_cinematica(self, causa=""):
        if self.current_q is None:
            return

        q1, q2, q3, q4, q5, q6 = self.current_q

        # Cadena cinemática directa T0_6 = T0_1 * T1_2 * ... * T5_6
        T0_1 = self.rot_z(q1)
        T1_2 = self.trans(0.0, 0.0, 0.290) @ self.rot_y(q2)
        T2_3 = self.trans(0.0, 0.0, 0.270) @ self.rot_y(q3)
        T3_4 = self.trans(0.0, 0.0, 0.070) @ self.rot_x(q4)
        T4_5 = self.trans(0.302, 0.0, 0.0) @ self.rot_y(q5)
        T5_6 = self.trans(0.072, 0.0, 0.0) @ self.rot_x(q6)

        T0_6 = T0_1 @ T1_2 @ T2_3 @ T3_4 @ T4_5 @ T5_6
        p_analitico = T0_6[0:3, 3]

        # Lectura de la transformada TF2 (RViz)
        try:
            trans_tf = self.tf_buffer.lookup_transform(
                'base_link',
                'gripper_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            
            p_tf = np.array([
                trans_tf.transform.translation.x,
                trans_tf.transform.translation.y,
                trans_tf.transform.translation.z
            ])

            # Distancia euclidiana
            error = np.linalg.norm(p_analitico - p_tf)
            estado = "APROBADO ✅" if error < 1e-3 else "REPROBADO ❌"

            print(f"--- [{causa}] ---")
            print(f"[Analítico]: x={p_analitico[0]:.6f}, y={p_analitico[1]:.6f}, z={p_analitico[2]:.6f}")
            print(f"[RViz TF]  : x={p_tf[0]:.6f}, y={p_tf[1]:.6f}, z={p_tf[2]:.6f}")
            print(f"[Error ΔE] : {error:.8e} m | Pasaporte Hito 1: {estado}\n")

        except Exception as ex:
            self.get_logger().warn(f"Sincronizando árbol TF2... ({ex})", throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = CinematicaDirectaNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

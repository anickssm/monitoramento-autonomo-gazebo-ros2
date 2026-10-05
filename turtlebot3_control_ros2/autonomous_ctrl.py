#!/usr/bin/env python3

import math
import signal
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class AutonomousCtrl(Node):

    def __init__(self):
        super().__init__('autonomous_ctrl')

        self.scan = None
        self.odom = None

        # Estados do controlador:
        # FORWARD: avanço normal
        # TURN: giro parado para desviar
        self.state = 'FORWARD'

        # Positivo = esquerda
        # Negativo = direita
        self.turn_direction = 1.0
        self.last_turn_direction = 1.0

        # Exige algumas leituras livres consecutivas
        # antes de encerrar uma curva.
        self.clear_cycles = 0
        self.log_counter = 0

        # Mapa discreto fornecido pelo professor para o Stage 4.
        #
        # 0 = célula não considerada
        # 1 = célula disponível ainda não visitada
        # 2 = célula visitada
        self.map = np.array([
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        ], dtype=np.uint8)

        self.coverage_percentage = 0.0

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Controle executado dez vezes por segundo.
        self.timer = self.create_timer(
            0.1,
            self.control
        )

        self.get_logger().info(
            'Controlador autonomo iniciado.'
        )

    def scan_callback(self, msg):
        self.scan = msg

    def odom_callback(self, msg):
        self.odom = msg

    def publish_velocity(self, linear, angular):
        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)

        self.cmd_pub.publish(msg)

    def update_coverage(self):
        """
        Converte a posição da odometria em uma célula
        da matriz e atualiza o percentual de cobertura.
        """

        if self.odom is None:
            return

        # Quatro células para cada metro:
        # uma célula representa aproximadamente 0,25 m.
        map_resolution = 4.0

        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y

        # Conversão usada no código-base fornecido pelo professor.
        index_x = -int(x * map_resolution)
        index_y = -int(y * map_resolution)

        index_x += self.map.shape[0] // 2
        index_y += self.map.shape[1] // 2

        # Impede índices fora da matriz.
        index_x = int(
            np.clip(index_x, 0, self.map.shape[0] - 1)
        )

        index_y = int(
            np.clip(index_y, 0, self.map.shape[1] - 1)
        )

        # Só registra se for uma célula disponível ainda não visitada.
        if self.map[index_x, index_y] != 1:
            return

        self.map[index_x, index_y] = 2

        visited = int(
            np.count_nonzero(self.map == 2)
        )

        available = int(
            np.count_nonzero(self.map == 1)
            + visited
        )

        if available > 0:
            self.coverage_percentage = (
                100.0 * visited / available
            )

        self.get_logger().info(
            'Nova regiao visitada | '
            f'celula=({index_x}, {index_y}) | '
            f'cobertura={self.coverage_percentage:.2f}% '
            f'({visited}/{available})'
        )

    def sector_distance(
        self,
        start_degree,
        end_degree,
        percentile=0.20
    ):
        """
        Calcula uma distância representativa de um setor do LiDAR.

        Os ângulos são analisados usando angle_min e angle_increment.
        Isso evita depender da suposição de que cada índice corresponde
        exatamente a um grau.

        O percentil reduz a influência de uma leitura isolada sem
        ignorar obstáculos reais.
        """

        valid_values = []

        for index, value in enumerate(self.scan.ranges):

            if not math.isfinite(value):
                continue

            if value < self.scan.range_min:
                continue

            if value > self.scan.range_max:
                continue

            angle_radians = (
                self.scan.angle_min
                + index * self.scan.angle_increment
            )

            angle_degrees = (
                math.degrees(angle_radians) % 360.0
            )

            if start_degree <= end_degree:
                inside_sector = (
                    start_degree
                    <= angle_degrees
                    <= end_degree
                )
            else:
                # Setor que atravessa zero graus.
                inside_sector = (
                    angle_degrees >= start_degree
                    or angle_degrees <= end_degree
                )

            if inside_sector:
                valid_values.append(float(value))

        if not valid_values:
            return float(self.scan.range_max)

        valid_values.sort()

        index = int(
            percentile * (len(valid_values) - 1)
        )

        return valid_values[index]

    def choose_turn_direction(
        self,
        left,
        right,
        front_left,
        front_right
    ):
        """
        Escolhe o lado com mais espaço e mantém a decisão
        quando esquerda e direita são semelhantes.
        """

        left_score = (
            0.65 * left
            + 0.35 * front_left
        )

        right_score = (
            0.65 * right
            + 0.35 * front_right
        )

        # Mantém a decisão anterior quando os lados são parecidos.
        # Isso evita zigue-zague e troca contínua de direção.
        if abs(left_score - right_score) < 0.12:
            return self.last_turn_direction

        if left_score > right_score:
            self.last_turn_direction = 1.0
        else:
            self.last_turn_direction = -1.0

        return self.last_turn_direction

    def show_status(
        self,
        front,
        front_left,
        front_right,
        left,
        right
    ):
        # Mostra aproximadamente uma mensagem por segundo.
        self.log_counter += 1

        if self.log_counter < 10:
            return

        self.log_counter = 0

        self.get_logger().info(
            f'Estado={self.state} | '
            f'frente={front:.2f} m | '
            f'frente_esq={front_left:.2f} m | '
            f'frente_dir={front_right:.2f} m | '
            f'esq={left:.2f} m | '
            f'dir={right:.2f} m | '
            f'cobertura={self.coverage_percentage:.2f}%'
        )

    def control(self):

        # Sem dados válidos do LiDAR ou odometria,
        # o robô permanece parado.
        if self.scan is None or self.odom is None:
            self.publish_velocity(0.0, 0.0)
            return

        if len(self.scan.ranges) < 10:
            self.publish_velocity(0.0, 0.0)
            return

        # Registra a região ocupada pelo robô.
        self.update_coverage()

        # 0° é a frente.
        # 90° é a esquerda.
        # 270° é a direita.

        front = self.sector_distance(
            345,
            15,
            percentile=0.12
        )

        front_left = self.sector_distance(
            15,
            55,
            percentile=0.18
        )

        left = self.sector_distance(
            55,
            125,
            percentile=0.35
        )

        front_right = self.sector_distance(
            305,
            345,
            percentile=0.18
        )

        right = self.sector_distance(
            235,
            305,
            percentile=0.35
        )

        # Distâncias de controle, em metros.
        stop_distance = 0.42
        caution_distance = 0.72

        diagonal_stop_distance = 0.25
        diagonal_caution_distance = 0.40

        turn_clear_distance = 0.68
        turn_side_clearance = 0.36

        # =========================================================
        # ESTADO DE GIRO
        # =========================================================

        if self.state == 'TURN':

            if self.turn_direction > 0:
                selected_side = front_left
            else:
                selected_side = front_right

            clear_path = (
                front > turn_clear_distance
                and selected_side > turn_side_clearance
            )

            if clear_path:
                self.clear_cycles += 1
            else:
                self.clear_cycles = 0

            # Só encerra o giro após cinco leituras livres
            # consecutivas, evitando oscilação.
            if self.clear_cycles >= 5:
                self.state = 'FORWARD'
                self.clear_cycles = 0

                self.publish_velocity(
                    0.10,
                    0.0
                )

            else:
                # Giro parado para evitar colisão e capotamento.
                self.publish_velocity(
                    0.0,
                    0.55 * self.turn_direction
                )

            self.show_status(
                front,
                front_left,
                front_right,
                left,
                right
            )

            return

        # =========================================================
        # OBSTÁCULO PRÓXIMO
        # =========================================================

        obstacle_close = (
            front < stop_distance
            or front_left < diagonal_stop_distance
            or front_right < diagonal_stop_distance
        )

        if obstacle_close:

            self.turn_direction = (
                self.choose_turn_direction(
                    left,
                    right,
                    front_left,
                    front_right
                )
            )

            self.state = 'TURN'
            self.clear_cycles = 0

            self.publish_velocity(
                0.0,
                0.55 * self.turn_direction
            )

            self.show_status(
                front,
                front_left,
                front_right,
                left,
                right
            )

            return

        # =========================================================
        # OBSTÁCULO SE APROXIMANDO
        # =========================================================

        obstacle_approaching = (
            front < caution_distance
            or front_left < diagonal_caution_distance
            or front_right < diagonal_caution_distance
        )

        if obstacle_approaching:

            direction = self.choose_turn_direction(
                left,
                right,
                front_left,
                front_right
            )

            self.publish_velocity(
                0.10,
                0.28 * direction
            )

            self.show_status(
                front,
                front_left,
                front_right,
                left,
                right
            )

            return

        # =========================================================
        # CAMINHO LIVRE
        # =========================================================

        angular_correction = 0.0

        # Só aplica centralização quando uma lateral está
        # relativamente próxima. Em área aberta, segue reto.
        if min(left, right) < 0.65:

            side_error = left - right

            angular_correction = max(
                -0.16,
                min(
                    0.16,
                    0.30 * side_error
                )
            )

        self.publish_velocity(
            0.20,
            angular_correction
        )

        self.show_status(
            front,
            front_left,
            front_right,
            left,
            right
        )

    def stop_robot(self):
        """
        Publica velocidade zero várias vezes antes de encerrar.
        """

        for _ in range(20):
            self.publish_velocity(0.0, 0.0)
            time.sleep(0.03)


def main(args=None):

    # Desabilita o tratamento automático do Ctrl+C.
    # Assim o comando zero é enviado antes de desligar o ROS 2.
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )

    node = AutonomousCtrl()

    running = True

    def request_shutdown(signum, frame):
        nonlocal running
        running = False

    signal.signal(
        signal.SIGINT,
        request_shutdown
    )

    signal.signal(
        signal.SIGTERM,
        request_shutdown
    )

    try:
        while rclpy.ok() and running:
            rclpy.spin_once(
                node,
                timeout_sec=0.1
            )

    finally:
        node.get_logger().info(
            f'Cobertura final: '
            f'{node.coverage_percentage:.2f}%'
        )

        node.stop_robot()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

# Monitoramento Autônomo com TurtleBot3 no Gazebo ROS 2

Projeto desenvolvido para a disciplina **Projeto de Robôs II — PRIA**.

- **Aluno:** Arthur Muritiba
- **Professor:** Ricardo Grando
- **Ambiente:** Ubuntu 24.04, ROS 2 Jazzy e Gazebo Harmonic

## Objetivo

Projetar e implementar um sistema robótico autônomo capaz de monitorar a maior área possível de um cenário simulado no Gazebo.

A solução integra:

- percepção do ambiente por LiDAR;
- tomada de decisão baseada nas distâncias medidas;
- controle autônomo de movimento;
- prevenção de colisões;
- odometria;
- medição do percentual de cobertura do cenário.

Após o início da missão, o robô executa sem teleoperação.

## Resultado obtido

Durante os testes no cenário `turtlebot3_dqn_stage4`, o TurtleBot3 executou a navegação autonomamente, utilizando o LiDAR para detectar obstáculos e selecionar direções de movimento.

Foi observada uma cobertura aproximada de **60% da área considerada pela matriz discreta**, sem colisões observadas durante a execução utilizada para validação.

O percentual pode variar conforme o tempo de execução e as decisões tomadas pelo controlador.

## Arquitetura

```text
                         ┌─────────────────────┐
                         │   Gazebo Stage 4    │
                         └──────────┬──────────┘
                                    │
                  ┌─────────────────┴─────────────────┐
                  │                                   │
             /scan — LiDAR                       /odom — Odometria
                  │                                   │
                  ▼                                   ▼
        Processamento de setores             Matriz de cobertura
                  │                                   │
                  ▼                                   ▼
        Máquina de estados                    Percentual visitado
        FORWARD / TURN                                │
                  │                                   │
                  └─────────────────┬─────────────────┘
                                    ▼
                         /cmd_vel — TwistStamped
                                    │
                                    ▼
                              TurtleBot3 Burger
```

## Interfaces ROS 2

| Tópico | Tipo | Função |
|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | Distâncias medidas pelo LiDAR |
| `/odom` | `nav_msgs/msg/Odometry` | Posição e deslocamento do robô |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Comandos de velocidade |
| `/tf` | `tf2_msgs/msg/TFMessage` | Transformações entre os frames |
| `/joint_states` | `sensor_msgs/msg/JointState` | Estado das juntas do robô |

## Estratégia de controle

O nó `autonomous_ctrl` utiliza uma máquina de estados simples.

### Estado `FORWARD`

O robô:

- analisa os setores frontal, frontal esquerdo e frontal direito;
- avança quando o caminho está livre;
- reduz a velocidade ao se aproximar de obstáculos;
- aplica pequenas correções para manter distância das paredes.

### Estado `TURN`

Quando um obstáculo está próximo, o robô:

1. interrompe a velocidade linear;
2. compara o espaço disponível à esquerda e à direita;
3. gira parado para o lado mais livre;
4. volta ao estado `FORWARD` após detectar um caminho seguro.

A decisão é baseada nas mensagens do LiDAR. Não é utilizada uma sequência de movimentos puramente temporizada.

## Medição de cobertura

A posição recebida por `/odom` é convertida em uma célula de uma matriz discreta de `20 × 20`.

Os valores da matriz representam:

```text
0 = região não considerada
1 = região disponível ainda não visitada
2 = região visitada
```

Quando o robô entra em uma nova célula válida, o controlador atualiza e publica no terminal o percentual de cobertura.

Exemplo:

```text
Nova regiao visitada | celula=(8, 12) | cobertura=25.93% (70/270)
```

## Tecnologias utilizadas

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Gazebo Harmonic
- TurtleBot3 Burger
- Python 3
- `rclpy`
- NumPy
- Oracle VirtualBox

## Dependências

O projeto utiliza os seguintes repositórios oficiais:

- `turtlebot3_msgs`
- `turtlebot3`
- `turtlebot3_simulations`

Também utiliza os pacotes de integração entre ROS 2 e Gazebo fornecidos por `ros_gz`.

## Preparação do workspace

Crie o workspace:

```bash
mkdir -p ~/desafio_gazebo_ros2/src
cd ~/desafio_gazebo_ros2/src
```

Clone as dependências para ROS 2 Jazzy:

```bash
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git

git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git

git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

Clone este projeto:

```bash
git clone https://github.com/anickssm/monitoramento-autonomo-gazebo-ros2.git
```

Volte para a raiz do workspace:

```bash
cd ~/desafio_gazebo_ros2
```

Carregue o ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
```

Instale as dependências:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Compile:

```bash
colcon build --symlink-install
```

Carregue o workspace:

```bash
source ~/desafio_gazebo_ros2/install/setup.bash
```

## Configuração para VirtualBox

O Gazebo e o GPU LiDAR apresentaram melhor funcionamento no VirtualBox com:

- controlador gráfico `VMSVGA`;
- 128 MB de memória de vídeo;
- aceleração 3D desativada;
- renderização por software.

O arquivo de launch configura automaticamente:

```bash
LIBGL_ALWAYS_SOFTWARE=1
```

Essa configuração pode não ser necessária em uma instalação nativa do Ubuntu.

## Execução

Após compilar e carregar o workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/desafio_gazebo_ros2/install/setup.bash
```

Inicie toda a solução com:

```bash
ros2 launch turtlebot3_control_ros2 desafio_gazebo.launch.py
```

O launch inicia:

- o cenário `turtlebot3_dqn_stage4`;
- o TurtleBot3 Burger;
- os bridges entre Gazebo e ROS 2;
- o publicador de estado do robô;
- o controlador autônomo;
- a medição de cobertura.

Para encerrar:

```text
Ctrl + C
```

O controlador publica comandos de velocidade zero antes de finalizar.

## Estrutura do projeto

```text
monitoramento-autonomo-gazebo-ros2/
├── config/
│   └── gazebo_gui.config
├── launch/
│   └── desafio_gazebo.launch.py
├── resource/
├── test/
├── turtlebot3_control_ros2/
│   ├── __init__.py
│   ├── autonomous_ctrl.py
│   └── turtlebot_ctrl.py
├── package.xml
├── setup.py
├── setup.cfg
├── LICENSE
├── README.md
└── .gitignore
```

## Código-base e desenvolvimento

O pacote `turtlebot3_control_ros2`, disponibilizado pelo professor Ricardo Grando, foi utilizado como referência inicial.

O nó `autonomous_ctrl.py`, a estratégia autônoma baseada no LiDAR, a integração da odometria, o cálculo de cobertura e o arquivo de launch foram desenvolvidos e adaptados para:

```text
Ubuntu 24.04
ROS 2 Jazzy
Gazebo Harmonic
```

## Referências

- Código-base do professor:  
  https://github.com/ricardoGrando/turtlebot3_control_ros2

- TurtleBot3:  
  https://github.com/ROBOTIS-GIT/turtlebot3

- Mensagens do TurtleBot3:  
  https://github.com/ROBOTIS-GIT/turtlebot3_msgs

- Simulações do TurtleBot3:  
  https://github.com/ROBOTIS-GIT/turtlebot3_simulations

## Licença

Este projeto mantém a licença Apache 2.0 do pacote utilizado como referência.

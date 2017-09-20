# Entrega I - Princípios Computacionais de Robótica

## Objetivo:

O robô pioneer deve se deslocar pelo primeiro andar do prédio CIC/EST. A posição inicial pode ser
dada pelo usuário ou pelo simulador. A posição do robô a cada instante de tempo também é
conhecida (fornecidade pela odometria ou por um sistema de localização).

O usuário deve especificar uma posição qualquer no mapa do prédio, em coordenadas do eixo
global e o robô deve se deslocar para esse ponto, sem tocar nenhum objeto que ele consiga detectar:
paredes, móveis, pessoas, etc.

### Dependências

O Trabalho foi desenvolvido em ROS Kinectic e contêm as dependências listadas a seguir.

* Dependências ROS:
    * roscpp
    * geometry_msgs
    * sensor_msgs
    * p2os_msgs
    * std_msgs
    * tf

* Dependências fora do ROS:
    * [catkin_tools](http://catkin-tools.readthedocs.io/en/latest/advanced/catkin_shell_verbs.html) [[1]]

* Instalação das dependências extras utilizadas:

```
    sudo apt-get install ros-kinectic-p2os-msgs
    sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers
    sudo apt-get install catkin_tools
```

### Configurações do Workspace
Primeiramente, certifique-se estar no diretório principal do workspace.

```
    cd path/to/fcr2017_ws
```


Para compilar o workspace, utilize o comando:

```
    catkin build
```
Em seguida, é necessário atualizar as referências do ambiente.
Execute o comando:

```
    source devel/setup.bash
```


### Execução
Primeiramente, deve-se lançar a simulação.
Esta está contida no pacote fcr2017 e foi configurada pelos monitores.

```
    roslaunch fcr2017 pioneer3at.gazebo.launch
```

Em seguida, deve-se executar o executável navigation, no pacote navigation.

```
    rosrun navigation navigation
```

O pacote será, então executado.

### Entradas e Saídas Testadas

Seu input padrão são coordenadas X e Y de um ponto no espaço.
O Robô é capaz de locomover-se de forma satisfatória ao logo do mapa.
No entanto, existem alguns pontos que não podem ser alcançados.

Portanto, recomenda-se o teste primário com as seguintes coordenadas:

* (X, Y) = (3, 0) - movimento retilíneo;
* (X, Y) = (-1, 5) - controle de trajetória;
* (X, Y) = (12, 12) - desvio de obstáculos (follow wall);

### Algoritimo
Pseudo código simplificado do algorítimo implementado.

```
INICIO
    enquanto ros::ok():
        MostraMenu()
        posicao_alvo <- RecebePonto()
        posicao <- LerOdometria()
        enquanto posicao != posicao_alvo:
            se DetectaObstaculo() = True:
                orientação_normal <- OrientacaoNormal()
                AjustaOrientacao(orientacao_normal)
            se nao:
                orientacao_alvo <- OrientaPonto(posicao_alvo)
                AjustaOrientacao(orientacao_alvo)
            fim se
            MoveFrente()
        fim enquanto
    fim enquanto
FIM
```

### Estrutura de Arquivos

```
navigation/
├── CMakeLists.txt -> cmake build instructions
├── include
│   ├── measurements
│   │   ├── laser_subscriber.h      -> laser class
│   │   ├── odometry_subscriber.h   -> odometry class
│   │   └── ultrasound_subscriber.h -> ultrasound class
│   ├── movement
│   │   ├── kinematics.h -> low level movement class
│   │   ├── movement.h   -> fundamental movements class
│   │   └── pid.h        -> PID implementation class [2]
│   ├── namespaces
│   │   ├── angleOps.h   -> angle operations function
│   │   └── parameters.h -> general parameters
│   └── navigation.h     -> navigation class
├── package.xml     -> package description
├── README.md       -> this file
└── src
    ├── kinematics.cpp          -> low level movement source
    ├── laser_subscriber.cpp    -> laser source
    ├── main.cpp                -> main file
    ├── movement.cpp            -> fundamental movements source
    ├── navigation.cpp          -> navigation source
    ├── odometry_subscriber.cpp -> odometry source
    ├── pid.cpp                 -> PID implementation source
    └── ultrasound_subscriber.cpp -> ultrasound source

```

[1]: http://www.ros.org/news/2016/04/5-reasons-ros-users-will-want-to-try-catkin-tools-beta-2.html
[2]: https://gist.github.com/bradley219/5373998

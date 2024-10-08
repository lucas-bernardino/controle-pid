# Automatização de Testes com um Controlador PID
#### Criação de um sistema automatizado para testes de pastilha de freio.

[Objetivos](#objetivos)

[Dispositivos Utilizados](#dispositivos-utilizados)

[Funcionamento](#funcionamento)

![Sistema](/assets/system.jpeg)

## Objetivos
Esse projeto foi desenvolvido para a iniciação científica para a Universidade Federal de Santa Catarina (UFSC), com o objetivo de automatizar o testes de pastilha de freio em uma motocicleta. O sistema utiliza o Arduino Uno para coordenar o controle e lidar com os sensores de velocidade da moto e temperatura do freio, bem como comandar a válvula de pressão para freiar a motocicleta e o motor de passo para acelerá-la. 

## Dispositivos Utilizados

* Um dos principais parâmetros do controle é a velocidade da moto. Para obter esse dado, foi utilizado um sensor do tipo Hall com um imã localizado na roda traseira da motocicleta. Através do tempo entre a detecção do sensor com esse imã e o diâmetro da roda, é possível obter a velocidade da moto.
* A temperatura do freio deve ser monitorada ao final de cada ciclo, e isso é feito através de um sensor de temperatura industrial localizado próximo à pastilha de freio. A comunicação entre o sensor de temperatura e o Arduino é feita por I2C.
* Para freiar a moto, foi utilizado uma válvula de pressão conectada ao freio da moto. O acionamento dessa válvula pelo Arduino foi feito por meio de um Relé, visto que a valvula industrial necessita de 24 V para operação.
* Acoplado na manopla da motocicleta, há um motor de passo que é responsável pela aceleração da moto. Foi utilizado um Nema-17 com 0.4Nm de torque. Devido ao fato desse motor de passo precisar 1.5A, foi utilizado o driver DRV-8825 para se comunicar com o Nema.

## Funcionamento
1. Inicialmente, deve-se rodar o script em python, responsável por enviar os parâmetros do controlador (Kp, Ki, Kd e setpoint) e salvar os dados de velocidade e temperatura que o Arduino envia via Serial para posteriormente plotar em um gráfico. 
2. Após o setup, a moto será acelerada com o objetivo de atingir o setpoint.
3. Atingido o setpoint, o Arduino irá ativar o freio por alguns segundos enquanto concomitantemente o controlador regula a velocidade da moto para tentar permanecer na velocidade do setpoint.
4. A sequência de frenagens é realizada 10 vezes e ao fim desse ciclo o Arduino interrompe a aceleração da moto e aguarda a temperatura dos freios, informada pelo sensor de temperatura, baixar a 100º C. Após atingir essa temperatura, um novo ciclo se inicia.
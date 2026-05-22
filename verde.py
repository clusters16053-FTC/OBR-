from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction, Color
from pybricks.tools import wait

# =========================
# HUB 🤖
# =========================

hub = PrimeHub()

# =========================
# MOTORES ⚙️
# =========================

motor_esq = Motor(
    Port.A,
    Direction.COUNTERCLOCKWISE
)

motor_dir = Motor(Port.B)

# =========================
# SENSORES 🎨
# =========================

sensor_esq = ColorSensor(Port.C)
sensor_dir = ColorSensor(Port.F)

# =========================
# PIDF 🎯
# =========================

KP = 18
KI = 0
KD = 1.2
KF = 1.9

# =========================
# VELOCIDADE ⚡
# =========================

VELOCIDADE = 50

# desaceleração nas curvas
KV = 1.8

# velocidade mínima
VELOCIDADE_MIN = 35

# =========================
# VARIÁVEIS 🧠
# =========================

erro_anterior = 0
integral = 0

# memória da curva
ultima_curva = 0

# força do verde
ajuste_verde = 0

# =========================
# LOOP PRINCIPAL 🚀
# =========================

while True:

    # =========================
    # LEITURA DAS CORES 🎨
    # =========================

    cor_esq = sensor_esq.color()
    cor_dir = sensor_dir.color()

    # =========================
    # DETECÇÃO DO VERDE 🟩
    # =========================

    # verde esquerda
    if cor_esq == Color.GREEN and cor_dir != Color.GREEN:

        ajuste_verde = -19

    # verde direita
    elif cor_dir == Color.GREEN and cor_esq != Color.GREEN:

        ajuste_verde = 19

    # duplo verde
    elif cor_esq == Color.GREEN and cor_dir == Color.GREEN:

        ajuste_verde = 0

    else:

        ajuste_verde = 0

    # =========================
    # REFLEXÃO DOS SENSORES 💡
    # =========================

    leitura_esq = sensor_esq.reflection()
    leitura_dir = sensor_dir.reflection()

    # =========================
    # ERRO DA LINHA ➕
    # =========================

    erro = leitura_esq - leitura_dir

    # memória da curva 🧠

    if erro > 5:
        ultima_curva = 1

    elif erro < -5:
        ultima_curva = -1

    # verde influencia MENOS o PID
    erro += ajuste_verde

    # =========================
    # PIDF 🎯
    # =========================

    # proporcional
    p = KP * erro

    # integral
    integral += erro

    # anti-windup 🚧
    integral = max(min(integral, 300), -300)

    i = KI * integral

    # derivada
    derivada = erro - erro_anterior
    d = KD * derivada

    # feedforward 🚀

    if erro > 0:
        ff = KF

    elif erro < 0:
        ff = -KF

    else:
        ff = 0

    # cálculo final
    correcao = p + i + d + ff

    # =========================
    # LIMITADOR 🚧
    # =========================

    correcao = max(min(correcao, 90), -90)

    # =========================
    # VELOCIDADE DINÂMICA ⚡
    # =========================

    velocidade_atual = (
        VELOCIDADE
        - (abs(erro) * KV)
    )

    velocidade_atual = max(
        velocidade_atual,
        VELOCIDADE_MIN
    )

    # =========================
    # CONTROLE DIRETO DOS MOTORES ⚙️
    # =========================

    velocidade_esq = velocidade_atual + correcao
    velocidade_dir = velocidade_atual - correcao

    # limite dos motores 🚧

    velocidade_esq = max(
        min(velocidade_esq, 100),
        -100
    )

    velocidade_dir = max(
        min(velocidade_dir, 100),
        -100
    )

    # movimento 🤖

    motor_esq.dc(int(velocidade_esq))
    motor_dir.dc(int(velocidade_dir))

    # salva erro anterior 🧠
    erro_anterior = erro

    # atualização mais rápida ⚡
    wait(5)
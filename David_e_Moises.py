from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# =========================
# HUB 🤖
# =========================

hub = PrimeHub()

# =========================
# MOTORES ⚙️
# =========================

motor_esq = Motor(Port.A, Direction.COUNTERCLOCKWISE)
motor_dir = Motor(Port.B)

# =========================
# SENSORES 🎨
# =========================

sensor_esq = ColorSensor(Port.C)
sensor_dir = ColorSensor(Port.F)

# =========================
# DRIVEBASE 🚗
# =========================

robo = DriveBase(
    motor_esq,
    motor_dir,
    wheel_diameter=62,
    axle_track=150
)

# =========================
# CONFIGURAÇÕES PIDF 🎯
# =========================

KP = 20
KI = 0.00
KD = 0.2
KF = 1

velocidade = 80

# =========================
# VARIÁVEIS 🧠
# =========================

erro_anterior = 0
integral = 0

# =========================
# SEGUIDOR DE LINHA 🚀
# =========================

while True:

    # leitura dos sensores
    leitura_esq = sensor_esq.reflection()
    leitura_dir = sensor_dir.reflection()

    # erro entre sensores
    erro = leitura_esq - leitura_dir

    # integral
    integral += erro

    # anti-windup
    integral = max(min(integral, 1000), -1000)

    # derivada
    derivada = erro - erro_anterior

    # feedforward
    if erro > 0:
        feedforward = KF
    elif erro < 0:
        feedforward = -KF
    else:
        feedforward = 0

    # PIDF
    correcao = (
        (KP * erro) +
        (KI * integral) +
        (KD * derivada) +
        feedforward
    )

    # limita correção
    correcao = max(min(correcao, 150), -150)

    # movimentação
    robo.drive(
        velocidade,
        correcao
    )

    # salva erro
    erro_anterior = erro

    wait(10)
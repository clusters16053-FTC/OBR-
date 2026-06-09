from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait

hub = PrimeHub()

motor_esq = Motor(
    Port.B,
    Direction.COUNTERCLOCKWISE
)

motor_dir = Motor(Port.A)

sensor_ext_esq = ColorSensor(Port.C)
sensor_int_esq = ColorSensor(Port.D)

sensor_int_dir = ColorSensor(Port.E)
sensor_ext_dir = ColorSensor(Port.F)

# =========================
# PIDF
# =========================

KP = 8.2
KI = 0.0
KD = 0.8
KF = 0

# =========================
# VELOCIDADE
# =========================

VELOCIDADE = 40
VELOCIDADE_MIN = 25

KV = 1.0



LIMIAR = 20

erro_anterior = 0
integral = 0

while True:

    s1 = sensor_ext_esq.reflection()
    s2 = sensor_int_esq.reflection()
    s3 = sensor_int_dir.reflection()
    s4 = sensor_ext_dir.reflection()

    

    if (
        s1 > 30 and
        s2 > 30 and
        s3 > 30 and
        s4 > 30
    ):
        motor_esq.dc(VELOCIDADE)
        motor_dir.dc(VELOCIDADE)
        wait(10)
        continue

    # =========================
    # CURVAS FECHADAS
    # =========================

    # Curva forte para esquerda
    if (
        s1 < LIMIAR and
        s2 > LIMIAR and
        s3 > LIMIAR
    ):
        motor_esq.dc(-50)
        motor_dir.dc(70)
        wait(40)
        continue

    # Curva forte para direita
    if (
        s4 < LIMIAR and
        s3 > LIMIAR and
        s2 > LIMIAR
    ):
        motor_esq.dc(70)
        motor_dir.dc(-50)
        wait(40)
        continue

    soma = s1 + s2 + s3 + s4

    if soma > 0:

        erro = (
            (3 * s1)
            + (1 * s2)
            + (-1 * s3)
            + (-3 * s4)
        ) / soma

        erro *= 20

    else:

        erro = erro_anterior

    # =========================
    # PID
    # =========================

    p = KP * erro

    integral += erro

    integral = max(
        min(integral, 200),
        -200
    )

    i = KI * integral

    derivada = erro - erro_anterior

    d = KD * derivada

    if erro > 0:
        ff = KF
    elif erro < 0:
        ff = -KF
    else:
        ff = 0

    correcao = p + i + d + ff

    correcao = max(
        min(correcao, 50),
        -50
    )

    velocidade_atual = (
        VELOCIDADE
        - abs(erro) * KV
    )

    velocidade_atual = max(
        velocidade_atual,
        VELOCIDADE_MIN
    )

    velocidade_esq = velocidade_atual + correcao
    velocidade_dir = velocidade_atual - correcao

    # Impede motor morrer

    if 0 < velocidade_esq < 20:
        velocidade_esq = 20

    if 0 < velocidade_dir < 20:
        velocidade_dir = 20

    velocidade_esq = max(
        min(velocidade_esq, 100),
        -100
    )

    velocidade_dir = max(
        min(velocidade_dir, 100),
        -100
    )

    motor_esq.dc(int(velocidade_esq))
    motor_dir.dc(int(velocidade_dir))

    erro_anterior = erro

    wait(10)
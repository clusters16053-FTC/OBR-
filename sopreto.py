from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait

from desvio_ultra import DesvioUltra
from Verde import Verde

hub = PrimeHub(broadcast_channel=2, observe_channels=[1])

motor_esq = Motor(
    Port.A,
    Direction.COUNTERCLOCKWISE
)

motor_dir = Motor(Port.B)

sensor_ext_esq = ColorSensor(Port.F)
sensor_int_esq = ColorSensor(Port.E)

sensor_int_dir = ColorSensor(Port.D)
sensor_ext_dir = ColorSensor(Port.C)

desvio = DesvioUltra(
    hub,
    motor_esq,
    motor_dir,
    sensor_ext_esq,
    sensor_int_esq,
    sensor_int_dir,
    sensor_ext_dir
)

verde = Verde(
    sensor_ext_esq,
    sensor_int_esq,
    sensor_int_dir,
    sensor_ext_dir,
    hub.imu,
    vel_giro=90,
    tempo_curva=700,
    tempo_180=1200
)

# =========================
# PIDF
# =========================
KP = 7.2
KI = 0.0
KD = 4.8
KF = 0.4

# =========================
# VELOCIDADE
# =========================

VELOCIDADE = 35

VELOCIDADE_RAMPA     = 80
VELOCIDADE_MIN_RAMPA = 70

VELOCIDADE_DESCIDA     = 20
VELOCIDADE_MIN_DESCIDA = 15

VELOCIDADE_MIN = 25

KV = 1.0

LIMIAR = 20

# =========================
# INCLINAÇÃO
# =========================

LIMIAR_INCLINACAO_SUBIDA  = 4  # mesmo de antes
LIMIAR_INCLINACAO_DESCIDA = 4 # detecta descida mais cedo

em_rampa = False

erro_anterior = 0
integral = 0

while True:

    # =========================
    # PRIORIDADE 1
    # ULTRASSÔNICO
    # =========================

    if desvio.verificar():
        wait(10)
        continue

    # =========================
    # PRIORIDADE 2
    # VERDE
    # =========================

    if verde.verificar_e_executar(motor_esq, motor_dir, LIMIAR):
        erro_anterior = 0
        integral = 0
        continue

    # =========================
    # PRIORIDADE 3
    # DETECÇÃO DE INCLINAÇÃO (IMU)
    # =========================

    pitch = hub.imu.tilt()[1]

    if pitch <= -LIMIAR_INCLINACAO_SUBIDA:
        # SUBINDO
        if not em_rampa:
            em_rampa = True
            hub.ble.broadcast(True)
        vel_base = VELOCIDADE_RAMPA
        vel_min  = VELOCIDADE_MIN_RAMPA

    elif pitch >= LIMIAR_INCLINACAO_DESCIDA:
        # DESCENDO
        if not em_rampa:
            em_rampa = True
            hub.ble.broadcast(True)
            # Freia no momento da transição para estabilizar
            motor_esq.brake()
            motor_dir.brake()
            wait(300)
        vel_base = VELOCIDADE_DESCIDA
        vel_min  = VELOCIDADE_MIN_DESCIDA

    else:
        # PLANO
        if em_rampa:
            em_rampa = False
            hub.ble.broadcast(False)
        vel_base = VELOCIDADE
        vel_min  = VELOCIDADE_MIN

    # =========================
    # LEITURA NORMAL DE LINHA
    # =========================

    s1 = sensor_ext_esq.reflection()
    s2 = sensor_int_esq.reflection()
    s3 = sensor_int_dir.reflection()
    s4 = sensor_ext_dir.reflection()

    # =========================
    # TODOS NO PRETO
    # =========================

    if (
        s1 < LIMIAR and
        s2 < LIMIAR and
        s3 < LIMIAR and
        s4 < LIMIAR
    ):
        motor_esq.dc(vel_base)
        motor_dir.dc(vel_base)

        wait(10)
        continue

    # =========================
    # TODOS NO BRANCO
    # =========================

    if (
        s1 > 30 and
        s2 > 30 and
        s3 > 30 and
        s4 > 30
    ):
        motor_esq.dc(vel_base)
        motor_dir.dc(vel_base)

        wait(10)
        continue

    # =========================
    # CURVAS FECHADAS
    # =========================

    if (
        s1 < LIMIAR and
        s2 > LIMIAR and
        s3 > LIMIAR
    ):
        motor_esq.dc(-50)
        motor_dir.dc(70)

        wait(40)
        continue

    if (
        s4 < LIMIAR and
        s3 > LIMIAR and
        s2 > LIMIAR
    ):
        motor_esq.dc(70)
        motor_dir.dc(-50)

        wait(40)
        continue

    # =========================
    # CÁLCULO DE ERRO DA LINHA
    # =========================

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
    # PROCESSAMENTO DO PID
    # =========================

    p = KP * erro

    integral += erro
    integral = max(min(integral, 200), -200)

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

    correcao = max(min(correcao, 140), -140)

    # =========================
    # VELOCIDADE DINÂMICA
    # =========================

    velocidade_atual = (
        vel_base
        - abs(erro) * KV
    )

    velocidade_atual = max(
        velocidade_atual,
        vel_min
    )

    velocidade_esq = velocidade_atual + correcao
    velocidade_dir = velocidade_atual - correcao

    # =========================
    # IMPEDIR MOTOR MORRER
    # =========================

    if 0 < velocidade_esq < 20:
        velocidade_esq = 20

    if 0 < velocidade_dir < 20:
        velocidade_dir = 20

    velocidade_esq = max(min(velocidade_esq, 100), -100)
    velocidade_dir = max(min(velocidade_dir, 100), -100)

    # =========================
    # ENVIO PARA OS MOTORES
    # =========================

    motor_esq.dc(int(velocidade_esq))
    motor_dir.dc(int(velocidade_dir))

    erro_anterior = erro

    wait(10)
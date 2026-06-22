from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait

from desvio_ultra import DesvioUltra
from Verde import Verde

hub = PrimeHub(observe_channels=[1])

motor_esq = Motor(
    Port.B,
    Direction.COUNTERCLOCKWISE
)

motor_dir = Motor(Port.A)

sensor_ext_esq = ColorSensor(Port.C)
sensor_int_esq = ColorSensor(Port.D)

sensor_int_dir = ColorSensor(Port.E)
sensor_ext_dir = ColorSensor(Port.F)

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
    hub.imu
)

# =========================
# PIDF
# =========================
KP = 8.2
KI = 0.0
KD = 1
KF = 0.4

# =========================
# VELOCIDADE
# =========================

VELOCIDADE = 49
VELOCIDADE_MIN = 25

KV = 1.0

LIMIAR = 20

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
    # VERDE (Lógica Nova e Centralizada)
    # =========================

    # A própria classe checa o HSV, faz o debounce e executa a curva bloqueante.
    # Se ela retornar True, significa que executou uma curva, então limpamos o PID.
    if verde.verificar_e_executar(motor_esq, motor_dir, LIMIAR):
        erro_anterior = 0
        integral = 0
        continue

    # =========================
    # LEITURA NORMAL DE LINHA
    # =========================
    
    s1 = sensor_ext_esq.reflection()
    s2 = sensor_int_esq.reflection()
    s3 = sensor_int_dir.reflection()
    s4 = sensor_ext_dir.reflection()

    # =========================
    # TODOS NO BRANCO
    # =========================

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

    correcao = max(min(correcao, 100), -100)

    # =========================
    # VELOCIDADE DINÂMICA
    # =========================

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
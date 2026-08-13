from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait

from desvio_ultra import DesvioUltra
from Verde import Verde
from Resgate import Resgate


# =========================
# HUB
# =========================

hub = PrimeHub(
    broadcast_channel=2,
    observe_channels=[1]
)


# =========================
# MOTORES
# =========================

motor_esq = Motor(
    Port.A,
    Direction.COUNTERCLOCKWISE
)

motor_dir = Motor(
    Port.B
)


# =========================
# SENSORES
# =========================

sensor_ext_esq = ColorSensor(Port.F)
sensor_int_esq = ColorSensor(Port.E)
sensor_int_dir = ColorSensor(Port.D)
sensor_ext_dir = ColorSensor(Port.C)


# =========================
# DESVIO ULTRASSÔNICO
# =========================

desvio = DesvioUltra(
    hub,
    motor_esq,
    motor_dir,
    sensor_ext_esq,
    sensor_int_esq,
    sensor_int_dir,
    sensor_ext_dir
)


# =========================
# VERDE
# =========================

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
# RESGATE
# =========================

resgate = Resgate()


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
VELOCIDADE_MIN = 25

KV = 1.0

LIMIAR = 20


# =========================
# COMPENSAÇÃO MECÂNICA
# =========================

COMPENSACAO_DIREITA = 5


# =========================
# ALINHAMENTO
# =========================

# S2 e S3 precisam estar no branco
# para considerar o robô alinhado.

LIMIAR_ALINHAMENTO = 30


# Velocidade durante o alinhamento.

VELOCIDADE_ALINHAMENTO = 25


# Tempo necessário para entrar
# no alinhamento reforçado.

TEMPO_ALINHAMENTO = 30


# Correção extra.

ALINHAMENTO_EXTRA = 30


# =========================
# IMU
# =========================

# Se a correção do PID for maior que
# este valor, a IMU não interfere.

LIMITE_CORRECAO_IMU = 25


# Quanto a IMU influencia na correção.

GANHO_IMU = 1.5


# Referência angular.

angulo_referencia = hub.imu.heading()


# =========================
# VARIÁVEIS PID
# =========================

erro_anterior = 0
integral = 0

tempo_alinhamento = 0


# =========================
# LOOP PRINCIPAL
# =========================

while True:

    # =========================
    # LEITURA DOS SENSORES
    # =========================

    s1 = sensor_ext_esq.reflection()
    s2 = sensor_int_esq.reflection()
    s3 = sensor_int_dir.reflection()
    s4 = sensor_ext_dir.reflection()


    # =========================
    # PRIORIDADE 1 — RESGATE
    # =========================

    resgate_ativo, resgate_pulso = resgate.verificar(
        s1,
        s2,
        s3,
        s4,
        dt=10
    )

    hub.ble.broadcast(
        (resgate_ativo, resgate_pulso)
    )

    if resgate_ativo:

        resgate.executar_re(
            motor_esq,
            motor_dir
        )

        wait(10)
        continue


    # =========================
    # PRIORIDADE 2 — ULTRASSÔNICO
    # =========================

    if desvio.verificar():

        wait(10)
        continue


    # =========================
    # PRIORIDADE 3 — VERDE
    # =========================

    if verde.verificar_e_executar(
        motor_esq,
        motor_dir,
        LIMIAR
    ):

        erro_anterior = 0
        integral = 0
        tempo_alinhamento = 0

        continue


    # =========================
    # TODOS NO PRETO
    # =========================

    if (
        s1 < LIMIAR
        and
        s2 < LIMIAR
        and
        s3 < LIMIAR
        and
        s4 < LIMIAR
    ):

        motor_esq.dc(
            VELOCIDADE
        )

        motor_dir.dc(
            VELOCIDADE
        )

        tempo_alinhamento = 0

        wait(10)
        continue


    # =========================
    # TODOS NO BRANCO
    # =========================

    if (
        s1 > 30
        and
        s2 > 30
        and
        s3 > 30
        and
        s4 > 30
    ):

        motor_esq.dc(
            VELOCIDADE
        )

        motor_dir.dc(
            VELOCIDADE
        )

        tempo_alinhamento = 0

        wait(10)
        continue


    # =========================
    # CURVA FECHADA PARA ESQUERDA
    # =========================

    if (
        s1 < LIMIAR
        and
        s2 > LIMIAR
        and
        s3 > LIMIAR
    ):

        tempo_alinhamento = 0

        motor_esq.dc(-50)
        motor_dir.dc(70)

        wait(40)
        continue


    # =========================
    # CURVA FECHADA PARA DIREITA
    # =========================

    if (
        s4 < LIMIAR
        and
        s3 > LIMIAR
        and
        s2 > LIMIAR
    ):

        tempo_alinhamento = 0

        motor_esq.dc(70)
        motor_dir.dc(-50)

        wait(40)
        continue


    # =========================
    # CÁLCULO DO ERRO
    # =========================

    soma = s1 + s2 + s3 + s4

    if soma > 0:

        erro = (
            (3 * s1)
            +
            (1 * s2)
            +
            (-1 * s3)
            +
            (-3 * s4)
        ) / soma

        erro *= 20

    else:

        erro = erro_anterior


    # =========================
    # VERIFICA ALINHAMENTO
    # =========================

    alinhado = (
        s2 > LIMIAR_ALINHAMENTO
        and
        s3 > LIMIAR_ALINHAMENTO
    )


    # =========================
    # ATUALIZA REFERÊNCIA DA IMU
    # =========================

    if alinhado:

        # Quando S2 e S3 estão brancos,
        # consideramos que o robô está
        # corretamente alinhado.

        angulo_referencia = hub.imu.heading()


    # =========================
    # CONTROLE DO ALINHAMENTO
    # =========================

    if alinhado:

        tempo_alinhamento = 0

    else:

        tempo_alinhamento += 10


    # =========================
    # PIDF
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


    ff = (
        KF
        if erro > 0
        else (
            -KF
            if erro < 0
            else 0
        )
    )


    correcao = (
        p
        +
        i
        +
        d
        +
        ff
    )


    # =========================
    # LIMITA PID
    # =========================

    correcao = max(
        min(correcao, 140),
        -140
    )


    # Guarda a correção do PID
    # antes da IMU.

    correcao_pid = correcao


    # =========================
    # IMU — CORREÇÃO FINA
    # =========================

    # A IMU só interfere quando:
    #
    # 1. Não estamos em uma curva fechada
    # 2. A correção do PID é pequena
    # 3. O robô não está perfeitamente alinhado

    if (
        not alinhado
        and
        abs(correcao_pid) <= LIMITE_CORRECAO_IMU
    ):

        angulo_atual = hub.imu.heading()


        # Calcula diferença angular.

        erro_imu = (
            angulo_referencia
            - angulo_atual
        )


        # Corrige passagem de 359° para 0°.

        if erro_imu > 180:

            erro_imu -= 360

        elif erro_imu < -180:

            erro_imu += 360


        # Correção da IMU.

        correcao_imu = (
            erro_imu
            *
            GANHO_IMU
        )


        # Limita a influência da IMU.

        correcao_imu = max(
            min(correcao_imu, 15),
            -15
        )


        # Soma IMU + PID.

        correcao += correcao_imu


    # =========================
    # VELOCIDADE DINÂMICA
    # =========================

    velocidade_atual = max(
        VELOCIDADE
        -
        abs(erro) * KV,
        VELOCIDADE_MIN
    )


    # =========================
    # ALINHAMENTO REFORÇADO
    # =========================

    if (
        not alinhado
        and
        tempo_alinhamento >= TEMPO_ALINHAMENTO
    ):

        velocidade_atual = min(
            velocidade_atual,
            VELOCIDADE_ALINHAMENTO
        )


        # Correção extra.

        if erro > 0:

            correcao += ALINHAMENTO_EXTRA

        elif erro < 0:

            correcao -= ALINHAMENTO_EXTRA


    # =========================
    # LIMITA CORREÇÃO FINAL
    # =========================

    correcao = max(
        min(correcao, 140),
        -140
    )


    # =========================
    # VELOCIDADES DOS MOTORES
    # =========================

    velocidade_esq = (
        velocidade_atual
        +
        correcao
    )

    velocidade_dir = (
        velocidade_atual
        -
        correcao
        +
        COMPENSACAO_DIREITA
    )


    # =========================
    # LIMITES MÍNIMOS
    # =========================

    if 0 < velocidade_esq < 20:

        velocidade_esq = 20


    if 0 < velocidade_dir < 20:

        velocidade_dir = 20


    # =========================
    # LIMITES MÁXIMOS
    # =========================

    velocidade_esq = max(
        min(velocidade_esq, 100),
        -100
    )

    velocidade_dir = max(
        min(velocidade_dir, 100),
        -100
    )


    # =========================
    # ENVIA PARA OS MOTORES
    # =========================

    motor_esq.dc(
        int(velocidade_esq)
    )

    motor_dir.dc(
        int(velocidade_dir)
    )


    # =========================
    # ATUALIZA PID
    # =========================

    erro_anterior = erro


    wait(10)
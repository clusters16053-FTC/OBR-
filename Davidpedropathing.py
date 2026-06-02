from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction, Color
from pybricks.tools import wait



hub = PrimeHub()



motor_esq = Motor(
    Port.A,
    Direction.COUNTERCLOCKWISE
)

motor_dir = Motor(Port.B)



sensor_ext_esq = ColorSensor(Port.C)
sensor_int_esq = ColorSensor(Port.D)

sensor_int_dir = ColorSensor(Port.E)
sensor_ext_dir = ColorSensor(Port.F)



KP = 18
KI = 0
KD = 1.2
KF = 1.9



VELOCIDADE = 50

KV = 1.8

VELOCIDADE_MIN = 35



LIMIAR = 10



erro_anterior = 0
integral = 0

ultima_curva = 0

ajuste_verde = 0



while True:

    

    cor_esq = sensor_int_esq.color()
    cor_dir = sensor_int_dir.color()

    if cor_esq == Color.GREEN and cor_dir != Color.GREEN:

        ajuste_verde = -19

    elif cor_dir == Color.GREEN and cor_esq != Color.GREEN:

        ajuste_verde = 19

    elif cor_esq == Color.GREEN and cor_dir == Color.GREEN:

        ajuste_verde = 0

    else:

        ajuste_verde = 0

    

    s1 = sensor_ext_esq.reflection()
    s2 = sensor_int_esq.reflection()
    s3 = sensor_int_dir.reflection()
    s4 = sensor_ext_dir.reflection()

    

    if (
        s1 > LIMIAR and
        s2 > LIMIAR and
        s3 > LIMIAR and
        s4 > LIMIAR
    ):

        erro = erro_anterior

    else:

        soma = s1 + s2 + s3 + s4

        if soma > 0:

            erro = (
                (-3 * s1)
                + (-1 * s2)
                + (1 * s3)
                + (3 * s4)
            ) / soma

            erro *= 25

        else:

            erro = erro_anterior

    

    if erro > 5:

        ultima_curva = 1

    elif erro < -5:

        ultima_curva = -1

    

    erro += ajuste_verde

    

    p = KP * erro

    integral += erro

    integral = max(
        min(integral, 300),
        -300
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
        min(correcao, 90),
        -90
    )

    

    velocidade_atual = (
        VELOCIDADE
        - (abs(erro) * KV)
    )

    velocidade_atual = max(
        velocidade_atual,
        VELOCIDADE_MIN
    )

    

    velocidade_esq = (
        velocidade_atual
        + correcao
    )

    velocidade_dir = (
        velocidade_atual
        - correcao
    )

    velocidade_esq = max(
        min(velocidade_esq, 100),
        -100
    )

    velocidade_dir = max(
        min(velocidade_dir, 100),
        -100
    )

    motor_esq.dc(
        int(velocidade_esq)
    )

    motor_dir.dc(
        int(velocidade_dir)
    )

    erro_anterior = erro

    wait(5)
from pybricks.tools import wait


class DesvioUltra:

    def __init__(
        self,
        hub,
        motor_esq,
        motor_dir,
        sensor_ext_esq,
        sensor_int_esq,
        sensor_int_dir,
        sensor_ext_dir
    ):
                
        self.hub = hub

        self.motor_esq = motor_esq
        self.motor_dir = motor_dir

        self.sensor_ext_esq = sensor_ext_esq
        self.sensor_int_esq = sensor_int_esq
        self.sensor_int_dir = sensor_int_dir
        self.sensor_ext_dir = sensor_ext_dir

        self.desviando = False
        self.obstaculo_processado = False
 
    def desviar(self):

        self.desviando = True

        wait(100)
        # Para

        self.motor_esq.dc(0)
        self.motor_dir.dc(0)

        wait(1200)

        # 90° direita

        self.motor_esq.reset_angle(0)

        self.motor_esq.dc(60)
        self.motor_dir.dc(-60)

        while abs(self.motor_esq.angle()) < 3000:
            wait(1)

        # Anda reto para sair da frente do obstáculo

        self.motor_esq.reset_angle(0)

        self.motor_esq.dc(50)
        self.motor_dir.dc(50)

        while abs(self.motor_esq.angle()) < 1000:
            wait(1)

        # 90° esquerda

        self.motor_esq.reset_angle(0)

        self.motor_esq.dc(-60)
        self.motor_dir.dc(60)

        while abs(self.motor_esq.angle()) < 300:
            wait(1)

        # Procura a linha

        while True:

            s1 = self.sensor_ext_esq.reflection()
            s2 = self.sensor_int_esq.reflection()
            s3 = self.sensor_int_dir.reflection()
            s4 = self.sensor_ext_dir.reflection()

            if (
                s1 < 20 or
                s2 < 20 or
                s3 < 20 or
                s4 < 20
            ):
                break

            self.motor_esq.dc(40)
            self.motor_dir.dc(40)

            wait(10)

        self.motor_esq.dc(0)
        self.motor_dir.dc(0)

        wait(100)

        self.desviando = False

    def verificar(self):

        estado = self.hub.ble.observe(1)

        if estado is None:
            return False

        # Liberar para próximo obstáculo
        if estado == 0:
            self.obstaculo_processado = False
            return False

        # Executa apenas uma vez
        if (
            estado == 1
            and not self.desviando
            and not self.obstaculo_processado
        ):

            self.obstaculo_processado = True

            self.desviar()

            return True

        return self.desviando
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

        print("INICIANDO DESVIO")

        self.desviando = True

        # Para
        self.motor_esq.dc(0)
        self.motor_dir.dc(0)

        wait(200)

        # Ré
        print("RE")

        self.motor_esq.reset_angle(0)

        self.motor_esq.dc(-50)
        self.motor_dir.dc(-50)

        while abs(self.motor_esq.angle()) < 300:
            wait(1)

        print("FIM RE")

        # Primeira metade da parábola
        print("CURVA 1")

        self.motor_esq.reset_angle(0)

        self.motor_esq.dc(15)
        self.motor_dir.dc(85)

        while abs(self.motor_esq.angle()) < 150:
            wait(1)

        print("FIM CURVA 1")

        # Curva 2A
        print("CURVA 2A")

        self.motor_esq.reset_angle(0)

        self.motor_esq.dc(85)
        self.motor_dir.dc(15)

        while abs(self.motor_esq.angle()) < 700:
            wait(1)

        print("FIM CURVA 2A")

        # Curva 2B - procurando linha
        print("CURVA 2B - PROCURANDO LINHA")

        self.motor_esq.reset_angle(0)

        self.motor_esq.dc(80)
        self.motor_dir.dc(15)

        while abs(self.motor_esq.angle()) < 600:

            s2 = self.sensor_int_esq.reflection()
            s3 = self.sensor_int_dir.reflection()
            s4 = self.sensor_ext_dir.reflection()

            if (
                s2 < 20 or
                s3 < 20 or
                s4 < 20
            ):

                print("LINHA ENCONTRADA")

                self.motor_esq.dc(0)
                self.motor_dir.dc(0)

                wait(100)

                self.desviando = False

                return

            wait(1)

        print("FIM CURVA 2B")

        # Para
        self.motor_esq.dc(0)
        self.motor_dir.dc(0)

        wait(100)

        self.desviando = False

        print("DESVIO FINALIZADO")

    def verificar(self):

        estado = self.hub.ble.observe(1)

        print("BLE:", estado)

        if self.desviando:
            return True

        if estado is None:
            return False

        if estado == 0:

            if self.obstaculo_processado:
                print("SISTEMA REARMADO")

            self.obstaculo_processado = False
            return False

        if (
            estado == 1
            and not self.obstaculo_processado
        ):

            print("OBSTACULO DETECTADO")

            self.obstaculo_processado = True

            self.desviar()

            return True

        return False
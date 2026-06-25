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

    def _mover_sincrono(self, vel_esq, vel_dir, angulo_alvo, parar_se=None):
        self.motor_esq.reset_angle(0)
        self.motor_dir.reset_angle(0)

        self.motor_esq.dc(vel_esq)
        self.motor_dir.dc(vel_dir)

        while abs(self.motor_esq.angle()) < angulo_alvo:
            if parar_se and parar_se():
                break
            wait(1)

        self.motor_esq.dc(0)
        self.motor_dir.dc(0)

        return bool(parar_se and parar_se())

    def _linha_detectada(self):
        return (
            self.sensor_int_esq.reflection() < 20 or
            self.sensor_int_dir.reflection() < 20 or
            self.sensor_ext_dir.reflection() < 20
        )

    def desviar(self):
        print("INICIANDO DESVIO")
        self.desviando = True

        self.motor_esq.dc(0)
        self.motor_dir.dc(0)
        wait(200)

        print("RE")
        self._mover_sincrono(-50, -50, 370)
        print("FIM RE")

        print("CURVA 1")
        self._mover_sincrono(15, 85, 230)
        print("FIM CURVA 1")

        print("CURVA 2A")
        self._mover_sincrono(85, 15, 880)
        print("FIM CURVA 2A")

        print("CURVA 2B - PROCURANDO LINHA")
        achou = self._mover_sincrono(85, 15, 750, parar_se=self._linha_detectada)

        if achou:
            print("LINHA ENCONTRADA")
            self.desviando = False
            return

        print("FIM CURVA 2B")
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

        if estado == 1 and not self.obstaculo_processado:
            print("OBSTACULO DETECTADO")
            self.obstaculo_processado = True
            self.desviar()
            return True

        return False
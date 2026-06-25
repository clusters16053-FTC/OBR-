from pybricks.tools import wait, StopWatch


class Verde:
    def __init__(
        self,
        sensor_ext_esq,
        sensor_int_esq,
        sensor_int_dir,
        sensor_ext_dir,
        imu,
        vel_giro=90,
        tempo_curva=700,
        tempo_180=1200
    ):
        self.sensor_ext_esq = sensor_ext_esq
        self.sensor_int_esq = sensor_int_esq
        self.sensor_int_dir = sensor_int_dir
        self.sensor_ext_dir = sensor_ext_dir
        self.imu = imu

        self.vel_giro = vel_giro
        self.tempo_curva = tempo_curva
        self.tempo_180 = tempo_180

        self.cronometro = StopWatch()
        self.tempo_bloqueio = 1000

    def eh_verde(self, sensor):
        if self.cronometro.time() < self.tempo_bloqueio:
            return False

        hsv = sensor.hsv()
        return (
            120 <= hsv.h <= 190 and
            hsv.s >= 50 and
            hsv.v >= 10
        )

    def _girar_por_tempo(self, motor_esq, motor_dir, vel, tempo_ms):
        motor_esq.dc(vel)
        motor_dir.dc(-vel)
        wait(tempo_ms)
        motor_esq.brake()
        motor_dir.brake()

    def verificar_e_executar(self, motor_esq, motor_dir, limiar):
        verde_esq = self.eh_verde(self.sensor_ext_esq) or self.eh_verde(self.sensor_int_esq)
        verde_dir = self.eh_verde(self.sensor_int_dir) or self.eh_verde(self.sensor_ext_dir)

        if not verde_esq and not verde_dir:
            return False

        # Janela de 100ms para o outro lado também confirmar verde
        espera = StopWatch()
        while espera.time() < 100:
            if not verde_esq:
                verde_esq = self.eh_verde(self.sensor_ext_esq) or self.eh_verde(self.sensor_int_esq)
            if not verde_dir:
                verde_dir = self.eh_verde(self.sensor_int_dir) or self.eh_verde(self.sensor_ext_dir)
            if verde_esq and verde_dir:
                break
            wait(5)

        self.cronometro.reset()

        if verde_esq and verde_dir:
            print("VERDE DOS 2 LADOS - girando 180 graus")
            angulo_inicial = self.imu.heading()

            # Gira 150 graus pré-programado pelo IMU
            print("GIRANDO 150 graus pre-programado...")
            while True:
                motor_esq.dc(self.vel_giro)
                motor_dir.dc(-self.vel_giro)
                girado = abs(self.imu.heading() - angulo_inicial)
                if girado > 180:
                    girado = 360 - girado
                if girado >= 150:
                    break
                wait(5)

            # Continua girando até achar a linha
            print("BUSCANDO LINHA - girando 180...")
            while True:
                motor_esq.dc(self.vel_giro)
                motor_dir.dc(-self.vel_giro)
                if (
                    self.sensor_ext_esq.reflection() < limiar or
                    self.sensor_int_esq.reflection() < limiar or
                    self.sensor_int_dir.reflection() < limiar or
                    self.sensor_ext_dir.reflection() < limiar
                ):
                    print("LINHA ENCONTRADA - 180")
                    break
                wait(5)

            motor_esq.brake()
            motor_dir.brake()
            wait(100)
            return True

        if verde_esq and not verde_dir:
            print("VERDE ESQUERDA - iniciando manobra")
            motor_esq.dc(80)
            motor_dir.dc(80)
            wait(300)
            motor_esq.brake()
            motor_dir.brake()
            wait(50)

            self._girar_por_tempo(motor_esq, motor_dir, -self.vel_giro, self.tempo_curva)
            wait(200)

            print("BUSCANDO LINHA - girando esquerda...")
            while True:
                motor_esq.dc(-55)
                motor_dir.dc(55)
                if (
                    self.sensor_ext_esq.reflection() < limiar or
                    self.sensor_int_esq.reflection() < limiar or
                    self.sensor_int_dir.reflection() < limiar
                ):
                    print("LINHA ENCONTRADA - esquerda")
                    break
                wait(5)

            motor_esq.brake()
            motor_dir.brake()
            wait(100)

        elif verde_dir and not verde_esq:
            print("VERDE DIREITA - iniciando manobra")
            motor_esq.dc(80)
            motor_dir.dc(80)
            wait(300)
            motor_esq.brake()
            motor_dir.brake()
            wait(50)

            self._girar_por_tempo(motor_esq, motor_dir, self.vel_giro, self.tempo_curva)
            wait(200)

            print("BUSCANDO LINHA - girando direita...")
            while True:
                motor_esq.dc(55)
                motor_dir.dc(-55)
                if (
                    self.sensor_int_esq.reflection() < limiar or
                    self.sensor_int_dir.reflection() < limiar or
                    self.sensor_ext_dir.reflection() < limiar
                ):
                    print("LINHA ENCONTRADA - direita")
                    break
                wait(5)

            motor_esq.brake()
            motor_dir.brake()
            wait(100)

        return True
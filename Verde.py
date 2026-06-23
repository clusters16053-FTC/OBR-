from pybricks.tools import wait, StopWatch


class Verde:
    def __init__(
        self,
        sensor_ext_esq,
        sensor_int_esq,
        sensor_int_dir,
        sensor_ext_dir,
        imu  # Necessário para guinada (yaw)
    ):
        self.sensor_ext_esq = sensor_ext_esq
        self.sensor_int_esq = sensor_int_esq
        self.sensor_int_dir = sensor_int_dir
        self.sensor_ext_dir = sensor_ext_dir
        self.imu = imu

        self.cronometro = StopWatch()
        self.tempo_bloqueio = 3000

    def eh_verde(self, sensor):
        if self.cronometro.time() < self.tempo_bloqueio:
            return False

        hsv = sensor.hsv()
        return (
            120 <= hsv.h <= 190 and
            hsv.s >= 50 and
            hsv.v >= 10
        )

    def _angulo_atual(self):
        """Retorna o ângulo de guinada (yaw) atual do IMU em graus."""
        return self.imu.heading()

    def _girar_ate_angulo(self, motor_esq, motor_dir, angulo_alvo, vel=35):
        """
        Gira o robô até atingir o ângulo alvo (absoluto) usando IMU.
        vel > 0 = vira direita, vel < 0 = vira esquerda.
        """
        while True:
            atual = self._angulo_atual()
            diff = angulo_alvo - atual

            # Normaliza diff para [-180, 180]
            while diff > 180:
                diff -= 360
            while diff < -180:
                diff += 360

            if abs(diff) <= 15:  # Tolerância de 2 graus
                break

            if diff > 0:
                motor_esq.dc(vel)
                motor_dir.dc(-vel)
            else:
                motor_esq.dc(-vel)
                motor_dir.dc(vel)

            wait(5)

        motor_esq.hold()
        motor_dir.hold()

    def verificar_e_executar(self, motor_esq, motor_dir, limiar):
        verde_esq = self.eh_verde(self.sensor_ext_esq) or self.eh_verde(self.sensor_int_esq)
        verde_dir = self.eh_verde(self.sensor_int_dir) or self.eh_verde(self.sensor_ext_dir)

        if not verde_esq and not verde_dir:
            return False

        self.cronometro.reset()

        # ====================================================
        # VERDE DOS 2 LADOS - GIRA 180 GRAUS
        # ====================================================
        if verde_esq and verde_dir:
            print("VERDE DOS 2 LADOS - girando 180 graus")

            angulo_inicial = self._angulo_atual()
            angulo_alvo_180 = angulo_inicial + 180

            self._girar_ate_angulo(motor_esq, motor_dir, angulo_alvo_180, vel=50)
            wait(100)

            return True

        # ====================================================
        # CURVA ESQUERDA
        # ====================================================
        if verde_esq and not verde_dir:
            print("VERDE ESQUERDA - iniciando manobra")

            # 1) Anda um pouco para frente (sem PID)
            motor_esq.dc(80)
            motor_dir.dc(80)
            wait(300)  # Ajuste conforme necessidade
            motor_esq.hold()
            motor_dir.hold()
            wait(50)

            # 2) Gira ~70 graus para a ESQUERDA usando IMU
            angulo_inicial = self._angulo_atual()
            angulo_alvo_70 = angulo_inicial - 45  # Negativo = esquerda

            self._girar_ate_angulo(motor_esq, motor_dir, angulo_alvo_70, vel=80)
            wait(100)

            # 3) Continua girando para a esquerda procurando linha com os 3 sensores
            #    (sensor_ext_dir ignorado pois foi o lado que detectou verde)
            print("BUSCANDO LINHA - girando esquerda...")

            while True:
                motor_esq.dc(-35)
                motor_dir.dc(35)

                # Detecta linha nos 3 sensores ativos (ignora ext_dir)
                if (
                    self.sensor_ext_esq.reflection() < limiar or
                    self.sensor_int_esq.reflection() < limiar or
                    self.sensor_int_dir.reflection() < limiar
                ):
                    print("LINHA ENCONTRADA - esquerda")
                    break

                wait(5)

            motor_esq.hold()
            motor_dir.hold()
            wait(100)

        # ====================================================
        # CURVA DIREITA
        # ====================================================
        elif verde_dir and not verde_esq:
            print("VERDE DIREITA - iniciando manobra")

            # 1) Anda um pouco para frente (sem PID)
            motor_esq.dc(80)
            motor_dir.dc(80)
            wait(300)  # Ajuste conforme necessidade
            motor_esq.hold()
            motor_dir.hold()
            wait(50)

            # 2) Gira ~70 graus para a DIREITA usando IMU
            angulo_inicial = self._angulo_atual()
            angulo_alvo_70 = angulo_inicial + 45 # Positivo = direita

            self._girar_ate_angulo(motor_esq, motor_dir, angulo_alvo_70, vel=80)
            wait(100)

            # 3) Continua girando para a direita procurando linha com os 3 sensores
            #    (sensor_ext_esq ignorado pois foi o lado que detectou verde)
            print("BUSCANDO LINHA - girando direita...")

            while True:
                motor_esq.dc(35)
                motor_dir.dc(-35)

                # Detecta linha nos 3 sensores ativos (ignora ext_esq)
                if (
                    self.sensor_int_esq.reflection() < limiar or
                    self.sensor_int_dir.reflection() < limiar or
                    self.sensor_ext_dir.reflection() < limiar
                ):
                    print("LINHA ENCONTRADA - direita")
                    break

                wait(5)

            motor_esq.hold()
            motor_dir.hold()
            wait(100)

        return True
from pybricks.parameters import Color

class Verde:

    def __init__(
        self,
        sensor_ext_esq,
        sensor_int_esq,
        sensor_int_dir,
        sensor_ext_dir
    ):

        self.sensor_ext_esq = sensor_ext_esq
        self.sensor_int_esq = sensor_int_esq
        self.sensor_int_dir = sensor_int_dir
        self.sensor_ext_dir = sensor_ext_dir

        self.VERDE_MIN = 12
        self.VERDE_MAX = 19

        self.cont_esq = 0
        self.cont_dir = 0

        self.ativo = False

    def eh_verde(self, valor):

        if valor < 10:
            return False

        return self.VERDE_MIN <= valor <= self.VERDE_MAX

    def verificar(self):

        s1 = self.sensor_ext_esq.reflection()
        s2 = self.sensor_int_esq.reflection()
        s3 = self.sensor_int_dir.reflection()
        s4 = self.sensor_ext_dir.reflection()

        # Tudo em uma linha
        print("EE:", s1, "| IE:", s2, "| ID:", s3, "| ED:", s4)

        verde_esq = (
            self.eh_verde(s1) or self.eh_verde(s2)
        )

        verde_dir = (
            self.eh_verde(s3) or self.eh_verde(s4)
        )

        if verde_esq:
            self.cont_esq += 1
        else:
            self.cont_esq = 0

        if verde_dir:
            self.cont_dir += 1
        else:
            self.cont_dir = 0

        if self.cont_esq >= 2 and not self.cont_dir:
            print("VERDE ESQUERDA")
            self.cont_esq = 0
            return "esquerda"

        if self.cont_dir >= 2 and not self.cont_esq:
            print("VERDE DIREITA")
            self.cont_dir = 0
            return "direita"

        if self.cont_esq >= 2 and self.cont_dir >= 2:
            print("DUPLO VERDE")
            self.cont_esq = 0
            self.cont_dir = 0
            return "retorno"

        return None
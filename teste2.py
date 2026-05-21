from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

# --- CONFIGURAÇÃO DE HARDWARE ---
hub = PrimeHub()
sensor_esq = ColorSensor(Port.C)
sensor_dir = ColorSensor(Port.F)

motor_esq = Motor(Port.A, Direction.COUNTERCLOCKWISE)
motor_dir = Motor(Port.B)

# Base motriz (Rodas 62.4mm, Eixo 150mm)
robo = DriveBase(motor_esq, motor_dir, wheel_diameter=62, axle_track=150)

# --- CONSTANTES PIDF ---
VELOCIDADE = 100    
KP = 4.5
KI = 0.01          
KD = 0.1            
KF = 0.35

BRANCO_TOTAL = 75   

# --- VARIÁVEIS DE ESTADO ---
erro_anterior = 0
integral = 0
memoria_90 = 0      # 0 = Reto | 1 = Curva Direita | -1 = Curva Esquerda
cronometro = StopWatch()

def main():
    global erro_anterior, integral, memoria_90

    hub.imu.reset_heading(0)

    while True:
        dt = max(cronometro.time() / 1000, 0.001) 
        cronometro.reset()

        ref_esq = sensor_esq.reflection()
        ref_dir = sensor_dir.reflection()
        
        erro = ref_esq - ref_dir

        # --- 🧠 MEMÓRIA DE CURVA FECHADA (90 GRAUS) ---
        # Analisamos o erro antes de entrar no Gap para saber se era uma curva
        if erro > 35: 
            memoria_90 = 1    # Erro positivo alto: a linha quebrou para a direita
        elif erro < -35:
            memoria_90 = -1   # Erro negativo alto: a linha quebrou para a esquerda
        elif abs(erro) < 15:
            memoria_90 = 0    # Erro pequeno: o robô está numa reta, limpa a memória

        # --- 💨 LÓGICA DO GAP OU RECUPERAÇÃO DE 90 GRAUS ---
        if ref_esq >= BRANCO_TOTAL and ref_dir >= BRANCO_TOTAL:
            
            if memoria_90 == 1:
                # Estava curvando pra direita e perdeu a linha: Gira no próprio eixo para a direita!
                robo.drive(0, 250) 
                wait(5)
                continue
                
            elif memoria_90 == -1:
                # Estava curvando pra esquerda e perdeu a linha: Gira no próprio eixo para a esquerda!
                robo.drive(0, -250)
                wait(5)
                continue
                
            else:
                # Estava reto (memoria_90 == 0). É um GAP verdadeiro. Vai reto com giroscópio!
                angulo_alvo = hub.imu.heading()
                while sensor_esq.reflection() >= BRANCO_TOTAL and sensor_dir.reflection() >= BRANCO_TOTAL:
                    erro_gyro = hub.imu.heading() - angulo_alvo
                    robo.drive(VELOCIDADE, -erro_gyro * 4)
                    wait(5)
                
                # Achou a linha após o gap, reseta o PID
                erro_anterior = 0
                integral = 0
                continue 

        # --- 🏎️ SEGUIDOR DE LINHA PIDF ---
        
        p_out = KP * erro
        
        integral += erro * dt
        integral = max(min(integral, 50), -50)
        i_out = KI * integral
        
        derivada = (erro - erro_anterior) / dt
        d_out = KD * derivada

        ff_out = 0
        if abs(erro) > 5: 
            ff_out = KF * VELOCIDADE if erro > 0 else -KF * VELOCIDADE

        direcao = p_out + i_out + d_out + ff_out

        vel_final = max(40, VELOCIDADE - abs(erro) * 1.5)

        robo.drive(vel_final, direcao)

        erro_anterior = erro
        wait(5)

main()
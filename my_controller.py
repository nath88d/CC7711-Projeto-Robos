from controller import Robot, Supervisor
import random
import math

TIMESTEP = 128
MAX_SPEED = 6.28
MIN_SPEED = 0.8
TURN_SPEED = 2.0

def delay(contagem):
    for _ in range(contagem):
        pass

class CaixaSupervisor(Supervisor):

    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())
        self.caixas = []
        self.ultimas_posicoes = []

        index = 1
        while True:
            if index < 10:
                nome = f"CAIXA0{index}"
            else:
                nome = f"CAIXA{index}"

            caixa_node = self.getFromDef(nome)
            if caixa_node is None:
                break
            self.caixas.append(caixa_node)
            translation_field = caixa_node.getField("translation")
            self.ultimas_posicoes.append(translation_field.getSFVec3f())
            index += 1

        print(f"Total de {len(self.caixas)} caixas monitoradas.")

    def monitorar_caixa(self):
        for i, caixa in enumerate(self.caixas):
            pos = caixa.getField("translation").getSFVec3f()
            ultima = self.ultimas_posicoes[i]
            if abs(pos[0] - ultima[0]) >= 0.0005 or abs(pos[2] - ultima[2]) >= 0.0005:
                return True
            print("Diferenca: x %f z %f" % (pos[0] - ultima[0], pos[2] != ultima[2]))

    def atualizar_posicao(self):
        for i, caixa in enumerate(self.caixas):
            self.ultimas_posicoes[i] = caixa.getField("translation").getSFVec3f()

def run_robot(robot):
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    leds = [robot.getDevice(f'led{i}') for i in range(8)]

    list_ps = []
    for ind in [0, 1, 2, 5, 6, 7]:
        sensor_name = 'ps' + str(ind)
        list_ps.append(robot.getDevice(sensor_name))
        list_ps[-1].enable(TIMESTEP)

    supervisor = CaixaSupervisor()
    alerta_ativado = False
    contador_led = 0

    while robot.step(TIMESTEP) != -1 and supervisor.step(TIMESTEP) != -1:
        if not alerta_ativado:
            alerta_ativado = supervisor.monitorar_caixa()
            if not alerta_ativado:
                supervisor.atualizar_posicao()

        if alerta_ativado:
            left_motor.setVelocity(MAX_SPEED)
            right_motor.setVelocity(-MAX_SPEED)
            cor_led = 0xFF0000 if contador_led % 2 == 0 else 0x000000
            for led in leds:
                led.set(cor_led)
            contador_led += 1
            continue

        sensor_values = [ps.getValue() for ps in list_ps]

        left_speed = MAX_SPEED
        right_speed = MAX_SPEED

        left_speed, right_speed = navigate(sensor_values, left_speed, right_speed)
        left_speed, right_speed = apply_random_movement(left_speed, right_speed)

        if is_stuck(sensor_values):
            print("Robô preso. Recuando.")
            right_speed = -MAX_SPEED * random.choice([-1, 0.5, 1])
            left_speed = -MAX_SPEED * random.choice([-1, 0.5, 1])

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

def navigate(sensor_values, left_speed, right_speed):
    OBSTACLE_THRESHOLD = 50
    PROPORTIONAL_GAIN = 0.7
    front_sensors = sensor_values[0:3]
    side_sensors = sensor_values[3:6]

    if any(val > OBSTACLE_THRESHOLD for val in front_sensors):
        max_front_sensor = max(front_sensors)
        left_speed -= PROPORTIONAL_GAIN * max_front_sensor
        right_speed += PROPORTIONAL_GAIN * max_front_sensor

    if any(val > OBSTACLE_THRESHOLD for val in side_sensors):
        max_side_sensor = max(side_sensors)
        left_speed += PROPORTIONAL_GAIN * max_side_sensor
        right_speed -= PROPORTIONAL_GAIN * max_side_sensor

    left_speed = max(min(left_speed, MAX_SPEED), MIN_SPEED)
    right_speed = max(min(right_speed, MAX_SPEED), MIN_SPEED)
    return left_speed, right_speed

def apply_random_movement(left_speed, right_speed):
    if random.random() < 0.3:
        random_turn = random.choice([-0.3, 0.3])
        left_speed += random_turn * TURN_SPEED
        right_speed -= random_turn * TURN_SPEED

    left_speed = max(min(left_speed, MAX_SPEED), MIN_SPEED)
    right_speed = max(min(right_speed, MAX_SPEED), MIN_SPEED)
    return left_speed, right_speed

def is_stuck(sensor_values):
    OBSTACLE_THRESHOLD = 70
    front_sensors = sensor_values[0:3]
    side_sensors = sensor_values[3:6]
    return all(val > OBSTACLE_THRESHOLD for val in front_sensors) and any(val > OBSTACLE_THRESHOLD for val in side_sensors)

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)

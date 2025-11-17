import math

# TODO: Replace with simulation from https://github.com/SCRT-Capstone-2025-26/SCRT_Rocket_SIM
class SimpleSim():
    def __init__(self, velocity, drag, servo_drag, g=9.81):
        self.height = 0
        self.velocity = velocity
        self.drag = drag
        self.servo_drag = servo_drag
        self.g = g

        self.time = 0
        self.history = [(0, (self.velocity, self.height, None))]


    def step(self, time, servo):
        drag = self.drag + (servo * self.servo_drag)
        c1 = -self.velocity - (self.g / drag)

        exp_term = c1 * math.exp(-drag * time)
        g_term = -self.g / drag

        self.velocity = g_term - exp_term
        self.height += (exp_term / drag) + (g_term * time) - (c1 / drag)

        self.time += time
        self.history.append((self.time, (self.velocity, self.height, servo)))

from . import test_launch as tl

import bayes_opt as opt
import itertools as it

def sample(p, i, d, velocity_r=[800, 1000, 1200], drag_r=[0.3, 0.5, 0.7], servo_drag_r=[2, 3, 4], graph=False):
    max_value = 0
    for velocity, drag, servo_drag in it.product(velocity_r, drag_r, servo_drag_r):
        if graph:
            print(velocity, drag, servo_drag)
        value = abs(tl.try_flight(velocity, drag, servo_drag, p, i, d, graph) - 1000)
        if graph:
            print(value)
            print()

        if value > max_value:
            max_value = value

    return -max_value


def run_opt(init=70, iters=30, p_r=(0, 0.3), i_r=(-0.01, 0.01), d_r=(-0.1, 0.1),
            velocity_r=[800, 1000, 1200],
            drag_r=[0.3, 0.5, 0.7],
            servo_drag_r=[2, 3, 4]):
    optim = opt.BayesianOptimization(f=lambda p, i, d: sample(p, i, d, velocity_r, drag_r, servo_drag_r), pbounds={'p': p_r, 'i': i_r, 'd': d_r})
    optim.maximize(init, iters)

    return optim.max


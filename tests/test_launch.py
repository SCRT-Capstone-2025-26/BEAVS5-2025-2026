from . import simple_sim as ss
import beavs_sim as sim
from matplotlib import pyplot as plt
import sys

def boot(s):
    s.set_pin(s.board.PIN_ARM, True)
    s.step_by(5500000)
    s.set_pin(s.board.PIN_ARM, False)
    s.step_by(500000)


def test_init():
    s = sim.Sim()
    boot(s)

    content = s.board.serial.contents()
    print(content.decode('utf-8'))

    content = s.board.sd.get_file("Logs/log_0.txt")
    print(content.decode('utf-8'))

    assert s.board.state == sim.State.ARMED


def fly(flight_sim, p=None, i=None, d=None):
    s = sim.Sim()

    if p is not None:
        s.board.P = p
    if i is not None:
        s.board.I = i
    if d is not None:
        s.board.D = d

    boot(s)

    s.board.bmp.altitude = flight_sim.height

    old_micros = s.micros
    while flight_sim.velocity >= 0:
        s.step()

        s.board.bmp.altitude = flight_sim.height

        servo = (s.get_pin(s.board.PIN_SERVO, True) - s.board.SERVO_FLUSH) / (s.board.SERVO_MAX - s.board.SERVO_FLUSH)
        assert 0 <= servo <= 1
        flight_sim.step((s.micros - old_micros) / 1000 / 1000, servo)
        old_micros = s.micros

    return s


def try_flight(velocity, drag, servo_drag, p=None, i=None, d=None, graph=False):
    flight_sim = ss.SimpleSim(velocity, drag, servo_drag)

    fly(flight_sim, p=p, i=i, d=d)

    ts, data = zip(*flight_sim.history)
    vs, ys, servs = zip(*data)

    if graph:
        plt.plot(ts, ys)
        plt.show()

        plt.plot(ts, vs)
        plt.show()

        plt.plot(ts, servs)
        plt.show()

    return ys[-1]


def test_flights():
    for velocity in [800, 1000, 1200]:
        for drag in [0.4, 0.5, 0.6]:
            apogee = try_flight(velocity, drag, 3, graph="pytest" not in sys.modules)
            assert abs(apogee - 1000) < 5


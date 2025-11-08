import beavs_sim as sim


def test_up():
    s = sim.Sim()

    while s.micros < 40000000:
        # This may not be the correct equation
        s.board.bmp.altitude = (((s.micros - 10000000) / 1000000) ** 2) / 2
        if 10000000 < s.micros:
            s.board.bno.acc_x = 30

        s.step()

    content = s.board.serial.contents()
    print(content)

    content = s.board.sd.get_file("Logs/log_0.txt")
    print(content)

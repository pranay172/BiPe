import math

from bipe.sim import KEYS, Sim, on_key


def test_sim_drives_sits_and_stands():
    s = Sim()
    on_key(s, KEYS["up"])
    on_key(s, KEYS["up"])
    on_key(s, KEYS["up"])
    for _ in range(150):  # 3 s
        s.step()
    assert abs(s.speed() - 0.3) < 0.1 and not s.fallen()
    on_key(s, KEYS["end"])
    for _ in range(200):
        s.step()
    assert s.behaviour == "sit_stand" and s.seated()
    on_key(s, KEYS["insert"])
    for _ in range(250):
        s.step()
    pitch, _, _ = s.attitude()
    assert s.behaviour == "drive" and abs(math.degrees(pitch)) < 10


def test_sim_gets_up_from_a_lean():
    s = Sim(start="back_lean")
    for _ in range(300):
        s.step()
    assert s.behaviour == "drive" and not s.fallen()

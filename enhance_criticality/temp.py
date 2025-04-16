import matplotlib.pyplot as plt
from commonroad.scenario.scenario import Scenario, ScenarioID
from commonroad.visualization.mp_renderer import MPRenderer

plt.rcParams["figure.max_open_warning"] = 50


def do_something():
    print("Hello")

    plt.figure(figsize=(25, 10))

    s_id = ScenarioID(False, "ZAM", "H-Test", 1, None, "S", None)

    s = Scenario(0.1, s_id, "Hristina", None, "TUM", None, None)
    # s.add_objects()

    my_renderer = MPRenderer()
    s.draw(my_renderer)
    my_renderer.render()

    return


do_something()

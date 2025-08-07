"""
This script can be run using the following command:
    $> python3 <path_to_script> > output.log 2>&1 &
Now the main process is running detached. You can check the progress with the following command:
    $> tail -n 100 output.log | nl
"""

import json
import multiprocessing
import os
import sys
from multiprocessing import current_process
from pathlib import Path

from tutorials import velocity_and_position

# Get the root directory (two levels up from this file)
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.append(str(PROJECT_ROOT / "scenarios"))

SCENARIOS_ROOT = Path(__file__).resolve().parent.parent.joinpath("scenarios")

SCENARIOS_LIST = [os.path.join(SCENARIOS_ROOT, name) for name in os.listdir(SCENARIOS_ROOT)]
# SCENARIOS_LIST = [os.path.join(SCENARIOS_ROOT, "DEU_Flensburg-94_1_T-1.xml")]

ITERATIONS_DEFAULT = 50


### WRAPPERS START
def run_velocity(scenario, iterations=ITERATIONS_DEFAULT):
    p = current_process()
    print(f"[PID {p.pid}] Started process '{p.name}' for scenario: {scenario}")
    return velocity_and_position.run_full_optimization_pipeline(
        scenario_path=scenario, decision_variables=[("ego", "velocity")], iterations=iterations
    )


def run_position(scenario, iterations=ITERATIONS_DEFAULT):
    p = current_process()
    print(f"[PID {p.pid}] Started process '{p.name}' for scenario: {scenario}")
    return velocity_and_position.run_full_optimization_pipeline(
        scenario_path=scenario, decision_variables=[("ego", "position")], iterations=iterations
    )


def run_both(scenario, iterations=ITERATIONS_DEFAULT):
    p = current_process()
    print(f"[PID {p.pid}] Started process '{p.name}' for scenario: {scenario}")
    return velocity_and_position.run_full_optimization_pipeline(
        scenario_path=scenario, decision_variables=[("ego", "velocity"), ("ego", "position")], iterations=iterations
    )


### WRAPPERS END

scores = {
    "velocity": {"files": [], "score": 0},
    "position": {"files": [], "score": 0},
    "both": {"files": [], "score": 0},
}

with multiprocessing.Pool(processes=multiprocessing.cpu_count() - 32) as pool:
    print(f"CPU COUNT: {multiprocessing.cpu_count()}")
    params = SCENARIOS_LIST
    try:
        result_velocity = pool.map(run_velocity, params)
    except Exception as e:
        print(f"[ERROR] Exception encountered: {e}")

    try:
        result_position = pool.map(run_position, params)
    except Exception as e:
        print(f"[ERROR] Exception encountered: {e}")

    try:
        result_both = pool.map(run_both, params)
    except Exception as e:
        print(f"[ERROR] Exception encountered: {e}")

for i in range(len(SCENARIOS_LIST)):
    scenario_path = SCENARIOS_LIST[i]
    v = result_velocity[i]
    p = result_position[i]
    b = result_both[i]

    results = {"velocity": v, "position": p, "both": b}
    best_optimized = min(results, key=results.get)
    print(f"[OK] {scenario_path}: best = {best_optimized} ({results[best_optimized]})")

    scores[best_optimized]["files"].append(scenario_path)
    scores[best_optimized]["score"] += 1

print(f"""{scores["velocity"]["score"]=}""")
print(f"""{scores["position"]["score"]=}""")
print(f"""{scores["both"]["score"]=}""")

with open("results.json", "w") as file:
    json.dump(scores, file, indent=4)

print("Done writing to file!")

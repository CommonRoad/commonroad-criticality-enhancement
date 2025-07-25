import json
import multiprocessing
import os
import sys
from multiprocessing import current_process
from pathlib import Path

from velocity_and_position import run_full_optimization_pipeline

# Get the root directory (two levels up from this file)
PROJECT_ROOT = Path(__file__).resolve().parent.parent
# Add src and scenario directories to sys.path
sys.path.append(str(PROJECT_ROOT / "src"))
sys.path.append(str(PROJECT_ROOT / "scenarios"))
sys.path.append(str(PROJECT_ROOT / "tutorials"))

# from velocity_and_position import run_full_optimization_pipeline

SCENARIOS_ROOT = Path(__file__).resolve().parent.parent.joinpath("scenarios")

SCENARIOS_LIST = [os.path.join(SCENARIOS_ROOT, name) for name in os.listdir(SCENARIOS_ROOT)]

ITERATIONS_DEFAULT = 15


### WRAPPERS START
def run_velocity(scenario, iterations=ITERATIONS_DEFAULT):
    p = current_process()
    print(f"[PID {p.pid}] Started process '{p.name}' for scenario: {scenario}")
    return run_full_optimization_pipeline(
        scenario_path=scenario, decision_variables=[("ego", "velocity")], iterations=iterations
    )


def run_position(scenario, iterations=ITERATIONS_DEFAULT):
    p = current_process()
    print(f"[PID {p.pid}] Started process '{p.name}' for scenario: {scenario}")
    return run_full_optimization_pipeline(
        scenario_path=scenario, decision_variables=[("ego", "position")], iterations=iterations
    )


def run_both(scenario, iterations=ITERATIONS_DEFAULT):
    p = current_process()
    print(f"[PID {p.pid}] Started process '{p.name}' for scenario: {scenario}")
    return run_full_optimization_pipeline(
        scenario_path=scenario, decision_variables=[("ego", "velocity"), ("ego", "position")], iterations=iterations
    )


### WRAPPERS END

scores = {
    "velocity": {"files": [], "score": 0},
    "position": {"files": [], "score": 0},
    "both": {"files": [], "score": 0},
}
corrupted_scenarios = []

with multiprocessing.Pool(processes=multiprocessing.cpu_count()) as pool:
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
    results = {"velocity": result_velocity[i], "position": result_position[i], "both": result_both[i]}

    best_optimized = min(results, key=results.get)
    print(best_optimized, results[best_optimized])

    scores[best_optimized]["files"].append(SCENARIOS_LIST[i])
    scores[best_optimized]["score"] += 1

print(f"""{scores["velocity"]["score"]=}""")
print(f"""{scores["position"]["score"]=}""")
print(f"""{scores["both"]["score"]=}""")

with open("results.json", "w") as file:
    json.dump(scores, file, indent=4)

with open("corrupted_scenarios.txt", "w") as file:
    for scenario in corrupted_scenarios:
        file.write(scenario + "\n")

print("Done writing to file!")

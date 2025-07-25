import json
import os
import sys
from pathlib import Path

from velocity_and_position import run_full_optimization_pipeline

# Get the root directory (two levels up from this file)
PROJECT_ROOT = Path(__file__).resolve().parent.parent
# Add src and scenario directories to sys.path
sys.path.append(str(PROJECT_ROOT / "src"))
sys.path.append(str(PROJECT_ROOT / "scenarios"))
sys.path.append(str(PROJECT_ROOT / "tutorials"))

SCENARIOS_ROOT = Path(__file__).resolve().parent.parent.joinpath("scenarios")

SCENARIOS_LIST = os.listdir(SCENARIOS_ROOT)


scores = {
    "velocity": {"files": [], "score": 0},
    "position": {"files": [], "score": 0},
    "both": {"files": [], "score": 0},
}
corrupted_scenarios = []

TOTAL = len(SCENARIOS_LIST)
count = 1
for scenario in SCENARIOS_LIST:
    print("**********************************************")
    print(f"RUN {count} of {TOTAL}")
    print("**********************************************")
    try:
        result_velocity = run_full_optimization_pipeline(
            scenario_path=str(SCENARIOS_ROOT / scenario), decision_variables=[("ego", "velocity")]
        )
        print("********** >>>> VELOCITY DONE <<<< **********")
        result_position = run_full_optimization_pipeline(
            scenario_path=str(SCENARIOS_ROOT / scenario), decision_variables=[("ego", "position")]
        )
        print("********** >>>> POSITION DONE <<<< **********")
        result_both = run_full_optimization_pipeline(
            scenario_path=str(SCENARIOS_ROOT / scenario), decision_variables=[("ego", "position"), ("ego", "velocity")]
        )
        print("********** >>>> BOTH DONE <<<< **********")
    except Exception as e:
        corrupted_scenarios.append(scenario)
        count += 1
        continue

    results = {"velocity": result_velocity, "position": result_position, "both": result_both}

    best_optimized = min(results, key=results.get)
    print(best_optimized, results[best_optimized])

    scores[best_optimized]["files"].append(scenario)
    scores[best_optimized]["score"] += 1
    count += 1


print(f"""{scores["velocity"]["score"]=}""")
print(f"""{scores["position"]["score"]=}""")
print(f"""{scores["both"]["score"]=}""")

with open("results.json", "w") as file:
    json.dump(scores, file, indent=4)

with open("corrupted_scenarios.txt", "w") as file:
    for scenario in corrupted_scenarios:
        file.write(scenario + "\n")

print("Done writing to file!")

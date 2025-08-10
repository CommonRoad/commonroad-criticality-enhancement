import json
import os
import sys
from pathlib import Path

from tutorials import velocity_and_position

# Get the root directory (two levels up from this file)
PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.append(str(PROJECT_ROOT / "scenarios"))

SCENARIOS_ROOT = Path(__file__).resolve().parent.parent.joinpath("scenarios")

# Prepare all scenarios
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
        # Run each scenario with the 3 options for decision variables
        result_velocity = velocity_and_position.run_full_optimization_pipeline(
            scenario_path=str(SCENARIOS_ROOT / scenario), decision_variables=[("ego", "velocity")]
        )
        print("********** >>>> VELOCITY DONE <<<< **********")
        result_position = velocity_and_position.run_full_optimization_pipeline(
            scenario_path=str(SCENARIOS_ROOT / scenario), decision_variables=[("ego", "position")]
        )
        print("********** >>>> POSITION DONE <<<< **********")
        result_both = velocity_and_position.run_full_optimization_pipeline(
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

# Print the counts for each decision variable
print(f"""{scores["velocity"]["score"]=}""")
print(f"""{scores["position"]["score"]=}""")
print(f"""{scores["both"]["score"]=}""")

with open("results.json", "w") as file:
    json.dump(scores, file, indent=4)

# Save scenarios that throw exception
with open("corrupted_scenarios.txt", "w") as file:
    for scenario in corrupted_scenarios:
        file.write(scenario + "\n")

print("Done writing to file!")

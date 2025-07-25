import multiprocessing
import sys
from pathlib import Path

from velocity_and_position import run_full_optimization_pipeline

# Get the root directory (two levels up from this file)
# Get the root directory (two levels up from this file)
PROJECT_ROOT = Path(__file__).resolve().parent.parent
# Add src and scenario directories to sys.path
sys.path.append(str(PROJECT_ROOT / "src"))
sys.path.append(str(PROJECT_ROOT / "scenarios"))
sys.path.append(str(PROJECT_ROOT / "tutorials"))

SCENARIOS_ROOT = Path(__file__).resolve().parent.parent.joinpath("scenarios")

SCENARIO = "DEU_Guetersloh-10_5_T-1.xml"

# results = []
for i in range(1, 11):
    print("#####################################")
    print(f"ITERATION {i} of 10")
    print("#####################################")
    # try:
    #     with multiprocessing.Pool(processes=multiprocessing.cpu_count()) as pool:
    #         params = [
    #             (str(SCENARIOS_ROOT / SCENARIO), [("ego", "position"), ("ego", "velocity")], i),
    #         ]
    #         results = pool.starmap(run_full_optimization_pipeline, params)
    #     with multiprocessing.Pool(processes=multiprocessing.cpu_count()) as pool:
    #         params = [
    #             (str(SCENARIOS_ROOT / SCENARIO), [("ego", "velocity")], i)
    #         ]
    #         results = pool.starmap(run_full_optimization_pipeline, params)

    result_velocity = run_full_optimization_pipeline(
        scenario_path=str(SCENARIOS_ROOT / SCENARIO), iterations=i, decision_variables=[("ego", "velocity")]
    )
    result_both = run_full_optimization_pipeline(
        scenario_path=str(SCENARIOS_ROOT / SCENARIO),
        iterations=i,
        decision_variables=[("ego", "position"), ("ego", "velocity")],
    )

    # results.append((i, result_velocity, result_both))
    # except Exception as e:
    #     print(f"[ERROR] Exception occurred in iteration {i}! Details: {e.with_traceback()}")

# print(f"{results=}")

#####################################
#####################################
#####################################

# result_velocity = run_full_optimization_pipeline(
#     scenario_path=str(SCENARIOS_ROOT / SCENARIO), iterations=4, decision_variables=[("ego", "velocity")]
# )
print(f"{result_velocity=}")

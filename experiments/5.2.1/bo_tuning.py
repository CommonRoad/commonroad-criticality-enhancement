import sys
import time
from pathlib import Path
from typing import List, Tuple

from src import bo, reach_flow

# Get the root directory (two levels up from this file)
PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.append(str(PROJECT_ROOT / "scenarios"))


def run_pipeline(
    scenario_path: str,
    decision_variables: List[Tuple[str, str]],
    budget: int = 10,
) -> None:
    print("Computing original drivable area...")
    area_original = reach_flow.compute_drivable_area(scenario_path)

    print("\nRunning Bayesian Optimization ...")
    start_bo = time.time()
    bo_best_params, bo_area = bo.run_bo_multi_variable(
        scenario_path=scenario_path,
        decision_variables=decision_variables,
        budget=budget,
    )
    end_bo = time.time()

    print(f"Sum of Original drivable area: {sum(area_original)}")
    print(f"BO optimization time: {end_bo - start_bo:.2f} seconds")
    print(f"BO optimization params: {bo_best_params}")
    print(f"Sum of BO drivable area: {sum(bo_area)}")


scenario_path = PROJECT_ROOT / "scenarios" / "DEU_Lohmar-32_1_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "ZAM_two_lanes_solid.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "ITA_Empoli-2_5_T-1.xml"

# Run for all 3 scenarios with different budgets to reproduce the table
run_pipeline(
    str(scenario_path),
    decision_variables=[("ego", "position"), ("ego", "velocity")],
    budget=500,
)

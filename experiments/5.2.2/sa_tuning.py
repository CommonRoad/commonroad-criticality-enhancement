import sys
import time
from pathlib import Path
from typing import List, Tuple

from src import reach_flow, sa

# Get the root directory (two levels up from this file)
PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.append(str(PROJECT_ROOT / "scenarios"))


def run_sa_pipeline(
    scenario_path: str,
    decision_variables: List[Tuple[str, str]],
    sa_max_iter: int = 500,
    sa_initial_temp: float = 2000.0,
) -> None:
    print("Computing original drivable area...")
    area_original = reach_flow.compute_drivable_area(scenario_path)

    print("\nRunning SA Optimization...")
    start_sa = time.time()
    sa_best_params, sa_area = sa.run_sa_with_scipy(
        scenario_path=scenario_path,
        decision_variables=decision_variables,
        max_iter=sa_max_iter,
        initial_temp=sa_initial_temp,
    )
    end_sa = time.time()

    print(f"Sum of Original drivable area: {sum(area_original)}")
    print(f"SA optimization time: {end_sa - start_sa:.2f} seconds")
    print(f"SA optimization velocity: {sa_best_params}")
    print(f"Sum of SA drivable area: {sum(sa_area)}")


scenario_path = PROJECT_ROOT / "scenarios" / "DEU_Lohmar-32_1_T-1.xml"


# Run with diffrerent parameters to reproduce the values from the table
run_sa_pipeline(
    scenario_path=str(scenario_path),
    decision_variables=[("ego", "velocity"), ("ego", "position")],
    sa_max_iter=30,
    sa_initial_temp=500.0,
)

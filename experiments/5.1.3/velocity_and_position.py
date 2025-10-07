import sys
from pathlib import Path

from commonroad.common.file_reader import CommonRoadFileReader

from commonroad_criticality_enhancement import optimization, reach_flow

# Get the root directory (two levels up from this file)
PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.append(str(PROJECT_ROOT / "scenarios"))


def run_full_optimization_pipeline(
    scenario_path: str,
    decision_variables: list,
    iterations: int = 20,
    a_ref_input: float = 1.0,
    semantics: str = "true",
) -> int:
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()
    area_original = reach_flow.compute_drivable_area(scenario, planning_problem_set, semantics)

    # Run gradient-based optimization
    final_params, area_modified = optimization.optimize(
        scenario,
        planning_problem_set,
        decision_variables=decision_variables,
        iterations=iterations,
        a_ref_input=a_ref_input,
        semantics=semantics,
    )
    print("final_params:", final_params)

    print(f"original objective: {(sum(area_original) - a_ref_input) ** 2}")
    print(f"modified area: {(sum(area_modified) - a_ref_input) ** 2}")

    return sum((area_modified - a_ref_input) ** 2)


scenario_path = PROJECT_ROOT / "scenarios" / "ITA_Foggia-6_1_T-1.xml"


# Run all 4 to reproduce the table values
run_full_optimization_pipeline(str(scenario_path), [("ego", "velocity"), ("ego", "position")])
# run_full_optimization_pipeline(str(scenario_path), [("ego", "velocity")])
# run_full_optimization_pipeline(str(scenario_path), [("ego", "position")])
# run_full_optimization_pipeline(str(scenario_path), [("ego", "position"), ("ego", "velocity")])

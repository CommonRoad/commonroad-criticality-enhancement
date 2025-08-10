import sys
from pathlib import Path

from commonroad.common.file_reader import CommonRoadFileReader

from src import optimization, reach_flow

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

    area_original = reach_flow.compute_drivable_area(scenario_path, semantics)

    # Run gradient-based optimization
    final_params, area_modified = optimization.optimize(
        scenario,
        planning_problem_set,
        scenario_path,
        decision_variables=decision_variables,
        iterations=iterations,
        a_ref_input=a_ref_input,
        semantics=semantics,
    )
    print("final_params:", final_params)

    print(f"original area: {(sum(area_original) - a_ref_input) ** 2}")
    print(f"modified area: {(sum(area_modified) - a_ref_input) ** 2}")

    return (sum(area_modified) - a_ref_input) ** 2


# scenario_path = PROJECT_ROOT / "scenarios" / "DEU_Flensburg-94_1_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "BEL_Aarschot-6_1_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "ZAM_Over-1_1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "DEU_Moelln-7_1_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "DEU_Lohmar-32_1_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "ZAM_two_lanes_solid.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "BEL_Putte-3_1_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "C-DEU_B471-1_3_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "DEU_IV21-2_1_T-1.xml"
scenario_path = PROJECT_ROOT / "scenarios" / "USA_US101-11_4_T-1.xml"

# Run for all scenarios and for all milestone iteration counts to get the data for mean_plot.py
run_full_optimization_pipeline(str(scenario_path), [("ego", "velocity")])

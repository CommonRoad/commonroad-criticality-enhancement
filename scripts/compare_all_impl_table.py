import multiprocessing
import os
from multiprocessing import current_process
from pathlib import Path
from typing import List, Tuple

from commonroad.common.file_reader import CommonRoadFileReader

from src import bo, optimization, reach_flow, sa


def run_comparison_pipeline(
    scenario_path: str,
    decision_variables: List[Tuple[str, str]],
    iterations: int = 50,
    a_ref_input: float = 1.0,
    sa_max_iter: int = 200,
    sa_initial_temp: float = 500.0,
    budget: int = 500,
) -> dict:
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

    print("Computing original drivable area...")
    area_original = reach_flow.compute_drivable_area(scenario_path)

    print("\nRunning Gradient-Based Optimization (ECOS)...")
    velocity_gradient, area_gradient = optimization.optimize(
        scenario,
        planning_problem_set,
        scenario_path,
        decision_variables=decision_variables,
        iterations=iterations,
        a_ref_input=a_ref_input,
    )

    print("\nRunning SA ...")
    sa_best_params, sa_area = sa.run_sa_with_scipy(
        scenario_path=scenario_path,
        decision_variables=decision_variables,
        max_iter=sa_max_iter,
        initial_temp=sa_initial_temp,
    )

    print("\nRunning Bayesian Optimization ...")
    bo_best_params, bo_area = bo.run_bo_multi_variable(
        scenario_path=scenario_path,
        decision_variables=decision_variables,
        budget=budget,
    )

    print(f"Sum of Original drivable area: {sum((area_original - a_ref_input) ** 2)}")
    print(f"Gradient optimization params (vel, x-pos, y-pos): {velocity_gradient}")
    print(f"Sum of Gradient drivable area: {sum((area_gradient - a_ref_input) ** 2)}")
    print(f"SA optimization params: {sa_best_params}")
    print(f"Sum of SA drivable area: {sum((sa_area - a_ref_input) ** 2)}")
    print(f"BO optimization params: {bo_best_params}")
    print(f"Sum of BO drivable area: {sum((bo_area - a_ref_input) ** 2)}")

    return {
        "original": sum((area_original - a_ref_input) ** 2),
        "gradient": sum((area_gradient - a_ref_input) ** 2),
        "sa_area": sum((sa_area - a_ref_input) ** 2),
        "bo_area": sum((bo_area - a_ref_input) ** 2),
    }


def wrapper_rc_pipeline(path: str):
    p = current_process()
    print(f"[PID {p.pid}] Started process '{p.name}' for scenario: {path}")
    return run_comparison_pipeline(
        path,
        decision_variables=[("ego", "velocity"), ("ego", "position")],
        iterations=3,
        a_ref_input=1.0,
        sa_max_iter=50,
        sa_initial_temp=500.0,
        budget=150,
    )


SCENARIOS_ROOT = Path(__file__).resolve().parent.parent.joinpath("scenarios")
SCENARIOS_LIST = [
    "DEU_Flensburg-94_1_T-1.xml",
    "BEL_Aarschot-6_1_T-1.xml",
    "ZAM_Over-1_1.xml",
    "DEU_Moelln-7_1_T-1.xml",
    "DEU_Lohmar-32_1_T-1.xml",
    "ZAM_two_lanes_solid.xml",
    "BEL_Putte-3_1_T-1.xml",
    "C-DEU_B471-1_3_T-1.xml",
    "DEU_IV21-2_1_T-1.xml",
    "USA_US101-11_4_T-1.xml",
]
SCENARIOS_LIST = [os.path.join(SCENARIOS_ROOT, name) for name in SCENARIOS_LIST]
print(SCENARIOS_LIST)

with multiprocessing.Pool(processes=multiprocessing.cpu_count()) as pool:
    params = SCENARIOS_LIST
    try:
        results = pool.map(wrapper_rc_pipeline, params)
    except Exception as e:
        print(f"[ERROR] Exception occurred: {e}")

# Printing to put directly into LaTeX

print("""\\begin{table}[H]""")
print("""\t\\centering""")
print("""\t\\caption{Drivable Area for each Optimizer}""")
print("""\t\\begin{tabular}{|c|c|c|c|c|}""")
print("""\t\t\\hline""")
print("""\t\t\\textbf{Approach:} & Original Area & Gradient & SA & BO \\\\""")
print("""\t\t\\hline""")
for i in range(len(SCENARIOS_LIST)):
    scenario_name = SCENARIOS_LIST[i].split("/")[-1].replace("_", "\\_")
    original_area = round(results[i]["original"])
    gradient_area = round(results[i]["gradient"])
    sa_area = round(results[i]["sa_area"])
    bo_area = round(results[i]["bo_area"])

    print(f"""\t\t\\textbf{{{scenario_name}}} & {original_area} & {gradient_area} & {sa_area} & {bo_area} \\\\""")
    print("""\t\t\\hline""")

print("""\t\\end{tabular}""")
print("""\t\\label{tab:combined}""")
print("""\\end{table}""")

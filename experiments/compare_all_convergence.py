import csv
import multiprocessing
import os
from functools import partial
from pathlib import Path
from typing import Dict, List, Tuple

import matplot2tikz
import matplotlib.pyplot as plt
import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader

from src import bo, optimization, reach_flow, sa

# ===== CONFIGURATION ===== #
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
SCENARIOS_LIST = [str(SCENARIOS_ROOT / name) for name in SCENARIOS_LIST]

DECISION_VARS = [("ego", "velocity"), ("ego", "position")]
A_REF = 1.0

SA_BO_BUDGETS = list(range(0, 501, 10))  # 0 to 500
GRADIENT_BUDGETS = [0, 10, 20, 30, 40, 50, 60, 100]


# ===== RUN OPTIMIZER FUNCTION ===== #
def run_optimizer_all_budgets(method: str, scenario_path: str) -> Tuple[str, Dict[int, float]]:
    area_original = reach_flow.compute_drivable_area(scenario_path)
    original_sum = sum(area_original)

    if method in ["sa", "bo"]:
        budgets = SA_BO_BUDGETS
    elif method == "gradient":
        budgets = GRADIENT_BUDGETS
    else:
        raise ValueError(f"Unknown method: {method}")

    result = {}

    for budget in budgets:
        try:
            if budget == 0:
                result[budget] = 100.0
                continue

            if method == "sa":
                _, area_optimized = sa.run_sa_with_scipy(
                    scenario_path=scenario_path,
                    decision_variables=DECISION_VARS,
                    max_iter=budget,
                    initial_temp=500.0,
                )
            elif method == "bo":
                _, area_optimized = bo.run_bo_multi_variable(
                    scenario_path=scenario_path,
                    decision_variables=DECISION_VARS,
                    budget=budget,
                )
            elif method == "gradient":
                scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()
                _, area_optimized = optimization.optimize(
                    scenario,
                    planning_problem_set,
                    scenario_path,
                    decision_variables=DECISION_VARS,
                    iterations=budget,
                    a_ref_input=A_REF,
                )

            optimized_sum = sum(area_optimized)
            result[budget] = (optimized_sum / original_sum) * 100
        except Exception as e:
            print(f"[ERROR] {method.upper()} @ {budget} on {os.path.basename(scenario_path)}: {e}")
            result[budget] = np.nan

    return scenario_path, result


# ===== AGGREGATION ===== #
def aggregate_results(raw_results: List[Tuple[str, Dict[int, float]]]) -> Dict[int, float]:
    budgets = raw_results[0][1].keys()
    aggregated = {}

    for budget in budgets:
        vals = [res[1][budget] for res in raw_results if not np.isnan(res[1][budget])]
        aggregated[budget] = np.mean(vals) if vals else np.nan

    return aggregated


# ===== PLOTTING ===== #
def plot_results(results: dict):
    with open("area_reduction_data.csv", mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Method", "Budget", "Mean Area (%)"])
        for method, data in results.items():
            for budget, mean_area in sorted(data.items()):
                writer.writerow([method, budget, mean_area])

    plt.figure(figsize=(10, 5))

    for method, data in results.items():
        budgets = sorted(data.keys())
        reductions = [data[b] for b in budgets]
        label = method.capitalize()
        plt.plot(budgets, reductions, label=label)

    plt.xlabel("Iteration Budget")
    plt.ylabel("Mean Drivable Area (%)")
    plt.title("Mean Area vs. Iteration Budget")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    matplot2tikz.save("area_reduction_plot.tex", axis_width="12cm", axis_height="6cm")
    plt.savefig("area_reduction_plot.png", dpi=300)
    plt.show()


# ===== MAIN ===== #
def main():
    all_methods = ["sa", "bo", "gradient"]
    final_results = {}

    with multiprocessing.Pool(processes=multiprocessing.cpu_count() - 32) as pool:
        print(f"Using {multiprocessing.cpu_count() - 32} worker processes...")

        for method in all_methods:
            print(f"\n--- Running method: {method.upper()} ---")
            run_method = partial(run_optimizer_all_budgets, method)
            try:
                raw_results = pool.map(run_method, SCENARIOS_LIST)
            except Exception as e:
                print(f"[ERROR] Exception occurred during {method.upper()}: {e}")
                continue

            aggregated = aggregate_results(raw_results)
            final_results[method] = aggregated

    plot_results(final_results)


if __name__ == "__main__":
    main()

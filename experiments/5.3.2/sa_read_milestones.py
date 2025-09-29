from pathlib import Path

from commonroad_criticality_enhancement import sa

# ===== CONFIGURATION ===== #
SCENARIOS_ROOT = Path(__file__).resolve().parent.parent.parent.joinpath("scenarios")
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

DECISION_VARS = [("ego", "velocity")]
A_REF = 1.0

iteration_counter = {"count": 0}


def checkpoint_saver_SA(x, f, context):
    iteration_counter["count"] += 1
    print(f"Current SA Iteration: {iteration_counter['count']} (after {sa.evaluation_counter['count']} evaluations)")
    # Write to file when a better objective value is computed and the samples so far (used for generating the plot)
    with open(f"SA_{iteration_counter['count']}.txt", "w") as f_out:
        f_out.write(f"Milestone achieved in Iteration: {iteration_counter['count']}\n")
        f_out.write(f"Best f(x) so far: {f}\n")
        f_out.write(f"Evaluations so far: {sa.evaluation_counter['count']}\n")


# Run bo for many iterations and the objective values will get saved any time when it changes
# The results of this script were used to generate the data for the plot (mean_plot.py)
# Run for all 10 scenarios
sa.run_sa_with_scipy(
    scenario_path=SCENARIOS_LIST[9], decision_variables=DECISION_VARS, max_iter=150, callback=checkpoint_saver_SA
)

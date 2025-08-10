from src import sa
from pathlib import Path

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

DECISION_VARS = [("ego", "velocity")]
A_REF = 1.0

iteration_counter = {"count": 0}
def checkpoint_saver_SA(x, f, context):
    iteration_counter["count"] += 1
    print(
        f"Current SA Iteration: {iteration_counter['count']} "
        f"(after {sa.evaluation_counter['count']} evaluations)"
    )

    milestones = (5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 60, 70, 80)

    # if iteration_counter["count"] in milestones:
    with open(f"SA_{iteration_counter['count']}.txt", "w") as f_out:
        f_out.write(f"Milestone achieved in Iteration: {iteration_counter['count']}\n")
        f_out.write(f"Best f(x) so far: {f}\n")
        f_out.write(f"Evaluations so far: {sa.evaluation_counter['count']}\n")

sa.run_sa_with_scipy(scenario_path=SCENARIOS_LIST[9],
                     decision_variables=DECISION_VARS,
                     max_iter=400,
                     callback=checkpoint_saver_SA)

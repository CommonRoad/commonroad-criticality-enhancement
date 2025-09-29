from pathlib import Path

from commonroad.common.file_reader import CommonRoadFileReader
from scipy.optimize import OptimizeResult

from commonroad_criticality_enhancement import bo

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

SA_BO_BUDGETS = list(range(0, 501, 10))  # 0 to 500


def checkpoint_saver_BO(res: OptimizeResult):
    milestones = (
        5,
        10,
        15,
        20,
        25,
        30,
        35,
        40,
        45,
        50,
        60,
        70,
        80,
        90,
        100,
        110,
        120,
        130,
        140,
        150,
        160,
        170,
        180,
        190,
        200,
        210,
        220,
        230,
        240,
        250,
        260,
        270,
        280,
        290,
        300,
        310,
        320,
        330,
        340,
        350,
        360,
        370,
        380,
        390,
        400,
    )
    current_iteration = len(res.x_iters)
    # Save data when a milestone iteration is reached
    if current_iteration in milestones:
        with open(f"""BO_{current_iteration}.txt""", "w") as f:
            f.write("Milestone achieved in Iteration: " + str(len(res.x_iters)) + "\n")
            f.write(f"""Best f(x) so far: {res.fun}""")


def progress_reporter_BO(res: OptimizeResult):
    print(f"""Current BO Iteration: {len(res.x_iters)}""")


scenario, planning_problem_set = CommonRoadFileReader(SCENARIOS_LIST[0]).open()
# Run bo for many iterations and the objective values will get saved at milestones
# The results of this script were used to generate the data for the plot (mean_plot.py)
# Run for all 10 scenarios
bo.run_bo_multi_variable(
    scenario,
    planning_problem_set,
    decision_variables=DECISION_VARS,
    budget=400,
    a_ref=A_REF,
    callback=[checkpoint_saver_BO, progress_reporter_BO],
)

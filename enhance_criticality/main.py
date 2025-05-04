import argparse
import pathlib
import reach_flow

from commonroad.common.file_reader import CommonRoadFileReader
from optimization import optimize_velocity
from reachability import load_scenario_and_compute_reachability


# CLI
def main():
    parser = argparse.ArgumentParser(
        description="Process a CommonRoad scenario and run optimization"
    )
    # parser.add_argument("scenario_path", type=str, help="Path to the CommonRoad XML scenario file.")
    # parser.add_argument("--scenario", required=True)
    # parser.add_argument("--iterations", type=int, default=10)
    # parser.add_argument("--a_ref", type=float, default=1.0)

    args = parser.parse_args()

    run_full_optimization_pipeline("DEU_Test-1_1_T-1", [("ego", "velocity")])


def run_full_optimization_pipeline(
    scenario_name: str, decision_variables: list, iterations: int = 5, a_ref_input: float = 1.0
):
    reach_interface = load_scenario_and_compute_reachability(scenario_name)

    scenario_file = pathlib.Path(__file__).parent.joinpath(f"./../scenarios/{scenario_name}.xml")
    scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()
    reach_flow.create_reach_graph(scenario, planning_problem_set, reach_interface)

    # Print the initial state of the ego vehicle
    # print(planning_problem.initial_state)

    # final_velocity = optimize_velocity(
    #     scenario,
    #     planning_problem_set,
    #     scenario_name,
    #     vehicle=planning_problem_set,
    #     decision_variables=decision_variables,
    # )


if __name__ == "__main__":
    main()

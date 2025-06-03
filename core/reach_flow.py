import time
from typing import Tuple

import commonroad_dc.pycrccosy as pycrccosy
import cr_reach_flow.cr_reach_flow_core as core
import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
from commonroad_route_planner.route_planner import RoutePlanner
from cr_reach_flow.collision_checker.collision_checker_factory import CollisionCheckerFactory
from cr_reach_flow.scenario.resampling import resample_scenario
from cr_reach_flow.visualization.scenario import draw_with_reach_set
from matplotlib import pyplot as plt


def compute_drivable_area(scenario_path: str) -> float:
    """
    Computes the drivable area for a given scenario.

    Parameters:
    - scenario_path (str): Path to the scenario file.

    Returns:
    - float: The computed drivable area.
    """
    graph, step_start, step_end, planning_problem, clcs = create_reach_graph(scenario_path)
    area = compute_area(graph, step_start, step_end)
    return area


def draw_reach_sets_end(
    step_end: int,
    scenario: Scenario,
    planning_problem: PlanningProblem,
    graph: object,
    clcs: object,
) -> None:
    """
    Draws the reachability sets at the final time step.

    Parameters:
    - step_end (int): The final time step.
    - scenario: The scenario object.
    - planning_problem: The planning problem definition.
    - graph: The reachability graph.
    - clcs: The list of collision-free trajectory sets.

    Returns:
    - None
    """
    fig, ax = plt.subplots(figsize=(10, 6))
    draw_with_reach_set(step_end, scenario, planning_problem, graph, clcs, ax)
    plt.show()


def plot(area_original: np.ndarray, area_modified: np.ndarray) -> None:
    """
    Plots a comparison of drivable area over time for original and modified scenarios.

    Parameters:
    - area_original (np.ndarray): 1D array of drivable area values for the original scenario.
    - area_modified (np.ndarray): 1D array of drivable area values for the modified scenario.

    Returns:
    - None
    """
    time_steps = np.arange(len(area_original))
    plt.figure(figsize=(10, 5))
    plt.plot(time_steps, area_original, label="Original Scenario", marker="o")
    plt.plot(time_steps, area_modified, label="Modified Scenario", marker="s")
    plt.xlabel("Time Step")
    plt.ylabel("Drivable Area")
    plt.title("Comparison of Drivable Areas Over Time")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


def compute_area(graph: object, step_start: int, step_end: int) -> np.ndarray:
    """
    Computes the drivable area between two time steps using the reachability graph.

    Parameters:
    - graph: The reachability graph structure (type unspecified).
    - step_start (int): The starting time step.
    - step_end (int): The ending time step.

    Returns:
    -  np.ndarray: 1D array of drivable area values for each time step in the range.
    """
    areas = np.full((step_end + 1), -1.0)

    for t in range(step_start, step_end + 1):
        try:
            nodes = graph.get_nodes_at_step(t)
            if not nodes:
                print(f"Warning: No reachable nodes at step {t}")
        except AttributeError:
            raise RuntimeError(f"Graph does not support time step access at t={t}")

        area = 0.0
        for node_ptr in nodes:
            if hasattr(node_ptr, "set"):
                node_set = node_ptr.set

                # Extract longitude and latitude bounds
                try:
                    lon_min = node_set.p_lon_min
                    lon_max = node_set.p_lon_max
                    lat_min = node_set.p_lat_min
                    lat_max = node_set.p_lat_max

                    # Compute the area
                    width = lon_max - lon_min
                    height = lat_max - lat_min
                    area += width * height

                except AttributeError:
                    print(f"Warning: Node at step {t} has an incomplete set.")
                    continue
            else:
                print(f"Warning: Node at step {t} has no 'set' attribute.")
                continue
        if area < 1e-5:
            print(f"Warning: Area  at step {t} is too small.")
        areas[t] = area if area > 0 else 0.0

    return areas


def create_reach_graph(
    scenario_path: str, semantics: str = "true"
) -> Tuple[object, int, int, PlanningProblem, object]:
    """
    Loads a CommonRoad scenario, configures the reachability executor, and computes the reachability graph.

    Parameters:
    - scenario_path (str): Path to the XML file containing the CommonRoad scenario.
    - semantics (str, optional): Semantics of the scenario. Default is "true".
    Returns:
    - Tuple:
        - graph (object): The reachability graph with reachable states.
        - step_start (int): The starting time step used for computation.
        - step_end (int): The ending time step used for computation.
        - planning_problem (PlanningProblem): The planning problem extracted from the scenario.
        - clcs (object): Curvilinear coordinate system generated from the route.
    """
    dt = 0.2
    step_start = 0
    step_end = 20
    initial_uncertainty = 0.01

    point_mass_params = core.layers.propagation.PointMassParameters()
    point_mass_params.a_lon_min = -9.5
    point_mass_params.a_lon_max = 11.5
    point_mass_params.a_lat_min = -2.0
    point_mass_params.a_lat_max = 2.0
    point_mass_params.v_lon_min = 0.0
    point_mass_params.v_lon_max = 50.8
    point_mass_params.v_lat_min = -4.0
    point_mass_params.v_lat_max = 4.0
    predicate_config = core.model_checking.PredicateConfiguration()
    inflation_radius = (
        predicate_config.ego_width / 2
        if predicate_config.ego_width < predicate_config.ego_length
        else predicate_config.ego_length / 2
    )
    splitter_params = core.layers.semantic.SemanticSplitterParameters()
    splitter_params.minimum_region_area = 0.01
    splitter_params.lanelet_inflation_radius = inflation_radius

    scenario, planning_problems = CommonRoadFileReader(scenario_path).open()
    scenario, planning_problems = resample_scenario(scenario, planning_problems, dt)
    planning_problem = list(planning_problems.planning_problem_dict.values())[0]

    # plan route and create clcs
    route = RoutePlanner(scenario, planning_problem).plan_routes().retrieve_first_route()
    splitter_params.route_lanelet_ids = set(route.lanelet_ids)
    lanelet_ids = splitter_params.route_lanelet_ids
    reference_path = pycrccosy.Util.resample_polyline(route.reference_path, 2.0)
    clcs = pycrccosy.CurvilinearCoordinateSystem(reference_path)
    print(f"Route lanelet IDs: {lanelet_ids}")

    # create reach set executor
    cc = CollisionCheckerFactory(
        step_start, step_end, inflation_radius
    ).create_curvilinear_collision_checker(scenario, clcs)

    # specs = ["G (InLanelet_13 | InLanelet_522 | InLanelet_946)"]
    lanelet_conditions = " | ".join(f"InLanelet_{lid}" for lid in lanelet_ids)
    specs = [f"G (({lanelet_conditions}) & ({semantics}))"]
    print(specs)

    automaton = core.model_checking.FiniteAutomaton(specs)
    init = core.initializers.base_set.CurvilinearUncertaintyInitializer(
        clcs, *([initial_uncertainty] * 4)
    )
    layers = [
        core.layers.propagation.PointMassPropagator(dt, point_mass_params),
        core.layers.semantic.SemanticSplitter(automaton, scenario_path, dt, clcs, splitter_params),
        core.layers.meta.GroupedByAutomatonStates(core.layers.repartition.PositionRepartitioner()),
        core.layers.collision.CollisionFilter(cc),
        core.layers.meta.GroupedByAutomatonStates(core.layers.repartition.PositionRepartitioner()),
    ]
    post = [
        core.post_processors.pruning.SemanticFinalStatePruner(automaton),
        core.post_processors.pruning.DanglingNodePruner(),
    ]

    rs = core.executors.DynamicReachExecutor(
        init, core.layers.meta.Sequential(layers), core.post_processors.meta.Sequential(post)
    )

    tic = time.perf_counter()
    rs.initialize(*initialize_from_planning_problem(planning_problem))
    toc = time.perf_counter()
    print(f"Initialization took {toc - tic:3f} seconds")

    try:
        tic = time.perf_counter()
        rs.compute(step_start + 1, step_end)
        toc = time.perf_counter()
        print(f"Reachable set computation took {toc - tic:3f} seconds")

        # create reachability graph
        tic = time.perf_counter()
        graph = rs.get_post_processed_reach_graph()
        toc = time.perf_counter()
        print(f"Graph creation took {toc - tic:3f} seconds")

        # Check if the graph contains any reachable nodes
        has_nodes = any(
            len(graph.get_nodes_at_step(t)) > 0 for t in range(step_start + 1, step_end + 1)
        )
        if not has_nodes:
            raise RuntimeError("Reachability graph is empty – possibly due to invalid parameters.")

    except RuntimeError as e:
        print(f"Warning: Error computing reachable sets: {e}")

    return graph, step_start, step_end, planning_problem, clcs


def initialize_from_planning_problem(
    planning_problem: PlanningProblem,
) -> Tuple[int, float, float, float, float, float]:
    """
    Extracts the initial state from a PlanningProblem and formats it as a tuple
    of initialization parameters for an executor or simulator.

    Parameters:
    - planning_problem (PlanningProblem): The planning problem containing the initial state.

    Returns:
    - Tuple[int, float, float, float, float, float]: A tuple containing:
        (time_step, position_x, position_y, velocity, acceleration (0), orientation)
    """
    state = planning_problem.initial_state
    # the planning problem does not contain an acceleration, so we return 0
    return (
        state.time_step,
        state.position[0],
        state.position[1],
        state.velocity,
        0,
        state.orientation,
    )

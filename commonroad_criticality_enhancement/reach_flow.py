import time
from typing import Tuple

import commonroad_clcs.pycrccosy as pycrccosy
import commonroad_route_planner.fast_api.fast_api as route_planner
import cr_reach_flow.cr_reach_flow_core as core
import matplot2tikz
import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
from cr_reach_flow.collision_checker.collision_checker_factory import CollisionCheckerFactory
from cr_reach_flow.cr_reach_flow_core.graphs import DynamicReachGraph
from cr_reach_flow.scenario.resampling import resample_scenario
from cr_reach_flow.visualization.scenario import draw_with_reach_set
from crcpp import World
from matplotlib import pyplot as plt
from numpy import ndarray


def compute_drivable_area(scenario_path: str, semantics: str = "true") -> np.ndarray:
    """
    Computes the drivable area for a given scenario.

    Parameters
    ----------
    scenario_path : str
        Path to the CommonRoad scenario file.

    semantics : str, optional
        The semantics to use in reachability analysis. Defaults to "true".

    Returns
    -------
    np.ndarray
        An array containing the computed drivable area over time.
    """
    graph, step_start, step_end, planning_problem, clcs = create_reach_graph(scenario_path, semantics=semantics)
    area = compute_area(graph, step_start, step_end)
    return area


def draw_reach_sets_end(
    step_end: int,
    scenario: Scenario,
    planning_problem: PlanningProblem,
    graph: DynamicReachGraph,
    clcs: pycrccosy.CurvilinearCoordinateSystem,
) -> None:
    """
    Draws the reachability sets at the final time step.

    Parameters
    ----------
    step_end : int
        The final time step.

    scenario : Scenario
        The scenario object containing all dynamic and static elements.

    planning_problem : PlanningProblem
        The planning problem definition specifying the ego vehicle's goal and initial state.

    graph : DynamicReachGraph
        The reachability graph structure mapping states across time steps.

    clcs : CurvilinearCoordinateSystem
        The curvilinear coordinate system.

    Returns
    -------
    None
    """
    fig, ax = plt.subplots(figsize=(10, 6))
    draw_with_reach_set(step_end, scenario, planning_problem, graph, clcs, ax)
    plt.show()


def plot(area_original: np.ndarray, area_modified: np.ndarray, output_path="plot.tex") -> None:
    """
    Plots a comparison of drivable area over time for original and modified scenarios.

    Parameters
    ----------
    area_original : np.ndarray
        1D array of drivable area values for the original scenario.

    area_modified : np.ndarray
        1D array of drivable area values for the modified scenario.
    output_path : str
        Path to save the TikZ output (e.g., 'myplot.tex')

    Returns
    -------
    None
    """
    time_steps = np.arange(len(area_original))
    plt.figure(figsize=(10, 5))
    plt.plot(time_steps, area_original, label="Original")
    plt.plot(time_steps, area_modified, label="Modified")
    plt.xlabel("Time Step")
    plt.ylabel("Drivable Area")
    plt.legend()
    plt.grid(True)
    plt.margins(x=0)
    plt.tight_layout()
    matplot2tikz.save(output_path, axis_width="7cm", axis_height="5cm")
    plt.show()


def compute_area(graph: DynamicReachGraph, step_start: int, step_end: int) -> np.ndarray:
    """
    Computes the drivable area between two time steps using the reachability graph.

    Parameters
    ----------
    graph: DynamicReachGraph
        The reachability graph structure.

    step_start : int
        The starting time step.

    step_end : int
        The ending time step.

    Returns
    -------
    np.ndarray
        1D array of drivable area values for each time step.
    """

    areas = np.array([-1.0 for _ in range(step_end + 1)])

    for t in range(step_start, step_end + 1):
        try:
            # Get all nodes for the time step
            nodes = graph.get_nodes_at_step(t)
            if not nodes:
                print(f"Warning: No reachable nodes at step {t}")
                raise ValueError()

        except AttributeError:
            raise RuntimeError(f"Graph does not support time step access at t={t}")

        area = 0.0
        for node_ptr in nodes:
            if hasattr(node_ptr, "set"):
                node_set = node_ptr.set

                # Extract longitude and latitude bounds of the set
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

        # Check if the area is too small
        if area < 1e-5:
            print(f"Area is too small at step {t}: {area}")
            raise ValueError()
        areas[t] = area if area > 0 else 0.0

    return areas


def create_reach_graph(
    scenario_path: str, semantics: str = "true"
) -> Tuple[DynamicReachGraph, int, int, PlanningProblem, pycrccosy.CurvilinearCoordinateSystem]:
    """
    Loads a CommonRoad scenario, configures the reachability executor, and computes the reachability graph.

    Parameters
    ----------
    scenario_path : str
        Path to the XML file containing the CommonRoad scenario.

    semantics : str, optional
        Semantics of the scenario. Defaults to "true".

    Returns
    -------
    tuple
        A tuple containing:
        - graph (DynamicReachGraph): The reachability graph with reachable states.
        - step_start (int): The starting time step used for computation.
        - step_end (int): The ending time step used for computation.
        - planning_problem (PlanningProblem): The planning problem extracted from the scenario.
        - clcs (CurvilinearCoordinateSystem): Curvilinear coordinate system generated from the route.
    """

    dt = 0.2
    step_start = 0
    step_end = 20
    initial_uncertainty = 0.01

    # Define physical constraints for a point-mass vehicle model, including acceleration and velocity bounds
    point_mass_params = core.layers.propagation.PointMassParameters()
    # point_mass_params.a_lon_min = -9.5
    # point_mass_params.a_lon_max = 11.5
    # point_mass_params.a_lat_min = -2.0
    # point_mass_params.a_lat_max = 2.0
    point_mass_params.v_lon_min = 0.0
    # point_mass_params.v_lon_max = 50.8
    # point_mass_params.v_lat_min = -4.0
    # point_mass_params.v_lat_max = 4.0

    predicate_config = core.model_checking.PredicateConfiguration()
    # Used to inflate the vehicle shape when checking collisions.
    inflation_radius = (
        predicate_config.ego_width / 2
        if predicate_config.ego_width < predicate_config.ego_length
        else predicate_config.ego_length / 2
    )

    # Defines how to segment the road into lanelets
    splitter_params = core.layers.semantic.SemanticSplitterParameters()
    splitter_params.minimum_region_area = 0.01
    splitter_params.lanelet_inflation_radius = inflation_radius

    # Read the scenario
    scenario, planning_problems = CommonRoadFileReader(scenario_path).open()
    scenario, planning_problems = resample_scenario(scenario, planning_problems, dt)
    planning_problem = list(planning_problems.planning_problem_dict.values())[0]

    # Plan route through lanelets and create clcs
    route = route_planner.generate_reference_path_from_lanelet_network_and_planning_problem(
        scenario.lanelet_network, planning_problem
    )
    splitter_params.route_lanelet_ids = set(route.lanelet_ids)
    lanelet_ids = splitter_params.route_lanelet_ids
    # Resample the reference path at 2.0-meter intervals to ensure uniform spacing
    reference_path = pycrccosy.Util.resample_polyline(route.reference_path, 2.0)
    clcs = pycrccosy.CurvilinearCoordinateSystem(reference_path)
    # print(f"Route lanelet IDs: {lanelet_ids}")

    # create collision checker
    cc = CollisionCheckerFactory(step_start, step_end, inflation_radius).create_curvilinear_collision_checker(
        scenario, clcs
    )

    # Add semantics for vehicle to stay on the road
    lanelet_conditions = " | ".join(f"InLanelet_{lid}" for lid in lanelet_ids)
    specs = [f"G (({lanelet_conditions}) & ({semantics}))"]
    print(specs)

    world = World(scenario)

    # Create a finite automaton from the specifications
    automaton = core.model_checking.FiniteAutomaton(specs)
    init = core.initializers.base_set.CurvilinearUncertaintyInitializer(clcs, *([initial_uncertainty] * 4))
    # Define the core layers used in the reachability analysis pipeline
    layers = {
        "propagation": core.layers.propagation.PointMassPropagator(dt, point_mass_params),
        "splitting": core.layers.semantic.SemanticSplitter(automaton, world, clcs, splitter_params),
        "repartitioning": core.layers.meta.GroupedByAutomatonStates(core.layers.repartition.PositionRepartitioner()),
        "collision_checking": core.layers.collision.CollisionFilter(cc),
    }

    # Wrap each layer with a timing wrapper for performance measurement
    layers = {key: core.layers.meta.Timed(value) for key, value in layers.items()}

    # Combine the layers into a sequential processing pipeline
    layer = core.layers.meta.Sequential(
        [
            layers["propagation"],
            layers["splitting"],
            layers["repartitioning"],
            layers["collision_checking"],
            layers["repartitioning"],
        ]
    )

    # Define a sequence of post-processing steps to clean up the automaton:
    # 1. Remove states that do not satisfy the semantic final conditions.
    # 2. Prune unreachable nodes from the reachability graph.
    post = core.post_processors.meta.Sequential(
        [
            core.post_processors.pruning.SemanticFinalStatePruner(automaton),
            core.post_processors.pruning.DanglingNodePruner(),
        ]
    )
    # Manages the full pipeline of reachable set computation.
    rs = core.executors.DynamicReachExecutor(step_start, step_end, init, layer, post)

    # Initialization of the reachable sets from the planning problem
    rs.initialize(*initialize_from_planning_problem(planning_problem))

    # The actual computation of the reachable set
    rs.compute()

    # Create reachability graph
    graph = rs.get_post_processed_reach_graph()

    # Check if the graph contains any reachable nodes
    has_nodes = any(len(graph.get_nodes_at_step(t)) > 0 for t in range(step_start + 1, step_end + 1))
    if not has_nodes:
        raise RuntimeError("Reachability graph is empty – possibly due to invalid parameters.")

    return graph, step_start, step_end, planning_problem, clcs


def initialize_from_planning_problem(
    planning_problem: PlanningProblem,
) -> Tuple[int, ndarray, ndarray, float, float, float]:
    """
    Extracts the initial state from a PlanningProblem and formats it as a tuple
    of initialization parameters for an executor or simulator.

    Parameters
    ----------
    planning_problem : PlanningProblem
        The planning problem containing the initial state.

    Returns
    -------
    tuple[int, float, float, float, float, float]
        A tuple containing:
        - time_step (int)
        - position_x (float)
        - position_y (float)
        - velocity (float)
        - acceleration (float), always 0
        - orientation (float)
    """

    state = planning_problem.initial_state
    # the planning problem does not contain an acceleration, so we return 0
    return (
        state.time_step,
        state.position[0],
        state.position[1],
        state.velocity,
        0.0,
        state.orientation,
    )

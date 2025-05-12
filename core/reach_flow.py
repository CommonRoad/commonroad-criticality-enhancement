import time
from typing import Tuple

import commonroad_dc.pycrccosy as pycrccosy
import cr_reach_flow.cr_reach_flow_core as core
import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem
from commonroad_route_planner.route_planner import RoutePlanner
from cr_reach_flow.collision_checker.collision_checker_factory import CollisionCheckerFactory
from cr_reach_flow.scenario.resampling import resample_scenario
from cr_reach_flow.visualization.scenario import draw_with_reach_set
from matplotlib import pyplot as plt


def compute_drivable_area(scenario_path):
    graph, step_start, step_end, planning_problem, clcs = create_reach_graph(scenario_path)
    area = compute_area(graph, step_start, step_end)
    return area


def draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs):
    fig, ax = plt.subplots(figsize=(10, 6))
    draw_with_reach_set(step_end, scenario, planning_problem, graph, clcs, ax)
    plt.show()


def plot(area_original, area_modified, labels=("Original", "Optimized")):
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


def compute_area(graph, step_start, step_end):
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
        areas[t] = area

    print(areas)
    return areas


def create_reach_graph(scenario_path="scenarios/ZAM_Merge-1_1_T-1.xml"):
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
    reference_path = pycrccosy.Util.resample_polyline(route.reference_path, 2.0)
    clcs = pycrccosy.CurvilinearCoordinateSystem(reference_path)
    print(f"Route lanelet IDs: {splitter_params.route_lanelet_ids}")

    # create reach set executor
    cc = CollisionCheckerFactory(
        step_start, step_end, inflation_radius
    ).create_curvilinear_collision_checker(scenario, clcs)
    specs = ["true"]
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
    """Create arguments to initialize an executor from a planning problem."""
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

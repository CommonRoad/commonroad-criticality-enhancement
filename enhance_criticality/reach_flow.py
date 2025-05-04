import time
from typing import Tuple

import commonroad_dc.pycrccosy as pycrccosy
import cr_reach_flow.cr_reach_flow_core as core
import matplotlib.pyplot as plt
import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem
from commonroad_route_planner.route_planner import RoutePlanner
from cr_reach_flow.collision_checker.collision_checker_factory import CollisionCheckerFactory
from cr_reach_flow.scenario.resampling import resample_scenario
from cr_reach_flow.visualization.interactive import InteractiveVisualization
from cr_reach_flow.visualization.scenario import draw_with_regions, draw_with_slider
from matplotlib import pyplot as plt


def plot(profile1, profile2, labels=("Original", "Optimized")):
    # plt.plot(range(step_start, step_end + 1), area_profile[step_start:step_end + 1])
    steps = range(len(profile1))
    plt.figure(figsize=(10, 5))
    plt.plot(steps, profile1, label=labels[0])
    plt.plot(steps, profile2, label=labels[1])
    plt.xlabel("Time Step")
    plt.ylabel("Drivable Area")
    plt.title("Drivable Area Over Time")
    plt.grid(True)
    plt.show()


def compute_area(graph, reach_interface):
    areas = np.full((reach_interface.step_end + 1,), -1.0)
    for t in range(reach_interface.step_start, reach_interface.step_end + 1):
        nodes = graph.nodes_at_time_step(t)
        area = sum(
            (n.region.p_lon_max - n.region.p_lon_min) * (n.region.p_lat_max - n.region.p_lat_min)
            for n in nodes
        )
        areas[t] = area
    return areas


def create_reach_graph(scenario, planning_problem: PlanningProblem, reach_interface):
    print("Creating Reach Graph")
    step_start = reach_interface.step_start
    step_end = reach_interface.step_end
    dt = reach_interface.dt
    initial_uncertainty = reach_interface.initial_uncertainty

    # Define propagation dynamics
    point_mass_params = core.layers.propagation.PointMassParameters()
    point_mass_params.a_lon_min = -9.5
    point_mass_params.a_lon_max = 11.5
    point_mass_params.a_lat_min = -2.0
    point_mass_params.a_lat_max = 2.0
    point_mass_params.v_lon_min = 0.0
    point_mass_params.v_lon_max = 50.8
    point_mass_params.v_lat_min = -4.0
    point_mass_params.v_lat_max = 4.0

    # Compute inflation radius
    predicate_config = core.model_checking.PredicateConfiguration()
    inflation_radius = min(predicate_config.ego_width, predicate_config.ego_length) / 2

    # Create semantic splitter params
    splitter_params = core.layers.semantic.SemanticSplitterParameters()
    splitter_params.minimum_region_area = 0.01
    splitter_params.lanelet_inflation_radius = inflation_radius

    # plan route and create clcs
    route = RoutePlanner(scenario, planning_problem).plan_routes().retrieve_first_route()
    splitter_params.route_lanelet_ids = set(route.lanelet_ids)
    reference_path = pycrccosy.Util.resample_polyline(route.reference_path, 2.0)
    clcs = pycrccosy.CurvilinearCoordinateSystem(reference_path)

    # create reach set executor
    # cc = CollisionCheckerFactory(step_start, step_end, inflation_radius).create_curvilinear_collision_checker(
    #     scenario, clcs
    # )
    specs = ["true"]

    automaton = core.model_checking.FiniteAutomaton(specs)
    init = core.initializers.base_set.CurvilinearUncertaintyInitializer(
        clcs, *([initial_uncertainty] * 4)
    )
    layers = [
        core.layers.propagation.PointMassPropagator(dt, point_mass_params),
        core.layers.semantic.SemanticSplitter(automaton, None, dt, clcs, splitter_params),
        # core.layers.semantic.SemanticSplitter(automaton, scenario_path, dt, clcs, splitter_params),
        core.layers.meta.GroupedByAutomatonStates(core.layers.repartition.PositionRepartitioner()),
        # core.layers.collision.CollisionFilter(cc),
        core.layers.meta.GroupedByAutomatonStates(core.layers.repartition.PositionRepartitioner()),
    ]
    post = [
        core.post_processors.pruning.SemanticFinalStatePruner(automaton),
        core.post_processors.pruning.DanglingNodePruner(),
    ]
    draw_with_regions(
        step_start, scenario, planning_problem, layers[1].lanelet_regions, clcs, plt.gca()
    )
    plt.show()

    rs = core.executors.DynamicReachExecutor(
        init, core.layers.meta.Sequential(layers), core.post_processors.meta.Sequential(post)
    )

    tic = time.perf_counter()
    rs.initialize(*initialize_from_planning_problem(planning_problem))
    toc = time.perf_counter()
    print(f"Initialization took {toc - tic:3f} seconds")

    # compute reachable sets
    tic = time.perf_counter()
    rs.compute(step_start + 1, step_end)
    toc = time.perf_counter()
    print(f"Reachable set computation took {toc - tic:3f} seconds")

    # create reachability graph
    tic = time.perf_counter()
    graph = rs.get_post_processed_reach_graph()
    toc = time.perf_counter()
    print(f"Graph creation took {toc - tic:3f} seconds")

    # draw_with_slider(step_start, step_end, scenario, planning_problem, graph, clcs)

    # InteractiveVisualization(scenario, planning_problem, clcs).draw_interactive_reach_graph(graph)


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

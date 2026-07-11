# SPDX-License-Identifier: LGPL-3.0-or-later
import math
from pathlib import Path

import jupedsim as jps
import pytest

####################
# Utility functions
####################


def path_distance(points: list[tuple[float, float]]) -> float:
    """Calculate total Euclidean length of a waypoint list."""
    return sum(
        math.hypot(x2 - x1, y2 - y1)
        for (x1, y1), (x2, y2) in zip(points[:-1], points[1:])
    )


def load_wkt_file(filename: str):
    # Load file relative to repo root
    repo_root = Path(__file__).parents[1]
    wkt_path = repo_root / filename
    return wkt_path.read_text(encoding="utf-8")


########################
# End Utility functions
########################


def test_routing_engine_with_excluded_areas():
    """Verify excluded_areas kwarg is forwarded to build_geometry."""
    outer = [(0, 0), (100, 0), (100, 100), (0, 100)]
    hole = [(40, 40), (60, 40), (60, 60), (40, 60)]

    engine = jps.RoutingEngine(
        geometry=outer,
        excluded_areas=[hole],
    )
    assert engine is not None


def test_routing_engine_without_excluded_areas():
    outer = [(0, 0), (100, 0), (100, 100), (0, 100)]
    engine = jps.RoutingEngine(geometry=outer)
    assert engine is not None


def make_l_corridor_simulation(routing_engine=None, run_in_2d=True):
    """One agent at (1, 1) heading for an exit at the top of the vertical leg;
    its route bends around the inner corner (8, 2)."""
    sim = jps.Simulation(
        model=jps.CollisionFreeSpeedModel(),
        geometry=[(0, 0), (10, 0), (10, 10), (8, 10), (8, 2), (0, 2)],
        routing_engine=routing_engine,
        run_in_2d=run_in_2d,
    )
    exit_id = sim.add_exit_stage([(8, 9), (10, 9), (10, 10), (8, 10)])
    journey_id = sim.add_journey(jps.JourneyDescription([exit_id]))
    sim.add_agent(
        jps.CollisionFreeSpeedModelAgentParameters(
            journey_id=journey_id, stage_id=exit_id, position=(1, 1)
        )
    )
    return sim


@pytest.mark.parametrize("run_in_2d", [True, False], ids=["2d", "surface"])
@pytest.mark.parametrize(
    "routing_engine",
    [jps.TAStarRouting(), jps.SurfaceGeodesicRouting()],
    ids=lambda engine: type(engine).__name__,
)
def test_simulation_evacuates_l_corridor_with_either_routing(
    routing_engine, run_in_2d
):
    """Smoke test for the routing_engine and run_in_2d parameters: the agent
    has to round the inner corner (8, 2) and reach the exit in every
    combination. Engine selection and 2D/surface parity themselves are
    asserted in the C++ tests Simulation.UsesTheInjectedRoutingEngine and
    Simulation.SurfaceOperationalPathReproducesThe2DPath."""
    sim = make_l_corridor_simulation(routing_engine, run_in_2d)
    while sim.agent_count() > 0:
        assert sim.iteration_count() < 10_000, "agent never reached the exit"
        sim.iterate()


@pytest.mark.parametrize("agent_count", [1, 5], ids=["1_agent", "5_agents"])
def test_run_in_2d_flag_reproduces_trajectories_on_buw(agent_count):
    """2D/surface parity on a real floorplan (BUW: ~50x32 m, 40 obstacles).
    The route from the room pocket around (20, 24) to the exit at (42.5, 10)
    is 1.9x the direct distance with 14 bends; with 5 clustered agents the
    neighbor interaction is exercised on top. Identical TA* routing isolates
    the operational (gather/apply) axis. Runs until everyone reached the
    exit; positions must match at every step."""
    geometry = load_wkt_file("examples/geometry/BUW.wkt")
    starts = [
        (20.21, 23.91),
        (19.2, 23.9),
        (20.2, 22.9),
        (19.2, 22.9),
        (21.2, 23.4),
    ][:agent_count]

    def build(run_in_2d):
        sim = jps.Simulation(
            model=jps.CollisionFreeSpeedModel(),
            geometry=geometry,
            run_in_2d=run_in_2d,
        )
        exit_id = sim.add_exit_stage(
            [(41.5, 9.4), (43.5, 9.4), (43.5, 10.6), (41.5, 10.6)]
        )
        journey_id = sim.add_journey(jps.JourneyDescription([exit_id]))
        for start in starts:
            sim.add_agent(
                jps.CollisionFreeSpeedModelAgentParameters(
                    journey_id=journey_id, stage_id=exit_id, position=start
                )
            )
        return sim

    sim_2d = build(run_in_2d=True)
    sim_surface = build(run_in_2d=False)
    while sim_2d.agent_count() > 0:
        assert sim_2d.iteration_count() < 6_000, "agents never reached the exit"
        sim_2d.iterate()
        sim_surface.iterate()
        assert sim_surface.agent_count() == sim_2d.agent_count()
        for expected, actual in zip(sim_2d.agents(), sim_surface.agents()):
            assert actual.position == pytest.approx(expected.position, abs=1e-9)


def test_run_in_2d_flag_reproduces_trajectories():
    """The parity gate over the public API: identical routing, only the
    operational path differs -- the trajectories must match exactly."""
    sim_2d = make_l_corridor_simulation()
    sim_surface = make_l_corridor_simulation(run_in_2d=False)
    for _ in range(800):
        sim_2d.iterate()
        sim_surface.iterate()
        for expected, actual in zip(sim_2d.agents(), sim_surface.agents()):
            assert actual.position == pytest.approx(expected.position, abs=1e-9)


BAD_ASTAR_ROUTINGS = [
    {
        "test_name": "corner_with_shortcut",
        "description": "Same starting point, but end points differ just 0.2 on y axis, but total distance diff was >>0.2",
        "wkt_path": "examples/geometry/corner_with_shortcut.wkt",
        "error_type": "max_diff",
        "path1": [(11.43, 0.44), (27.93, 15.0)],
        "path2": [(11.43, 0.44), (27.93, 15.2)],
        "max_diff": 0.2,
    },
    {
        "test_name": "corner_with_shortcut2",
        "description": "Same starting point, but end points differ just 0.03 on y axis, but total distance diff was >>0.03",
        "wkt_path": "examples/geometry/corner_with_shortcut.wkt",
        "error_type": "max_diff",
        "path1": [(11.80, 1.00), (28.50, 13.54)],
        "path2": [(11.80, 1.00), (28.50, 13.55)],
        "max_diff": 0.01,
    },
    {
        "test_name": "aknz_evac",
        "description": "Direct path possible, but path was ways longer",
        "error_type": "direct path possible",
        "wkt_path": "examples/geometry/aknz_evac.wkt",
        "path": [(530.15, 1762.46), (530.15, 1774.66)],
    },
    {
        "test_name": "BUW",
        "description": "Direct path possible, but path was ways longer",
        "error_type": "direct path possible",
        "wkt_path": "examples/geometry/BUW.wkt",
        "path": [(12.57, 36.49), (22.0, 36.49)],
    },
    {
        "test_name": "SiB2023_entrance_jupedsim",
        "description": "Direct path possible, but path was ways longer",
        "error_type": "direct path possible",
        "wkt_path": "examples/geometry/SiB2023_entrance_jupedsim.wkt",
        "path": [(-1864.57, -83.91), (-1764.4, -116.18)],
        # Still needs some tolerance here, but it was ways worse beforehand
        "abs_diff_tolerance": 0.25,
    },
]


@pytest.mark.parametrize(
    "test_entry",
    [
        test_entry
        for test_entry in BAD_ASTAR_ROUTINGS
        if test_entry["error_type"] == "max_diff"
    ],
    ids=lambda params: params["test_name"],
)
def test_max_diff_example(test_entry):
    geometry = load_wkt_file(test_entry["wkt_path"])
    navi = jps.RoutingEngine(geometry)

    path1 = navi.compute_waypoints(
        test_entry["path1"][0], test_entry["path1"][1]
    )
    path2 = navi.compute_waypoints(
        test_entry["path2"][0], test_entry["path2"][1]
    )

    distance1 = path_distance(path1)
    distance2 = path_distance(path2)
    distance_diff = math.fabs(distance2 - distance1)
    assert distance_diff <= test_entry["max_diff"]


@pytest.mark.parametrize(
    "test_entry",
    [
        test_entry
        for test_entry in BAD_ASTAR_ROUTINGS
        if test_entry["error_type"] == "direct path possible"
    ],
    ids=lambda params: params["test_name"],
)
def test_direct_path_possible_example(test_entry):
    geometry = load_wkt_file(test_entry["wkt_path"])
    navi = jps.RoutingEngine(geometry)

    path = navi.compute_waypoints(*test_entry["path"])
    distance = path_distance(path)

    direct_distance = path_distance(test_entry["path"])
    abs_tolerance = test_entry.get("abs_diff_tolerance", 1e-12)
    rel_tolerance = test_entry.get("rel_diff_tolerance", 1e-6)
    assert distance == pytest.approx(
        direct_distance, abs=abs_tolerance, rel=rel_tolerance
    )

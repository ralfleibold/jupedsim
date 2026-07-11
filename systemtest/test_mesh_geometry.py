# SPDX-License-Identifier: LGPL-3.0-or-later
import jupedsim as jps
import pytest

# Two-storey geometry as one welded triangle mesh (the shape used by the C++
# multi-region tests): ground floor (x in [0,16], y in [0,6], z=0) -> ramp
# (x in [12,16], climbing y 6..12 to z=3) -> upper floor in a U whose last
# part (x in [6,12], y in [0,12], z=3) lies directly above the ground floor.
TWO_STOREY_OBJ = """
v 0 0 0
v 12 0 0
v 16 0 0
v 16 6 0
v 12 6 0
v 0 6 0
v 16 12 3
v 12 12 3
v 16 16 3
v 12 16 3
v 6 16 3
v 6 12 3
v 6 0 3
v 12 0 3
f 1 2 5
f 1 5 6
f 2 3 4
f 2 4 5
f 5 4 7
f 5 7 8
f 8 7 9
f 8 9 10
f 12 8 10
f 12 10 11
f 13 14 8
f 13 8 12
"""


@pytest.fixture
def two_storey_obj(tmp_path):
    path = tmp_path / "two_storey.obj"
    path.write_text(TWO_STOREY_OBJ, encoding="utf-8")
    return path


def test_simulation_evacuates_from_obj_geometry(two_storey_obj):
    """The OBJ input chain end-to-end: a Path builds a mesh geometry, the
    simulation defaults to the surface path and the geodesic engine, and an
    agent on the ground floor walks straight into the exit."""
    sim = jps.Simulation(
        model=jps.CollisionFreeSpeedModel(),
        geometry=two_storey_obj,
    )
    exit_id = sim.add_exit_stage([(0, 0), (2, 0), (2, 2), (0, 2)])
    journey_id = sim.add_journey(jps.JourneyDescription([exit_id]))
    sim.add_agent(
        jps.CollisionFreeSpeedModelAgentParameters(
            journey_id=journey_id, stage_id=exit_id, position=(4, 3)
        )
    )

    for _ in range(2000):
        if sim.agent_count() == 0:
            break
        sim.iterate()
    assert sim.agent_count() == 0


def test_mesh_geometry_rejects_the_2d_path(two_storey_obj):
    with pytest.raises(RuntimeError, match=r"no projected 2D view"):
        jps.Simulation(
            model=jps.CollisionFreeSpeedModel(),
            geometry=two_storey_obj,
            run_in_2d=True,
        )


def test_mesh_geometry_rejects_tastar_routing(two_storey_obj):
    with pytest.raises(ValueError, match=r"needs a polygon geometry"):
        jps.Simulation(
            model=jps.CollisionFreeSpeedModel(),
            geometry=two_storey_obj,
            routing_engine=jps.TAStarRouting(),
        )


def test_unreadable_obj_file_reports_the_path(tmp_path):
    path = tmp_path / "missing.obj"
    with pytest.raises(RuntimeError, match=r"missing\.obj"):
        jps.Simulation(
            model=jps.CollisionFreeSpeedModel(),
            geometry=path,
        )


def test_z_hint_selects_the_floor_for_agent_and_exit(two_storey_obj):
    """From Python, z_hint anchors the stage and the agent on the upper
    floor: an agent placed above the ground floor evacuates through an exit
    that overlaps the ground in (x,y) but is anchored one storey up."""
    sim = jps.Simulation(
        model=jps.CollisionFreeSpeedModel(),
        geometry=two_storey_obj,
    )
    # Exit footprint (x in [7,9], y in [0,2]) sits over both floors; the
    # z_hint pins it to the upper storey at z=3.
    exit_id = sim.add_exit_stage([(7, 0), (9, 0), (9, 2), (7, 2)], z_hint=3.0)
    journey_id = sim.add_journey(jps.JourneyDescription([exit_id]))
    sim.add_agent(
        jps.CollisionFreeSpeedModelAgentParameters(
            journey_id=journey_id, stage_id=exit_id, position=(8, 8)
        ),
        z_hint=3.0,
    )

    for _ in range(2000):
        if sim.agent_count() == 0:
            break
        sim.iterate()
    assert sim.agent_count() == 0


def test_z_hint_without_a_matching_sheet_raises(two_storey_obj):
    sim = jps.Simulation(
        model=jps.CollisionFreeSpeedModel(),
        geometry=two_storey_obj,
    )
    # (8,1) has sheets at z=0 and z=3; a hint of 1.5 matches neither within
    # the 0.25 m tolerance.
    exit_id = sim.add_exit_stage([(0, 0), (2, 0), (2, 2), (0, 2)])
    journey_id = sim.add_journey(jps.JourneyDescription([exit_id]))
    with pytest.raises(RuntimeError, match=r"not inside walkable area"):
        sim.add_agent(
            jps.CollisionFreeSpeedModelAgentParameters(
                journey_id=journey_id, stage_id=exit_id, position=(8, 1)
            ),
            z_hint=1.5,
        )

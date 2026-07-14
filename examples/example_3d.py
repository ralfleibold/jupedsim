from pathlib import Path

import jupedsim as jps
from simulation_viewer import SimulationViewer

OBJ = Path(__file__).parents[0] / "geometry/multi_level_u_stair.obj"

sim = jps.Simulation(model=jps.WarpDriverModel(), geometry=OBJ)

exit_id = sim.add_exit_stage(
    [(3.5, 12.5), (4.5, 12.5), (4.5, 13.5), (3.5, 13.5)],
    z_hint=3.0,
)

# Due to no wall clearance in 3D yet, we need to set intermediate waypoints on the plateau of the
# U-turn stairs.
waypoints = [
    ((13.38, 8.42), 0.2, 1.5),
    ((13.25, 10.05), 0.2, 1.5),
    ((7.5, 10.5), 0.2, 3.0),
]
waypoint_ids = [
    sim.add_waypoint_stage(pos, dist, z_hint=z) for pos, dist, z in waypoints
]

route = [*waypoint_ids, exit_id]
journey = jps.JourneyDescription(route)
for current, following in zip(route, route[1:]):
    journey.set_transition_for_stage(
        current, jps.Transition.create_fixed_transition(following)
    )
journey_id = sim.add_journey(journey)

start_positions = [
    (3.3, 7.6),
    (4.0, 7.6),
    (3.3, 8.3),
    (4.0, 8.3),
    (3.65, 6.9),
]
for pos in start_positions:
    sim.add_agent(
        jps.WarpDriverModelAgentParameters(
            journey_id=journey_id, stage_id=route[0], position=pos
        ),
        z_hint=0.0,
    )


def on_step(sim):
    # Runs on every iterate step. This is where one could add / retarget agents.
    # We don't in this example.
    pass


viewer = SimulationViewer(sim, on_step=on_step, geometry_obj=OBJ)
viewer.run()

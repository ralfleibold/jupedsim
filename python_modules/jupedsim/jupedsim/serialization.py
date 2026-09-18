# SPDX-License-Identifier: LGPL-3.0-or-later
"""Serialization/deserialization support

In this file you will find interfaces and implementations to serialize and
deserialize different forms of input / output commonly used.
"""

import abc


class TrajectoryWriter(metaclass=abc.ABCMeta):
    """Interface for trajectory serialization"""

    @abc.abstractmethod
    def begin_writing(self, simulation) -> None:
        """Begin writing trajectory data.

        This method is intended to handle all data writing that has to be done
        once before the trajectory data can be written. E.g. Meta information
        such as frame rate etc...

        """
        raise NotImplementedError

    @abc.abstractmethod
    def write_iteration_state(self, simulation) -> None:
        """Write trajectory data of one simulation iteration.

        This method is intended to handle serialization of the trajectory data
        of a single iteration.

        """
        raise NotImplementedError

    @abc.abstractmethod
    def every_nth_frame(self) -> int:
        """Returns the interval of this writer in frames between writes.

        1 indicates all frames are written, 10 indicates every 10th frame is
        writen and so on.

        Returns:
            Number of frames between writes as int

        """

    class Exception(Exception):
        """Represents exceptions specific to the trajectory writer."""

        pass


def walkable_area_as_wkt(simulation) -> str:
    """The simulation's walkable area as WKT, for a writer's header.

    This only works for a geometry built from exactly 1 region, not from a
    surface mesh.

    Raises:
        TrajectoryWriter.Exception: if the geometry has more than 1 region or
            was built from a surface mesh.
    """
    polygon = simulation.get_geometry().polygon()
    if polygon is None:
        raise TrajectoryWriter.Exception(
            "Cannot write trajectories for a simulation with a geometry "
            "containing multiple regions or built from a surface "
            "mesh: the trajectory formats JuPedSim ships are 2D so far. "
            "Build the simulation from a polygon, or "
            "pass a trajectory_writer of your own."
        )
    return polygon.as_wkt()

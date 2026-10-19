# trame_viewer

## Install & run

You need a python setup with all of jupedsims dependencies.

`jupedsim` is intentionally **not** a declared dependency, ensure `jupedsim` is
built and `environemnt` from the build folder is sourced before trying to run
`trame_viewer`.

Development flow (shared venv + build tree):

```sh
source <build-dir>/environment               # PYTHONPATH: jupedsim + built bindings
pip install -e python_modules/trame_viewer   # from the checkout root, once

trame_viewer [--obj mesh.obj | --trajectory traj.h5] [--native] [--port N]
```

`--trajectory` plays back an HDF5 file written by `jps.Hdf5TrajectoryWriter`; the
geometry comes from the file as well. Pathfinding and Boundary Highlight need a
`jps.Geometry` and are only available for OBJ meshes.

`python -m trame_viewer` works as well.

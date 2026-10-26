"""PCT planner pybind11 extensions (a_star, ele_planner, traj_opt, py_map_manager).

The extensions are built against the bundled pybind11 2.11.0.dev1, which
predates NumPy 2 support. Under NumPy 2 the first matrix argument passed to
an extension crashes the interpreter, so fail early with a clear message.
"""

import numpy as _np

if int(_np.__version__.split('.')[0]) >= 2:
    raise ImportError(
        'PCT planner extensions require NumPy < 2 (found NumPy {}); the '
        'bundled pybind11 2.11.0.dev1 does not support NumPy 2. Use the '
        'Ubuntu 22.04 / ROS 2 Humble python3-numpy or pip install '
        '"numpy<2".'.format(_np.__version__)
    )

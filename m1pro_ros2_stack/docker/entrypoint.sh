#!/bin/bash
set -e

# Keep ROS Jazzy binary modules (e.g. cv_bridge) compatible with NumPy ABI.
if python3 - <<'PY'
import sys
try:
    import numpy as np
except Exception:
    sys.exit(1)

major = int(np.__version__.split('.')[0])
sys.exit(0 if major < 2 else 1)
PY
then
    :
else
    echo "[entrypoint] NumPy 2.x detected; installing NumPy 1.26.x for ROS compatibility..."
    pip3 install --no-cache-dir --break-system-packages "numpy>=1.26,<2.0"
fi

source /opt/ros/jazzy/setup.bash
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
exec "$@"

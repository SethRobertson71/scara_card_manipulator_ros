#!/bin/sh
# generated from ament_package/template/environment_hook/ament_prefix_path.sh.em
if [ -z "$AMENT_PREFIX_PATH" ]; then
  export AMENT_PREFIX_PATH="@(prefix)"
else
  export AMENT_PREFIX_PATH="@(prefix):$AMENT_PREFIX_PATH"
fi

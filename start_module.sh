#!/bin/bash

gnome-terminal \
--tab --title="sim_driver" --command='bash -c "source install/setup.bash; source install/setup_env.bash; ros2 run sim_driver sim_driver; exec bash"' \
--tab --title="my_brain" --command='bash -c "source install/setup.bash; ros2 run my_brain my_brain; exec bash"' \
--tab --title="grasp_object" --command='bash -c "source install/setup.bash; ros2 run grasp_object grasp_object_client; exec bash"' \
--tab --title="graspnet_generator" --command='bash -c "source install/setup.bash; ros2 run graspnet_generator graspnet_generator; exec bash"' \
--tab --title="gongga_core" --command='bash -c "source install/setup.bash; ros2 run gongga_core gongga_driver; exec bash"' \
--tab --title="motion_generator" --command='bash -c "source install/setup.bash; source install/setup_env.bash; ros2 run motion_generator stable_motion_generator; exec bash"'


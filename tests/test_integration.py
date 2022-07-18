import carla_source_node
import control_node
import obstacle_location_node
import perfect_detection_node
import pid_control_node
import planning_node

inputs = {}
inputs = carla_source_node.dora_run()
inputs.update(perfect_detection_node.dora_run(inputs))
inputs.update(obstacle_location_node.dora_run(inputs))
inputs.update(planning_node.dora_run(inputs))
inputs.update(pid_control_node.dora_run(inputs))
inputs.update(control_node.dora_run(inputs))

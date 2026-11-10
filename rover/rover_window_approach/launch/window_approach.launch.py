from rover_configuration.launch import node_launch


def generate_launch_description():
    return node_launch(
        'rover_window_approach', 'window_approach_node', 'window_approach_node', 'window_approach.yaml',
    )

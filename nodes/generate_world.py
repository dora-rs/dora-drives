import random
import time

from carla import Client, Location, Rotation, Transform, command

IMAGE_WIDTH = 800
IMAGE_HEIGHT = 600

sensor_transform = Transform(
    Location(3, 0, 1), Rotation(pitch=0, yaw=0, roll=0)
)


def add_lidar(world, transform, callback, vehicle):
    lidar_blueprint = world.get_blueprint_library().find(
        "sensor.lidar.ray_cast"
    )
    lidar_blueprint.set_attribute("channels", "32")
    lidar_blueprint.set_attribute("range", "5000")
    lidar_blueprint.set_attribute("points_per_second", "500000")
    lidar_blueprint.set_attribute("rotation_frequency", "20")
    lidar_blueprint.set_attribute("upper_fov", "15")
    lidar_blueprint.set_attribute("lower_fov", "-30")
    lidar = world.spawn_actor(lidar_blueprint, transform, attach_to=vehicle)
    # Register callback to be invoked when a new point cloud is received.
    lidar.listen(callback)
    return lidar


def add_depth_camera(world, transform, callback, vehicle):
    depth_blueprint = world.get_blueprint_library().find("sensor.camera.depth")
    depth_blueprint.set_attribute("image_size_x", "800")
    depth_blueprint.set_attribute("image_size_y", "600")
    depth_blueprint.set_attribute("fov", str(90.0))
    depth_camera = world.spawn_actor(
        depth_blueprint, transform, attach_to=vehicle
    )
    # Register callback to be invoked when a new frame is received.
    depth_camera.listen(callback)
    return depth_camera


def add_camera(world, transform, callback, vehicle):
    camera_blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
    camera_blueprint.set_attribute("image_size_x", "800")
    camera_blueprint.set_attribute("image_size_y", "600")
    camera = world.spawn_actor(camera_blueprint, transform, attach_to=vehicle)
    # Register callback to be invoked when a new frame is received.
    camera.listen(callback)
    return camera


def add_segmented_camera(world, transform, callback, vehicle):
    segmented_blueprint = world.get_blueprint_library().find(
        "sensor.camera.semantic_segmentation"
    )
    segmented_blueprint.set_attribute("image_size_x", str(IMAGE_WIDTH))
    segmented_blueprint.set_attribute("image_size_y", str(IMAGE_HEIGHT))
    segmented_blueprint.set_attribute("fov", str(90.0))
    segmented_camera = world.spawn_actor(
        segmented_blueprint, transform, attach_to=vehicle
    )
    segmented_camera.listen(callback)
    return segmented_camera


def spawn_driving_vehicle(client, world):
    """This function spawns the driving vehicle and puts it into
    an autopilot mode.
    Args:
        client: The Client instance representing the simulation to
          connect to.
        world: The world inside the current simulation.
    Returns:
        A Actor instance representing the vehicle that was just spawned.
    """
    # Get the blueprint of the vehicle and set it to AutoPilot.
    vehicle_bp = random.choice(
        world.get_blueprint_library().filter("vehicle.*")
    )
    while (
        not vehicle_bp.has_attribute("number_of_wheels")
        or int(vehicle_bp.get_attribute("number_of_wheels")) != 4
    ):
        vehicle_bp = random.choice(
            world.get_blueprint_library().filter("vehicle.*")
        )
    # vehicle_bp.set_attribute("role_name", "autopilot")

    # Get the spawn point of the vehicle.
    start_pose = random.choice(world.get_map().get_spawn_points())

    # Spawn the vehicle.
    batch = [
        command.SpawnActor(vehicle_bp, start_pose).then(
            command.SetAutopilot(command.FutureActor, False)
        )
    ]

    global vehicle_id
    vehicle_id = client.apply_batch_sync(batch)[0].actor_id
    while world.get_actors().find(vehicle_id) is None:

        # Find the vehicle and return the Actor instance.
        time.sleep(
            5
        )  # This is so that the vehicle gets registered in the actors.
        print("waiting for ego vehicle to create")
    return world.get_actors().find(vehicle_id)


def check_simulator_version(
    simulator_version: str,
    required_major: int = 0,
    required_minor: int = 9,
    required_patch: int = 1,
):
    """Checks if the simulator meets the minimum version requirements."""
    ver_strs = simulator_version.split(".")
    if len(ver_strs) < 2 or len(ver_strs) > 3:
        print(
            "WARNING: Assuming that installed CARLA {} API is compatible "
            "with CARLA 0.9.10 API".format(simulator_version)
        )
        ver_strs = "0.9.10".split(".")
    major = int(ver_strs[0])
    minor = int(ver_strs[1])
    if major != required_major:
        return major > required_major
    else:
        if minor != required_minor:
            return minor > required_minor
        else:
            if len(ver_strs) == 3:
                patch = int(ver_strs[2])
                return patch >= required_patch
            else:
                return True


def spawn_actors(
    client,
    world,
    traffic_manager_port: int,
    simulator_version: str,
    ego_spawn_point_index: int,
    auto_pilot: bool,
    num_people: int,
    num_vehicles: int,
    logger,
):
    vehicle_ids = spawn_vehicles(
        client, world, traffic_manager_port, num_vehicles, logger
    )
    ego_vehicle = spawn_ego_vehicle(
        world, traffic_manager_port, ego_spawn_point_index, auto_pilot
    )
    people = []

    if check_simulator_version(
        simulator_version, required_minor=9, required_patch=6
    ):
        # People do not move in versions older than 0.9.6.
        (people, people_control_ids) = spawn_people(
            client, world, num_people, logger
        )
        people_actors = world.get_actors(people_control_ids)
        for i, ped_control_id in enumerate(people_control_ids):
            # Start person.
            people_actors[i].start()
            people_actors[i].go_to_location(
                world.get_random_location_from_navigation()
            )
    return ego_vehicle, vehicle_ids, people


def spawn_ego_vehicle(
    world,
    traffic_manager_port: int,
    spawn_point_index: int,
    auto_pilot: bool,
    blueprint: str = "vehicle.lincoln.mkz2017",
):
    v_blueprint = world.get_blueprint_library().filter(blueprint)[0]
    ego_vehicle = None
    while not ego_vehicle:
        if spawn_point_index == -1:
            # Pick a random spawn point.
            start_pose = random.choice(world.get_map().get_spawn_points())
        else:
            spawn_points = world.get_map().get_spawn_points()
            assert spawn_point_index < len(spawn_points), (
                "Spawn point index is too big. "
                "Town does not have sufficient spawn points."
            )
            start_pose = spawn_points[spawn_point_index]

        ego_vehicle = world.try_spawn_actor(v_blueprint, start_pose)
    if auto_pilot:
        ego_vehicle.set_autopilot(True, traffic_manager_port)
    return ego_vehicle


def spawn_people(client, world, num_people: int, logger):
    """Spawns people at random locations inside the world.

    Args:
        num_people: The number of people to spawn.
    """
    from carla import Transform, command

    p_blueprints = world.get_blueprint_library().filter("walker.pedestrian.*")
    unique_locs = set([])
    spawn_points = []
    # Get unique spawn points.
    for i in range(num_people):
        attempt = 0
        while attempt < 10:
            spawn_point = Transform()
            loc = world.get_random_location_from_navigation()
            if loc is not None:
                # Transform to tuple so that location is comparable.
                p_loc = (loc.x, loc.y, loc.z)
                if p_loc not in unique_locs:
                    spawn_point.location = loc
                    spawn_points.append(spawn_point)
                    unique_locs.add(p_loc)
                    break
            attempt += 1
        if attempt == 10:
            logger.error("Could not find unique person spawn point")
    # Spawn the people.
    batch = []
    for spawn_point in spawn_points:
        p_blueprint = random.choice(p_blueprints)
        if p_blueprint.has_attribute("is_invincible"):
            p_blueprint.set_attribute("is_invincible", "false")
        batch.append(command.SpawnActor(p_blueprint, spawn_point))
    # Apply the batch and retrieve the identifiers.
    ped_ids = []
    for response in client.apply_batch_sync(batch, True):
        if response.error:
            logger.info(
                "Received an error while spawning a person: {}".format(
                    response.error
                )
            )
        else:
            ped_ids.append(response.actor_id)
    # Spawn the person controllers
    ped_controller_bp = world.get_blueprint_library().find(
        "controller.ai.walker"
    )
    batch = []
    for ped_id in ped_ids:
        batch.append(command.SpawnActor(ped_controller_bp, Transform(), ped_id))
    ped_control_ids = []
    for response in client.apply_batch_sync(batch, True):
        if response.error:
            logger.info(
                "Error while spawning a person controller: {}".format(
                    response.error
                )
            )
        else:
            ped_control_ids.append(response.actor_id)

    return (ped_ids, ped_control_ids)


def spawn_vehicles(
    client, world, traffic_manager_port: int, num_vehicles: int, logger
):
    """Spawns vehicles at random locations inside the world.

    Args:
        num_vehicles: The number of vehicles to spawn.
    """
    from carla import command

    logger.debug("Trying to spawn {} vehicles.".format(num_vehicles))
    # Get the spawn points and ensure that the number of vehicles
    # requested are less than the number of spawn points.
    spawn_points = world.get_map().get_spawn_points()
    if num_vehicles >= len(spawn_points):
        logger.warning(
            "Requested {} vehicles but only found {} spawn points".format(
                num_vehicles, len(spawn_points)
            )
        )
        num_vehicles = len(spawn_points)
    else:
        random.shuffle(spawn_points)

    # Get all the possible vehicle blueprints inside the world.
    v_blueprints = world.get_blueprint_library().filter("vehicle.*")

    # Construct a batch message that spawns the vehicles.
    batch = []
    for transform in spawn_points[:num_vehicles]:
        blueprint = random.choice(v_blueprints)

        # Change the color of the vehicle.
        if blueprint.has_attribute("color"):
            color = random.choice(
                blueprint.get_attribute("color").recommended_values
            )
            blueprint.set_attribute("color", color)

        # Let the vehicle drive itself.
        blueprint.set_attribute("role_name", "autopilot")

        batch.append(
            command.SpawnActor(blueprint, transform).then(
                command.SetAutopilot(
                    command.FutureActor, True, traffic_manager_port
                )
            )
        )

    # Apply the batch and retrieve the identifiers.
    vehicle_ids = []
    for response in client.apply_batch_sync(batch, True):
        if response.error:
            logger.info(
                "Received an error while spawning a vehicle: {}".format(
                    response.error
                )
            )
        else:
            vehicle_ids.append(response.actor_id)
    return vehicle_ids

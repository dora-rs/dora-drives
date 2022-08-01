def get_world(host: str = "localhost", port: int = 2000, timeout: int = 10):
    """Get a handle to the world running inside the simulation.

    Args:
        host (:obj:`str`): The host where the simulator is running.
        port (:obj:`int`): The port to connect to at the given host.
        timeout (:obj:`int`): The timeout of the connection (in seconds).

    Returns:
        A tuple of `(client, world)` where the `client` is a connection to the
        simulator and `world` is a handle to the world running inside the
        simulation at the host:port.
    """
    try:
        from carla import Client

        client = Client(host, port)
        client_version = client.get_client_version()
        server_version = client.get_server_version()
        err_msg = "Simulator client {} does not match server {}".format(
            client_version, server_version
        )
        assert client_version == server_version, err_msg
        client.set_timeout(timeout)
        world = client.get_world()
    except RuntimeError as r:
        raise Exception(
            "Received an error while connecting to the "
            "simulator: {}".format(r)
        )
    except ImportError:
        raise Exception("Error importing CARLA.")
    return (client, world)


def get_map(host: str = "localhost", port: int = 2000, timeout: int = 10):
    """Get a handle to the CARLA map.

    Args:
        host (:obj:`str`): The host where the simulator is running.
        port (:obj:`int`): The port to connect to at the given host.
        timeout (:obj:`int`): The timeout of the connection (in seconds).

    Returns:
        carla.Map: A map of the CARLA town.
    """
    _, world = get_world(host, port, timeout)
    return world.get_map()

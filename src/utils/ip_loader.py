import socket

from src.config import config


def is_valid_ipv4(ip: str) -> bool:
    """Checks if the given IP address is a valid IPv4 address.

    :param: The IP address to check.
    :return: bool: True if the IP address is valid, otherwise False.
    """
    try:
        socket.inet_pton(socket.AF_INET, ip)
        return True
    except OSError:
        return False


def get_ip() -> str:
    """Gets the local IP address of the machine.

    :return: The local IPv4 address of the machine if valid, otherwise the root URL from the config.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    ip = s.getsockname()[0]
    s.close()

    if is_valid_ipv4(ip):
        return ip

    return config.telemetry.server.root_url

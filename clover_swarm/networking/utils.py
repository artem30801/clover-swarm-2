import asyncio
import ipaddress
import socket
from asyncio import get_running_loop
from typing import Optional, cast

import anyio
import attr
import psutil
from anyio._backends._asyncio import DatagramProtocol, UDPSocket
from anyio.abc import SocketAttribute

name_factory = attr.Factory(lambda self: str(self.uuid)[:6], takes_self=True)


async def create_broadcast_socket(
    local_port: int,
    local_host: Optional[str] = None,
    reuse_addr: bool = True,
    family: socket.AddressFamily = socket.AF_INET,
) -> UDPSocket:
    """
    Alternative of [anyio.create_connected_udp_socket][] to create UDP broadcast sockets.

    Args:
        local_port: 12
        local_host: Broadcast address of desired network interface. By default will try bind to "0.0.0.0" or "::"
        reuse_addr: Enables [socket.SO_REUSEADDR](https://docs.python.org/3.9/library/socket.html#socket.SOMAXCONN) for created socket (allows more than one socket to connect to chosen port)
        family: AddressFamily of the socket ([socket.AF_INET][] and [socket.AF_INET6][] are supported)

    Returns:
        Connected [anyio.abc.UDPSocket][] with ability to send and receive *broadcast* messages on chosen interface

    """

    if local_host is None:
        local_host = "0.0.0.0" if family == socket.AF_INET else "::"

    sock = socket.socket(family, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

    if reuse_addr:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    sock.bind((local_host, local_port))

    result = await get_running_loop().create_datagram_endpoint(DatagramProtocol, sock=sock)

    transport = cast(asyncio.DatagramTransport, result[0])
    protocol = cast(DatagramProtocol, result[1])
    if protocol.exception:
        transport.close()
        raise protocol.exception

    return UDPSocket(transport, protocol)


async def get_ip(remote_host: str = "8.8.8.8", remote_port: int = 80) -> str:
    """
    Returns IPv4 address of this computer in local network by trying to probe a remote host (works even if it's unreachable)

    For more information about your local connection or interface use [clover_swarm.networking.utils.get_active_interface][]

    Args:
        remote_host: Remote host to probe
        remote_port: Port of the remote host to probe

    Returns:
        IPv4 address of this computer in local network
    """
    sock = await anyio.create_connected_udp_socket(remote_host=remote_host, remote_port=remote_port)
    async with sock:
        ip = sock.extra(SocketAttribute.local_address)[0]
        return ip


def _filter_interfaces(my_ip, interface_stats):
    for interface, (addresses, _) in interface_stats.items():
        for snicaddr in addresses:
            if snicaddr.family == socket.AF_INET and snicaddr.address == my_ip:
                return snicaddr

    return None


async def get_active_interface(remote_host: str = "8.8.8.8", remote_port: int = 80) -> ipaddress.IPv4Interface:
    """
    Returns IPv4 address of primary IPv4 Interface used to communicate with local network

    This function uses [clover_swarm.networking.utils.get_ip][] function internally

    Args:
        remote_host: Remote host to probe
        remote_port: Port of the remote host to probe

    Raises:
        RuntimeError: There is no interface matching local IP

    Returns:
        Primary IPv4 Interface used to communicate with local network
    """

    my_ip = await get_ip(remote_host=remote_host, remote_port=remote_port)
    interface_stats = await anyio.to_thread.run_sync(get_interface_stats)

    snicaddr = _filter_interfaces(my_ip, interface_stats)
    if snicaddr is None:
        # return None?
        raise RuntimeError("No matching interfaces found")

    return ipaddress.IPv4Interface(f"{snicaddr.address}/{snicaddr.netmask}")


def get_interface_stats():
    addresses = psutil.net_if_addrs()
    stats = psutil.net_if_stats()

    interface_stats = {interface: (addrs, stats[interface]) for interface, addrs in addresses.items()}
    return interface_stats


def interface_is_active(interface, interface_stats=None):
    interface_stats = interface_stats or get_interface_stats()
    try:
        addresses, stats = interface_stats[interface]
    except KeyError:
        return False

    if not stats.isup:
        return False

    return socket.AF_INET in [snicaddr.family for snicaddr in addresses]


def get_active_interfaces(interface_stats=None):
    interface_stats = interface_stats or get_interface_stats()
    return [interface for interface in interface_stats.keys() if interface_is_active(interface, interface_stats)]


if __name__ == "__main__":

    async def main():
        print(list(get_interface_stats()))
        print(get_active_interfaces(), sep="\n")
        print(await get_ip())
        interface = await get_active_interface()
        print(interface.ip)
        print(interface.network.broadcast_address)

    # logging.basicConfig(level=logging.DEBUG)

    asyncio.run(main())

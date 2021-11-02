import socket
from typing import Optional, cast

import asyncio
from asyncio import get_running_loop

import anyio
from anyio.abc import SocketAttribute
from anyio._backends._asyncio import UDPSocket, DatagramProtocol

import psutil
import ipaddress


async def create_broadcast_socket(
    family: socket.AddressFamily,
    local_port: int,
    local_host: Optional[str] = None,
    reuse_addr: bool = True,
) -> UDPSocket:

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


async def get_ip(remote_host="8.8.8.8", remote_port=80):
    sock = await anyio.create_connected_udp_socket(remote_host=remote_host, remote_port=remote_port)
    ip = sock.extra(SocketAttribute.local_address)[0]
    return ip


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
    return [interface for interface in interface_stats.keys()
            if interface_is_active(interface, interface_stats)]


def _filter_interfaces(my_ip, interface_stats):
    for interface, (addresses, _) in interface_stats.items():
        for snicaddr in addresses:
            if snicaddr.family == socket.AF_INET and snicaddr.address == my_ip:
                return snicaddr


async def get_active_interface() -> ipaddress.IPv4Interface:
    my_ip = await get_ip()
    interface_stats = await anyio.to_thread.run_sync(get_interface_stats)

    # interfaces = get_active_interfaces(interface_stats)
    snicaddr = _filter_interfaces(my_ip, interface_stats)
    if snicaddr is None:
        # return None?
        raise RuntimeError("No interfaces found")

    return ipaddress.IPv4Interface(f"{snicaddr.address}/{snicaddr.netmask}")


if __name__ == '__main__':
    async def main():
        print(list(get_interface_stats()))
        print(get_active_interfaces(), sep="\n")
        print(await get_ip())
        interface = await get_active_interface()
        print(interface)
        print(interface.network.broadcast_address)
    # logging.basicConfig(level=logging.DEBUG)

    # asyncio.run(main())
    asyncio.get_event_loop().run_until_complete(main())


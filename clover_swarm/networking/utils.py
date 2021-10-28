import socket
from typing import Optional, cast

import asyncio
from asyncio import get_running_loop

from anyio._backends._asyncio import UDPSocket, DatagramProtocol


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

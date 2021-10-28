import logging
import socket
import struct
import typing
import uuid

import asyncio
import anyio
from anyio.abc import SocketAttribute
import attr

from clover_swarm.utils.clock import Clock, Rate
from clover_swarm.utils.signal import Signal
from clover_swarm.networking.utils import create_broadcast_socket

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from clover_swarm.networking.agent import Agent


logger = logging.getLogger(__name__)


@attr.define(kw_only=True)
class Beacon:
    port: int = attr.field(default=19700)
    addr: str = attr.field(default="255.255.255.255")
    send_interval: float = attr.field(default=1)

    running: bool = attr.field(default=False, init=False, repr=False)

    on_message: Signal = attr.field(factory=Signal, repr=False)
    on_receive: Signal = attr.field(factory=Signal, repr=False)
    on_send: Signal = attr.field(factory=Signal, repr=False)

    _clock: "Clock" = attr.field(factory=Clock, repr=False)
    _socket: anyio.abc.UDPSocket = attr.field(init=False, repr=False)

    _listener: asyncio.Future = attr.field(init=False, repr=False, default=None)
    _sender: asyncio.Future = attr.field(init=False, repr=False, default=None)

    def encode_message(self) -> bytes:
        ...

    def decode_message(self, message: bytes):
        ...

    async def start(
        self,
        start_sender: bool = True,
        start_listener: bool = True,
    ):
        if self.running:
            raise RuntimeError("Beacon already running")

        logger.debug(f"{self} starting on {self.addr}:{self.port}")

        # todo think about replacing 0.0.0.0 with looping over all interfaces
        self._socket = await create_broadcast_socket(socket.AF_INET, local_port=self.port, reuse_addr=True)

        if start_sender:
            self._sender = asyncio.ensure_future(self._start_sender())

        if start_listener:
            self._listener = asyncio.ensure_future(self._start_listener())
        # await get_running_loop().create_datagram_endpoint
        self.running = True

    @property
    def _running_tasks(self):
        return [task for task in (self._sender, self._listener) if task is not None]

    async def stop(self):
        logger.debug(f"{self} stopping")
        for task in self._running_tasks:
            task.cancel()
        await self.stopped
        logger.debug(f"{self} stopped")

    @property
    def stopped(self):
        if not self.running:
            raise RuntimeError("Beacon is not running yet!")

        return asyncio.gather(*self._running_tasks, return_exceptions=True)

    async def _start_sender(self):
        logger.info(f"{self} sender starting with interval {self.send_interval}")
        rate = self._clock.make_rate(delay=self.send_interval)

        try:
            while True:
                await self._send_beacon()
                await rate.sleep()
        except asyncio.CancelledError:
            logger.info(f"{self} sender stopped")
            raise

    async def _send_beacon(self):
        message = self.encode_message()
        await self._socket.sendto(message, self.addr, self.port)
        logger.debug(f"{self} sent broadcast message {message} to {self.addr}:{self.port}")
        await self.on_send.emit(message)
        return message

    async def _start_listener(self):
        logger.info(f"{self} listener starting")
        try:
            while True:
                await self._receive_beacon()
        except asyncio.CancelledError:
            logger.info(f"{self} listener stopped")
            raise

    async def _receive_beacon(self):
        message, (addr, port) = await self._socket.receive()
        logger.debug(f"{self} received broadcast message {message} from {addr}:{port}")
        await self.on_receive.emit(message, addr=addr, port=port)
        data = self.decode_message(message)
        await self.on_message.emit(data, addr=addr, port=port)
        return data, addr, port


@attr.define(kw_only=True)
class MessageBeacon(Beacon):
    message: str = attr.field()

    def encode_message(self) -> bytes:
        return str.encode(self.message)

    def decode_message(self, message: bytes):
        return message.decode()


@attr.define(kw_only=True)
class AgentBeacon(Beacon):
    agent: "Agent" = attr.field()

    on_peer: Signal = attr.field(factory=Signal, repr=False)

    version: int = attr.field(default=1, repr=False)  # unsigned short
    prefix: bytes = attr.field(default=b"CS", repr=False)  # Clover Swarm
    struct_format: str = attr.field(default=">2sH16sH", repr=False)  # 2 identifier bytes, 16 UUID bytes, unsigned short (port)
    # message_len: int = attr.field(default=32, repr=False)

    def encode_message(self) -> bytes:
        packed = struct.pack(
            self.struct_format,
            self.prefix,
            self.version,
            self.agent.uuid.bytes,
            self.agent.port,
        )
        return packed

    def decode_message(self, message: bytes):
        # todo error processing for wrong beacons
        prefix, version, peer_uuid, peer_port = struct.unpack(self.struct_format, message)
        if prefix != self.prefix or version != self.version:
            pass

        return uuid.UUID(bytes=peer_uuid), peer_port

    async def _receive_beacon(self):
        data, addr, port = await super()._receive_beacon()
        peer_uuid, peer_port = data
        await self._process_peer(peer_uuid, addr, peer_port)

    async def _process_peer(self, peer_uuid, peer_addr, peer_port):
        if peer_uuid == self.agent.uuid:
            return

        await self.on_peer.emit(peer_uuid, peer_addr, peer_port)
        logger.info(f"{self} detected new peer {peer_uuid} at {peer_addr}:{peer_port}")


if __name__ == '__main__':
    async def main():
        beacon = MessageBeacon(message="Hello world")
        await beacon.start()
        await anyio.sleep(3)
        await beacon.stop()

    logging.basicConfig(level=logging.DEBUG)

    # asyncio.run(main())
    asyncio.get_event_loop().run_until_complete(main())


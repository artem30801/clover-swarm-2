import asyncio
import ipaddress
import logging
import socket
import struct
import uuid
from abc import ABC
from contextlib import AsyncExitStack
from types import TracebackType
from typing import TYPE_CHECKING, Any, NoReturn, Optional, Tuple, Type

import anyio
import attr
from anyio.abc import SocketAttribute

from clover_swarm.networking.utils import create_broadcast_socket, get_active_interface
from clover_swarm.utils.attr_utils import add_docs
from clover_swarm.utils.callback_signal import Signal
from clover_swarm.utils.clock import Clock, Rate

if TYPE_CHECKING:
    from clover_swarm.networking.agent import Agent


logger = logging.getLogger(__name__)


@attr.define(kw_only=True)
class Beacon(ABC):
    """
    Base abstract class for beacons: UDP broadcast autodiscovery and message transmission objects
    """

    # config
    port: int = attr.field(default=19700)
    add_docs(port, "Port used for broadcast sending and receiving")

    interface: ipaddress.IPv4Interface = attr.field(default=None)
    add_docs(interface, "Interface used for broadcast sending and receiving")

    send_interval: float = attr.field(default=5)
    add_docs(send_interval, "Interval between sending broadcast messages (in seconds)")

    send: bool = attr.field(default=True)
    add_docs(send, "Whether to send broadcast messages")

    listen: bool = attr.field(default=True)
    add_docs(listen, "Whether to listen for broadcast messages")

    # public runtime attributes
    running: bool = attr.field(
        default=False,
        init=False,
        repr=False,
    )
    add_docs(running, "Indicates whether this beacon is currently running")

    # public signals

    # TODO USE event dispatcher obj

    on_message: Signal = attr.field(factory=Signal, repr=False)
    on_receive: Signal = attr.field(factory=Signal, repr=False)
    on_send: Signal = attr.field(factory=Signal, repr=False)

    # internal fields; can be used in init
    _clock: "Clock" = attr.field(factory=Clock, repr=False)
    _socket: anyio.abc.UDPSocket = attr.field(init=False, repr=False)

    # internal fields; can not be used in init
    _stack: AsyncExitStack = attr.field(init=False, repr=False, default=None)
    _task_group: anyio.abc.TaskGroup = attr.field(init=False, repr=False, default=None)
    _stopped: asyncio.Future = attr.field(init=False, repr=False, default=None)

    def encode_message(self) -> bytes:
        ...

    def decode_message(self, message: bytes) -> Any:
        ...

    async def __aenter__(self):
        if self.running:
            raise RuntimeError("Beacon is already running")

        if self.interface is None:
            self.interface = await get_active_interface()

        logger.debug(f"{self} starting on {self.addr}:{self.port}")

        self._stopped = asyncio.Future()
        self._socket = await create_broadcast_socket(
            local_port=self.port,
            family=socket.AF_INET,
            reuse_addr=True,
        )
        self._task_group = anyio.create_task_group()

        async with AsyncExitStack() as stack:
            await stack.enter_async_context(self._socket)
            await stack.enter_async_context(self._task_group)
            self._stack = stack.pop_all()

        if self.send:
            self._task_group.start_soon(self._start_sender)

        if self.listen:
            self._task_group.start_soon(self._start_listener)

        self.running = True

    async def __aexit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_val: Optional[BaseException],
        exc_tb: Optional[TracebackType],
    ) -> Optional[bool]:
        result = await self._stack.__aexit__(exc_type, exc_val, exc_tb)
        self.running = False
        self._stopped.set_result(True)  # todo set exception?
        logger.debug(f"{self} stopped")
        return result

    async def start(self):
        return await self.__aenter__()

    async def stop(self):
        self.cancel()
        return await self.__aexit__(None, None, None)

    def cancel(self):
        logger.debug(f"{self} stopping")
        self._task_group.cancel_scope.cancel()

    @property
    def stopped(self):  # TODO
        if not self.running:
            raise RuntimeError("Beacon is not running yet!")

        return asyncio.gather(*self._running_tasks, return_exceptions=True)

    @property
    def addr(self) -> str:
        return str(self.interface.network.broadcast_address)

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
        logger.debug(
            f"{self} sent broadcast message {message} to {self.addr}:{self.port}"
        )
        self._task_group.start_soon(self.on_send.emit, message)
        return message

    async def _start_listener(self):
        logger.info(f"{self} listener starting")
        try:
            while True:
                await self._receive_beacon()
        except asyncio.CancelledError:
            logger.info(f"{self} listener stopped")
            raise

    async def _receive_beacon(self) -> Optional[Tuple[Any, str, int]]:
        data, (addr, port) = await self._socket.receive()
        logger.debug(f"{self} received broadcast message {data} from {addr}:{port}")
        self._task_group.start_soon(self.on_receive.emit, data, addr, port)

        try:
            message = self.decode_message(data)
        except Exception as e:
            logger.warning(f"Error during decoding beacon message {data}: {e}")
            return

        self._task_group.start_soon(self.on_message.emit, addr, port)
        return message, addr, port


@attr.define(kw_only=True)
class MessageBeacon(Beacon):
    message: str = attr.field()

    def encode_message(self) -> bytes:
        return str.encode(self.message)

    def decode_message(self, message: bytes) -> str:
        return message.decode()


@attr.define(kw_only=True)
class AgentBeacon(Beacon):
    agent: "Agent" = attr.field(repr=False)
    add_docs(agent, "Agent associated with the beacon")

    on_peer: Signal = attr.field(factory=Signal, repr=False)

    version: int = attr.field(default=1, repr=False)  # unsigned short
    prefix: bytes = attr.field(default=b"CS", repr=False)  # Clover Swarm
    struct_format: str = attr.field(
        default=">2sH16sH", repr=False
    )  # 2 identifier bytes, 16 UUID bytes, unsigned short (port)
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

    def decode_message(self, message: bytes) -> Tuple["uuid.UUID", int]:
        prefix, version, peer_uuid, peer_port = struct.unpack(
            self.struct_format, message
        )
        if prefix != self.prefix or version != self.version:
            raise ValueError

        return uuid.UUID(bytes=peer_uuid), peer_port

    async def _receive_beacon(self) -> Optional[Tuple[Any, str, int]]:
        data = await super()._receive_beacon()
        if data is None:
            return None

        message, host, port = data
        peer_uuid, peer_port = message
        await self._process_peer(peer_uuid, host, peer_port)
        return message, host, port

    async def _process_peer(self, peer_uuid, peer_host, peer_port):
        if peer_uuid == self.agent.uuid:
            return

        self._task_group.start_soon(self.on_peer.emit, peer_uuid, peer_host, peer_port)
        logger.debug(f"{self} detected peer {peer_uuid} at {peer_host}:{peer_port}")


if __name__ == "__main__":

    async def main():
        beacon = MessageBeacon(message="Hello world")
        await beacon.start()
        await anyio.sleep(10)
        await beacon.stop()
        async with beacon:
            await anyio.sleep(10)

    logging.basicConfig(level=logging.DEBUG)
    asyncio.run(main())

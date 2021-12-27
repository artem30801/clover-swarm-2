# import rospy
import logging
from collections import AsyncIterable
from contextlib import suppress
from functools import partial
from typing import Callable, Optional

import anyio

from clover_swarm.async_ros.errors import TopicError

logger = logging.getLogger(__name__)


class AsyncSubscriber(AsyncIterable):
    def __init__(self, name, service_class, buffer_size=0, timeout=None):
        self.name = name
        self.service_class = service_class
        self.timeout = timeout

        send_channel, receive_channel = anyio.create_memory_object_stream(buffer_size)
        self._receive_channel = receive_channel
        self._send_channel = send_channel
        self._subscriber: Optional[Callable] = None  # Optional[rospy.Subscriber]
        # self._subscriber = rospy.Subscriber(self.name, self.service_class, )
        self._closed = False
        # self._lock = trio.Lock()

    def __str__(self):
        return f"{self.__class__.__name__}: {self.name}"

    @property
    def closed(self):
        return self._closed

    async def subscribe(self, timeout=None):
        logger.info(f"Waiting until service '{self.name}' is available.")
        service_waiter = partial(rospy.wait_for_service, self.name)
        with trio.fail_after(timeout):
            await trio.to_thread.run_sync(service_waiter)
        logger.info(f"Service '{self.name}' is available, subscribing.")

        self._subscriber = rospy.Subscriber(
            self.name,
            self.service_class,
        )  # todo queue size

    def unsubscribe(self):
        if self._subscriber is None:
            raise TopicError("Not subscribed yet")

        if self._closed:
            raise TopicError("Already unsubscribed")

        self._subscriber.unregister()
        self._closed = True

    async def get(self):
        return await self._receive_channel.receive()

    def get_nowait(self, suppress_errors=False):
        result = None

        try:
            result = self._receive_channel.receive_nowait()
        except trio.WouldBlock:
            if not suppress_errors:
                raise

        return result

    def __aiter__(self):
        async for value in self._receive_channel:
            yield value

    def _callback(self, data):
        logger.info(f"{self}: received data {data}")
        try:
            self._send_channel.send_nowait(data)
        except trio.WouldBlock:
            # drop the leftmost (oldest) item and put new stuff at the right
            old_data = self._receive_channel.receive_nowait()
            logger.info(f"{self}: receive buffer is full, dropping old data {old_data}")
            self._send_channel.send_nowait(data)

    # def process_data(self, data):
    #     return data


if __name__ == "__main__":
    logging.basicConfig(level=logging.CRITICAL)

    async def main():
        rangefinder = AsyncSubscriber("124", "SOME CLASS OBJ")
        # rangefinder.get_last()

        with anyio.fail_after(10):  # wait for up to 10 seconds
            print(await rangefinder._receive_channel.receive())

        print(rangefinder._receive_channel.receive_nowait())

        async for range_data in rangefinder:
            print(range_data)

        # async with trio.open_nursery() as nursery:
        #     nursery.start_soon(camera)
        #     nursery.start_soon(flight)

        print("ALL DONE")

    trio.run(main)

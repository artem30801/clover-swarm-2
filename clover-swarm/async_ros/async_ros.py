import trio
import rospy

import logging
#  import attr
from functools import partial
from typing import Optional, Callable

import time

logger = logging.getLogger(__name__)


async def debug_counter():
    print("-started counting")
    for i in range(6):
        print("-counting", i)
        await trio.sleep(0.5)

    print("-done counting")


def some_blocking(*args, **kwargs):
    print("+started blocking service call", args, kwargs)
    time.sleep(2)
    print("+done blocking call")

    return "some result"


# @attr.define()
class AsyncService:
    # _service_proxy: Callable = attr.field()

    # todo add logging
    def __init__(self, name, service_class):
        self.name = name
        self.service_class = service_class
        self._service_proxy: Optional[Callable] = None
        # maybe needs lock?

    def __str__(self):
        return f"{self.__class__.__name__}: {self.name}"

    async def connect(self, timeout=None):
        logger.info(f"Waiting until service '{self.name}' is available.")
        service_waiter = partial(rospy.wait_for_service, self.name, timeout=timeout)
        await trio.to_thread.run_sync(service_waiter)

        logger.info(f"Service '{self.name}' is available, connecting proxy.")
        proxy = rospy.ServiceProxy(self.name, self.service_class)

        # proxy = some_blocking  # debug code

        self._service_proxy = proxy

    async def call(self, *args, **kwargs):
        return await self.__call__(*args, **kwargs)

    def sync_call(self, *args, **kwargs):
        result = self._service_proxy(*args, **kwargs)
        return result

    async def __call__(self, *args, **kwargs):
        if self._service_proxy is None:
            logger.warning(f"Service {self.name} was not connected previously, connecting now. "
                           f"Use AsyncService.connect() to connect service proxy preemptively.")
            await self.connect()
        
        logger.debug(f"Calling service {self.name} with args {args}; kwargs {kwargs}")
        proxy = partial(self._service_proxy, *args, **kwargs)
        result = await trio.to_thread.run_sync(proxy)
        logger.debug(f"Service {self.name} returned result {result}")
        return result


if __name__ == '__main__':

    logging.basicConfig(level=logging.DEBUG)
    
    async def main():
        navigate = AsyncService("124", "SOME CLASS OBJ")

        await navigate("Some call")

        async with trio.open_nursery() as nursery:
            nursery.start_soon(debug_counter)
            nursery.start_soon(navigate, "Another call")

        print("ALL DONE")

    trio.run(main)

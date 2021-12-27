import logging
import time
from functools import partial
from typing import Callable, Optional

import anyio
import attr
import rospy

logger = logging.getLogger(__name__)


async def debug_counter():
    print("-started counting")
    for i in range(6):
        print("-counting", i)
        await anyio.sleep(0.5)

    print("-done counting")


def some_blocking(*args, **kwargs):
    print("+started blocking service call", args, kwargs)
    time.sleep(2)
    print("+done blocking call")

    return "some result"


@attr.define()
class AsyncService:
    name: str = attr.field()
    service_class: type = attr.field()
    _service_proxy: Optional[Callable] = attr.field(default=None)
    _lock: anyio.Lock = attr.field(factory=anyio.Lock)

    # def __str__(self):
    #     return f"{self.__class__.__name__}: {self.name}"

    async def connect(self, timeout=None):
        logger.info(f"Waiting until service '{self.name}' is available.")
        service_waiter = partial(rospy.wait_for_service, self.name, timeout=timeout)
        try:
            with anyio.fail_after(timeout):
                await anyio.to_thread.run_sync(service_waiter, cancellable=True)
        except TimeoutError:
            logger.error(f"Timeout! Service '{self.name}' is not available!")
            return

        logger.info(f"Service '{self.name}' is available, connecting proxy.")
        proxy = rospy.ServiceProxy(self.name, self.service_class)

        # proxy = some_blocking  # debug code

        self._service_proxy = proxy

    async def call(self, *args, **kwargs):
        return await self.__call__(*args, **kwargs)

    def call_nowait(self, *args, **kwargs):
        """NO LOCK!"""
        logger.debug(f"Calling service {self.name} with args {args}; kwargs {kwargs}")
        result = self._service_proxy(*args, **kwargs)
        logger.debug(f"Service {self.name} returned result {result}")
        return result

    async def __call__(self, *args, **kwargs):
        if self._service_proxy is None:
            logger.warning(
                f"Service {self.name} was not connected previously, connecting now. "
                f"Use AsyncService.connect() to connect service proxy preemptively."
            )
            await self.connect()

        async with self._lock:
            proxy = partial(self.call_nowait, *args, **kwargs)
            result = await anyio.to_thread.run_sync(proxy)
            return result


if __name__ == "__main__":
    logging.basicConfig(level=logging.CRITICAL)

    async def main():
        navigate = AsyncService("124", "SOME CLASS OBJ")

        await navigate("Some call")

        async with trio.open_nursery() as nursery:
            nursery.start_soon(debug_counter)
            nursery.start_soon(navigate, "Another call")

        print("ALL DONE")

    import trio

    trio.run(main)

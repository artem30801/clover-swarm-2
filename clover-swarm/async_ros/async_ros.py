import trio
#  import attr
from functools import partial
from typing import Optional, Callable

import time


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


# @attr.define()
class AsyncService:
    # _service_proxy: Callable = attr.field()

    # todo add logging
    def __init__(self, name, service_class):
        self.name = name
        self.service_class = service_class
        self._service_proxy: Optional[Callable] = None
        # maybe needs lock?

    async def connect(self):
        # rospy.wait_for_service(self.name)
        # proxy = rospy.ServiceProxy(self.name, self.service_class)

        proxy = some_blocking  # debug code

        self._service_proxy = proxy

    async def call(self, *args, **kwargs):
        return await self.__call__(*args, **kwargs)

    def sync_call(self, *args, **kwargs):
        result = self._service_proxy(*args, **kwargs)
        return result

    async def __call__(self, *args, **kwargs):
        if self._service_proxy is None:
            await self.connect()

        proxy = partial(self._service_proxy, *args, **kwargs)
        result = await trio.to_thread.run_sync(proxy)
        return result


if __name__ == '__main__':
    async def main():
        service = AsyncService("124", "SOME CLASS OBJ")

        await service.call("Some call")

        async with trio.open_nursery() as nursery:
            nursery.start_soon(debug_counter)
            nursery.start_soon(service.call, "Another call")

    trio.run(main)

import asyncio
from types import TracebackType
from typing import List, Optional, Type

import anyio
import anyio.abc
import attr


@attr.define()
class GatherTaskGroup:
    _results: List[asyncio.Future] = attr.field(factory=list)
    _task_group: anyio.abc.TaskGroup = attr.field(factory=anyio.create_task_group)

    async def __aenter__(self):
        await self._task_group.__aenter__()
        return self

    async def __aexit__(
        self, exc_type: Optional[Type[BaseException]], exc_val: Optional[BaseException], exc_tb: Optional[TracebackType]
    ) -> Optional[bool]:
        result = await self._task_group.__aexit__(exc_type, exc_val, exc_tb)

        for future in self._results:
            if not future.done():
                future.cancel()

        return result

    def __getitem__(self, item):
        return self._results[item]

    @property
    def cancel_scope(self):
        return self._task_group.cancel_scope

    async def _run_async(self, pos, func, *args):
        future = self._results[pos]
        try:
            result = await func(*args)
            future.set_result(result)
        except Exception as e:
            future.set_exception(e)

    def start_soon(self, func, *args):
        pos = len(self._results)
        future = asyncio.Future()
        self._results.append(future)
        self._task_group.start_soon(self._run_async, pos, func, *args)

        return pos

    @property
    def results(self):
        return self._results.copy()

    async def wait_result(self, pos):
        future = self._results[pos]
        return await future

    def get_result(self, pos, return_exception=False):
        future: asyncio.Future = self._results[pos]
        if return_exception:
            exception = future.exception()
            if exception is not None:
                return Exception

        return future.result()

    def get_results(self, return_exception=False) -> list:
        return [self.get_result(i, return_exception) for i in range(len(self._results))]

    # @staticmethod
    # def get_future_result(future, return_exception=False, catch_all=False):
    #     try:
    #         if return_exception:
    #             exception = future.exception()
    #             if exception is not None:
    #                 return exception
    #     except Exception


if __name__ == "__main__":

    async def test():
        async def s(x):
            await anyio.sleep(x)
            return 3 * x

        async with GatherTaskGroup() as rg:
            a = rg.start_soon(s, 0.125)
            b = rg.start_soon(s, 0.25)

        assert rg.get_result(a) == 0.375
        assert rg.get_result(b) == 0.75

        print(rg.results)
        assert rg.get_results() == [0.375, 0.75]

    anyio.run(test)

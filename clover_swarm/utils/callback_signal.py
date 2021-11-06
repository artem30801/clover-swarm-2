import attr
from typing import Set, Callable
import logging

import asyncio
from functools import partial


from clover_swarm.utils.task_group import GatherTaskGroup

logger = logging.getLogger(__name__)


@attr.define()
class Signal:
    _callbacks: Set[Callable] = attr.field(factory=list)

    def connect(self, func):
        self._callbacks.add(func)

    def disconnect(self, func):
        self._callbacks.remove(func)

    @property
    def all(self):
        return self._callbacks.copy()

    async def emit(self, *args, **kwargs):
        results = list()
        async with GatherTaskGroup() as task_group:
            for func in self._callbacks:
                func = partial(func, *args, **kwargs)
                if asyncio.iscoroutinefunction(func):
                    task_group.start_soon(func)
                else:
                    results.append(func())

        # at this point all tasks are done
        for future in task_group.results:
            try:
                result = future.result()
            except Exception as e:
                logger.warning(f"Error during signal callback execution {func}: {e}")
                result = e
            results.append(result)

        return results

    # async def __call__(self, *args, **kwargs):
    #     return await self.emit(*args, **kwargs)

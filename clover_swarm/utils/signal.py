import attr
from typing import List, Callable
import logging

import asyncio

logger = logging.getLogger(__name__)


@attr.define()
class Signal:
    _callbacks: List[Callable] = attr.field(factory=list)

    def connect(self, func):
        self._callbacks.append(func)

    def disconnect(self, func):
        self._callbacks.remove(func)

    @property
    def all(self):
        return self._callbacks.copy()

    async def emit(self, *args, **kwargs):
        results = []
        for func in self._callbacks:
            try:
                if asyncio.iscoroutinefunction(func):
                    result = await func(*args, **kwargs)
                else:
                    result = func(*args, **kwargs)
            except Exception as e:
                logger.warning(f"Error during signal callback execution {func}: {e}")
                result = e
            results.append(result)

        return results

    # async def __call__(self, *args, **kwargs):
    #     return await self.emit(*args, **kwargs)

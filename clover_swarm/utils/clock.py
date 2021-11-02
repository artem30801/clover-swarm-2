import attr
import time
import anyio
import logging
from typing import Optional

logger = logging.getLogger(__name__)


@attr.define()
class Clock:
    max_sleep: float = attr.field(default=0.3)

    @property
    def time(self):
        return time.time()

    async def sleep(self, duration: float):
        await self.sleep_until(self.time + duration)

    async def sleep_until(self, end_time: float, max_sleep: Optional[float] = None):
        max_sleep = self.max_sleep if max_sleep is None else max_sleep

        while True:
            diff = min(end_time - self.time, max_sleep)
            if diff <= 0:
                break
            # elif diff <= 0.05:
            #     await anyio.sleep(diff)
            else:
                await anyio.sleep(diff / 2)

    def make_rate(self, *, rate: float = None, delay: float = None):
        return Rate(rate=rate, delay=delay, clock=self)


@attr.define()
class Rate:
    delay: float = attr.field()

    _clock: "Clock" = attr.field(repr=False, factory=Clock)
    _last_call: Optional[float] = attr.field(init=False, repr=False, default=None)

    def __init__(self, *, rate: float = None, delay: float = None, clock: Optional["Clock"] = None):
        # either rate or delay must be specified
        if (rate is None and delay is None) or (rate is not None and delay is not None):
            raise ValueError("Either rate or delay must be specified")

        if rate is not None:
            delay = 1 / rate

        self.__attrs_init__(delay=delay, clock=clock)
        self.reset()

    @property
    def rate(self):
        return 1 / self.delay

    @rate.setter
    def rate(self, value):
        self.delay = 1 / value

    def reset(self):
        self._last_call = self._clock.time

    async def sleep(self):
        if self._last_call is None:
            self.reset()

        self._last_call += self.delay
        if (delta := self._last_call - self._clock.time) <= 0:
            delta = abs(delta)
            logger.warning(f"Rate is behind the schedule by {delta} seconds, skipping sleep")
            # self._last_call += delta
            return

        await self._clock.sleep_until(self._last_call)

    async def __call__(self):
        await self.sleep()

    def __await__(self):
        return self.sleep().__await__()

    async def __aiter__(self):
        while True:
            await self.sleep()
            yield

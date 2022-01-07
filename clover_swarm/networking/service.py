import asyncio
import logging
from abc import ABC
from contextlib import AsyncExitStack
from types import TracebackType
from typing import Optional, Type

import anyio.abc
import attr

logger = logging.getLogger(__name__)


@attr.define()
class Service(ABC):
    """Base abstract class for services that can be run as context managers."""

    running: bool = attr.field(default=False, init=False, repr=False)
    was_run: bool = attr.field(default=False, init=False, repr=False)

    _stack: AsyncExitStack = attr.field(init=False, repr=False, default=None)
    _task_group: anyio.abc.TaskGroup = attr.field(init=False, repr=False, factory=anyio.create_task_group)
    stopped: asyncio.Future = attr.field(init=False, repr=False, factory=asyncio.Future)

    async def __aenter__(self):
        if self.running:
            raise RuntimeError(f"{self} is already running.")

        if self.was_run:
            raise RuntimeError(f"{self} is not re-usable! Create another instance to start it again.")

        self.running = True
        self.was_run = True

        async with AsyncExitStack() as stack:
            await self._enter_contexts(stack)
            self._stack = stack.pop_all()

        logger.debug(f"{self} started!")

    async def __aexit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_val: Optional[BaseException],
        exc_tb: Optional[TracebackType],
    ) -> Optional[bool]:
        self.cancel()
        result = await self._stack.__aexit__(exc_type, exc_val, exc_tb)
        self.running = False
        self.stopped.set_result(True)  # todo set exception?
        logger.debug(f"{self} stopped")
        return result

    async def _enter_contexts(self, stack: AsyncExitStack) -> None:
        await stack.enter_async_context(self._task_group)

    async def start(self) -> None:
        """Starts the service and waits for initialisation."""
        return await self.__aenter__()

    async def stop(self) -> Optional[bool]:
        """Stops the service and waits for graceful shutdown."""
        return await self.__aexit__(None, None, None)

    def cancel(self) -> None:
        """Triggers shutdown of running service, does not wait for shutdown to be complete."""
        if not self.running:
            raise RuntimeError(f"{self} is not running now!")

        logger.debug(f"{self} stopping")
        self._task_group.cancel_scope.cancel()

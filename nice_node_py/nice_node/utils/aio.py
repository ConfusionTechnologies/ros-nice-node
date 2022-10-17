"""Utilities for using asyncio in ROS2 nodes."""

import asyncio
import contextvars
import functools
from typing import Coroutine

__all__ = ["to_thread", "wait_coro"]

# https://github.com/python/cpython/blob/main/Lib/asyncio/threads.py
async def to_thread(func, /, *args, **kwargs):
    """BACKPORTED: Asynchronously run function *func* in a separate thread.

    Any *args and **kwargs supplied for this function are directly passed
    to *func*. Also, the current :class:`contextvars.Context` is propagated,
    allowing context variables from the main thread to be accessed in the
    separate thread.

    Return a coroutine that can be awaited to get the eventual result of *func*.
    """
    loop = asyncio.get_running_loop()
    ctx = contextvars.copy_context()
    func_call = functools.partial(ctx.run, func, *args, **kwargs)
    return await loop.run_in_executor(None, func_call)


def wait_coro(
    coro: Coroutine, loop: asyncio.AbstractEventLoop = None, timeout: float = None
):
    """Wait for async coroutine to complete.

    If `loop` is None, the event loop will be retrieved via `asyncio.get_event_loop()`.
    This can have unintended consequences, so try to explicitly set the event loop.

    Args:
        coro (Coroutine): Coroutine to wait for.
        loop (asyncio.AbstractEventLoop, optional): Asyncio event loop to use. Defaults to None.
        timeout (float, optional): Timeout in seconds till coroutine is cancelled. Defaults to None.

    Returns:
        Any: Return value of the coroutine.
    """
    if loop is None:
        loop = asyncio.get_event_loop()
    future = asyncio.run_coroutine_threadsafe(coro, loop)
    try:
        return future.result(timeout)
    finally:
        future.cancel()

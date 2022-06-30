"""Utilities for using asyncio with ROS2"""

import asyncio
import contextvars
import functools
from typing import Coroutine


# https://github.com/python/cpython/blob/main/Lib/asyncio/threads.py
async def to_thread(func, /, *args, **kwargs):
    """(BACKPORTED) Asynchronously run function *func* in a separate thread.

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

def wait_coro(coro: Coroutine, loop: asyncio.AbstractEventLoop=None, timeout: float=10.0):
    """Wait for async coro synchronously with timeout of 10.0 seconds by default."""
    if loop is None:
        loop = asyncio.get_event_loop()
    future = asyncio.run_coroutine_threadsafe(coro, loop)
    try:
        return future.result(timeout)
    finally:
        future.cancel()

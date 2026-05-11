# SPDX-License-Identifier: LGPL-3.0-or-later
from contextlib import contextmanager
from functools import wraps

import jupedsim.native as py_jps


class Trace:
    """
    .. important::

        This is indented for internal usage. We will not guarantee that this API will
        stable and available in any release. It might be changed on any update, regardless of
        a major/minor/patch update.
    """

    def __init__(self) -> None:
        self._obj = py_jps.Trace

    @property
    def iteration_duration(self) -> float:
        """Time for one simulation iteration in us.

        Returns:
             Time for one simulation iteration in us
        """
        return self._obj.iteration_duration

    @property
    def operational_level_duration(self) -> float:
        """Time for one simulation iteration in the operational level in us.

        Returns:
             Time for one simulation iteration in the operational level in us
        """

        return self._obj.operational_level_duration

    def __str__(self) -> str:
        return self._obj.__repr__()


def enable_tracing():
    py_jps.Profiler.enable()


def disable_tracing():
    py_jps.Profiler.disable()


def is_tracing_enabled():
    return py_jps.Profiler().is_enabled


def start_trace(name: str):
    py_jps.Profiler.start_trace_event(name)


def end_trace():
    py_jps.Profiler.end_trace_event()


def dump_traces(filename: str):
    py_jps.Profiler.dump_and_reset(filename)


@contextmanager
def trace_event(name: str):
    start_trace(name)
    try:
        yield
    finally:
        end_trace()


def trace_function(_func=None, *, name: str | None = None):
    """Decorator to trace a function using the function name as the trace event name.

    Args:
        name: Optional name for the trace event. If not provided, uses the function's
              ``__name__`` attribute.

    Returns:
        A decorator that wraps the function with tracing.
    """

    def decorator(func):
        trace_name = name if name is not None else func.__name__

        @wraps(func)
        def wrapper(*args, **kwargs):
            start_trace(trace_name)
            try:
                return func(*args, **kwargs)
            finally:
                end_trace()

        return wrapper

    return decorator(_func) if _func else decorator

"""Trajectory utility package.

Provides goto and goto_async functions. They let you easily create and compose movements on joint coordinates.
"""

import asyncio
import numpy as np
import time
from concurrent.futures import ThreadPoolExecutor
from queue import Queue
from typing import Dict, Optional

from .interpolation import InterpolationMode
from ..joint import Joint


def goto(
    goal_positions: Dict[Joint, float],
    duration: float,
    starting_positions: Optional[Dict[Joint, float]] = None,
    sampling_freq: float = 100,
    interpolation_mode: InterpolationMode = InterpolationMode.LINEAR,
):
    """Send joints command to move the robot to a goal_positions within the specified duration.

    This function will block until the movement is over. See goto_async for an asynchronous version.

    The goal positions is expressed in joints coordinates. You can use as many joints target as you want.
    The duration is expressed in seconds.
    You can specify the starting_position, otherwise its current position is used,
    for instance to start from its goal position and avoid bumpy start of move.
    The sampling freq sets the frequency of intermediate goal positions commands.
    You can also select an interpolation method use (linear or minimum jerk) which will influence directly the trajectory.

    """
    exc_queue: Queue[Exception] = Queue()

    def _wrapped_goto():
        try:
            asyncio.run(
                goto_async(
                    goal_positions=goal_positions,
                    duration=duration,
                    starting_positions=starting_positions,
                    sampling_freq=sampling_freq,
                    interpolation_mode=interpolation_mode,
                ),
            )
        except Exception as e:
            exc_queue.put(e)

    with ThreadPoolExecutor() as exec:
        exec.submit(_wrapped_goto)
    if not exc_queue.empty():
        raise exc_queue.get()


async def goto_async(
    goal_positions: Dict[Joint, float],
    duration: float,
    starting_positions: Optional[Dict[Joint, float]] = None,
    sampling_freq: float = 100,
    interpolation_mode: InterpolationMode = InterpolationMode.LINEAR,
):
    """Send joints command to move the robot to a goal_positions within the specified duration.

    This function is asynchronous and will return a Coroutine. This can be used to easily combined multiple gotos.
    See goto for an blocking version.

    The goal positions is expressed in joints coordinates. You can use as many joints target as you want.
    The duration is expressed in seconds.
    You can specify the starting_position, otherwise its current position is used,
    for instance to start from its goal position and avoid bumpy start of move.
    The sampling freq sets the frequency of intermediate goal positions commands.
    You can also select an interpolation method use (linear or minimum jerk) which will influence directly the trajectory.

    """
    for key in goal_positions.keys():
        if not isinstance(key, Joint):
            raise ValueError('goal_positions keys should be Joint!')

    if starting_positions is None:
        starting_positions = {j: j.goal_position for j in goal_positions.keys()}

    # Make sure both starting and goal positions are in the same order
    starting_positions = {j: starting_positions[j] for j in goal_positions.keys()}

    length = round(duration * sampling_freq)
    if length < 1:
        raise ValueError('Goto length too short! (incoherent duration {duration} or sampling_freq {sampling_freq})!')

    joints = starting_positions.keys()
    dt = 1 / sampling_freq

    traj_func = interpolation_mode(
        np.array(list(starting_positions.values())),
        np.array(list(goal_positions.values())),
        duration,
    )

    t0 = time.time()
    while True:
        elapsed_time = time.time() - t0
        if elapsed_time > duration:
            break

        point = traj_func(elapsed_time)
        for j, pos in zip(joints, point):
            j.goal_position = pos

        await asyncio.sleep(dt)

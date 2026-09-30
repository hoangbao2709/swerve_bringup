"""Symmetric continuous drive derating against the final steering target."""
import math


def alignment_scale(error, full_error, stop_error):
    if not 0 <= full_error < stop_error <= math.pi:
        raise ValueError('alignment errors must satisfy 0 <= full < stop <= pi')
    error = abs((error + math.pi) % (2 * math.pi) - math.pi)
    if error <= full_error:
        return 1.0
    if error >= stop_error:
        return 0.0
    return 0.5 * (1 + math.cos(math.pi * (error - full_error) / (stop_error - full_error)))

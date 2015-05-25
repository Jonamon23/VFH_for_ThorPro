function value = clampWithinRange(value, low, high)
value = max(min(value, high), low);
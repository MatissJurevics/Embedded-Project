def _map(cur, low, high, min = 0, max = 100):
    ratio = (cur - low) / (high - low)
    return (max - min) * ratio

print(_map(3, 0, 5, 0, 25))
# filters.py

class MovingAverageFilter:
    def __init__(self, window_size=10):
        self.window_size = window_size
        self.values = []

    def update(self, new_value):
        self.values.append(new_value)
        if len(self.values) > self.window_size:
            self.values.pop(0)
        return sum(self.values) / len(self.values)

class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.last_value = None

    def update(self, new_value):
        if self.last_value is None:
            self.last_value = new_value
        else:
            self.last_value = self.alpha * new_value + (1 - self.alpha) * self.last_value
        return self.last_value

"""

The code below defines two types of filters commonly used in signal processing:
the Moving Average Filter and the Low Pass Filter.
Each class is designed to filter noise from data in different ways, useful for smoothing out sensor readings or any time-series data.

Moving Average Filter:
This filter calculates the average of the last n values (window_size).
It's useful for smoothing out short-term fluctuations and highlighting longer-term trends or cycles.
As new values come in, older ones are removed from the calculation, maintaining a "moving" window of values to average.

Low Pass Filter:
This filter allows signals with a frequency lower than a cutoff frequency to pass through and attenuates signals with frequencies higher than the cutoff frequency.
The alpha value determines the smoothing effect, with lower values providing more smoothing.
It's particularly useful for dampening high-frequency noise in sensor readings while still being responsive to actual changes in the signal.
"""

# Define a class for the Moving Average Filter
class MovingAverageFilter:
    def __init__(self, window_size=10):
        # Constructor for the filter with a default window size of 10
        self.window_size = window_size  # The number of values to consider for the moving average
        self.values = []  # A list to store the latest values

    def update(self, new_value):
        # Update the filter with a new value and calculate the moving average
        self.values.append(new_value)  # Add the new value to the list
        # If the list exceeds the window size, remove the oldest value
        if len(self.values) > self.window_size:
            self.values.pop(0)
        # Return the average of the values in the window
        return sum(self.values) / len(self.values)

# Define a class for the Low Pass Filter
class LowPassFilter:
    def __init__(self, alpha):
        # Constructor for the filter with a smoothing factor alpha
        self.alpha = alpha  # Smoothing factor between 0 and 1
        self.last_value = None  # Store the last filtered value

    def update(self, new_value):
        # Update the filter with a new value and calculate the low pass filtered value
        if self.last_value is None:
            # If it's the first value, just set it as the last value
            self.last_value = new_value
        else:
            # Apply the low pass filter formula
            self.last_value = self.alpha * new_value + (1 - self.alpha) * self.last_value
        # Return the filtered value
        return self.last_value

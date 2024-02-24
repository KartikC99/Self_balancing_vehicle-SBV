"""
The code PIDController class intended to implement a PID (Proportional-Integral-Derivative) control algorithm. This algorithm is widely used in control systems to
maintain a desired setpoint by adjusting the output based on the difference between the desired setpoint and the current value.
Upon initialization, the class sets up PID constants for proportional gain (Kp), integral gain (Ki), derivative gain (Kd), and the target setpoint.
It initializes variables to keep track of the previous error and the integral of errors over time, which are essential for calculating the PID terms.

The update method calculates the control signal needed to minimize the error between the current value and the setpoint.
It does this by computing the proportional term based on the current error, the integral term by accumulating past errors over time,
and the derivative term by determining the rate of change of the error.
These three components are then combined to form the control signal, which can be used to adjust the system towards the desired setpoint.
The method also updates the previous error for use in the next iteration.

The clear method resets the integral and previous error values, useful for reinitializing the controller in certain scenarios.
The Adjust method allows for dynamic adjustment of the PID coefficients, enabling the tuning of the controller to achieve optimal performance.
This class is a fundamental building block for systems requiring precise control to maintain stability or reach a specific condition,
such as our self-balancing vehicle.
"""
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        # Initialize PID constants
        self.Kp = Kp # Proportional gain
        self.Ki = Ki # Integral gain
        self.Kd = Kd # Derivative gain
        self.setpoint = setpoint # Desired value
        
        # Initialize variables for calculating the derivative term
        self.prev_error = 0 # Previous error value
        self.integral = 0 # Integral sum

    def update(self, current_value, dt):
        # Calculate the difference between the desired and actual value
        error = self.setpoint - current_value

        # Proportional term: proportional to current error
        P = self.Kp * error

        # Integral term: sum of all past errors, multiplied by time and Ki
        self.integral += error * dt
        I = self.Ki * self.integral

        # Derivative term: rate of error change, multiplied by Kd
        derivative = (error - self.prev_error) / dt
        D = self.Kd * derivative

        # Calculate the control signal
        control_signal = P + I + D

        # Update previous error for the next iteration
        self.prev_error = error

        return control_signal
    
    def clear(self):
        # Reset integral and previous error to zero
        self.integral = 0
        self.prev_error = 0
        
    def Adjust(self, Ap, Ai, Ad):
        # Method to adjust PID coefficients
        self.Kp = Ap  # Adjust Proportional gain
        self.Ki = Ai  # Adjust Integral gain
        self.Kd = Ad  # Corrected: Adjust Derivative gain
        

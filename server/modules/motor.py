"""Control the robot's motors through Jetson GPIO and PWM."""

from time import sleep

import Jetson.GPIO as GPIO

# USAGE
# pip install Jetson.GPIO
# pip install -U Jetson.GPIO
# sudo groupadd -f -r gpio
# sudo usermod -a -G gpio nano
# sudo /opt/nvidia/jetson-io/jetson-io.py

# INFO
# https://github.com/NVIDIA/jetson-gpio/blob/master/samples/simple_pwm.py
# https://github.com/NVIDIA/jetson-gpio
# https://forums.developer.nvidia.com/t/jetson-gpio-lib-pwm-not-working-on-orin-nano-dev-kit/255070/5
# https://docs.nvidia.com/jetson/archives/r35.4.1/DeveloperGuide/text/HR/ConfiguringTheJetsonExpansionHeaders.html

# NOTE


class MotorHandler:
    """Gracefully Manage motor GPIO states, speed, direction and shutdown."""

    def __init__(self, logHandler):
        """Configure GPIO pins, PWM outputs and initial motor state."""
        self._log = logHandler
        GPIO.setmode(GPIO.BOARD)
        frequency = 50
        GPIO.setup(33, GPIO.OUT, initial=GPIO.HIGH)  # left motor forward (pwm)
        self._left_pwm = GPIO.PWM(33, frequency)
        GPIO.setup(15, GPIO.OUT, initial=GPIO.HIGH)  # right motor forward (pwm)
        self._right_pwm = GPIO.PWM(15, frequency)
        self._left_pin = 29  # left motor reverse (NOT pwm)
        GPIO.setup(self._left_pin, GPIO.OUT, initial=GPIO.LOW)
        self._right_pin = 31  # right motor reverse (NOT pwm)
        GPIO.setup(self._right_pin, GPIO.OUT, initial=GPIO.LOW)
        self._left_forward = False
        self._left_reverse = False
        self._right_forward = False
        self._right_reverse = False
        self._pwm_speed = 45

    def motorGetPwmSpeed(self) -> int:
        """Return the configured PWM value (not the hardware PWM value)."""
        return self._pwm_speed

    def motorSetLeftSpeed(self, speed: int):
        """Set the left motor's forward PWM speed."""
        if speed >= 0 or speed <= 100:
            try:
                if self._left_reverse:
                    self._left_reverse = False
                    GPIO.output(self._left_pin, GPIO.LOW)
                if not self._left_forward:
                    self._left_forward = True
                    self._left_pwm.start(speed)
                else:
                    self._left_pwm.ChangeDutyCycle(speed)
                # self._log.logAdd(f"MOTOR: Left motor set to: {speed}%", True)
            except (OSError, RuntimeError, ValueError) as e:
                GPIO.cleanup()
                self._log.logAdd(f"MOTOR: Left motor speed set caused error: {e}", True)
        else:
            self._log.logAdd(f"MOTOR: Attempted to set left motor to invalid speed: {speed!s}%", True)

    def motorSetRightSpeed(self, speed: int):
        """Set the right motor's forward PWM speed."""
        if speed >= 0 or speed <= 100:
            try:
                if self._right_reverse:
                    self._right_reverse = False
                    GPIO.output(self._right_pin, GPIO.LOW)
                if not self._right_forward:
                    self._right_forward = True
                    self._right_pwm.start(speed)
                else:
                    self._right_pwm.ChangeDutyCycle(speed)
                # self._log.logAdd(f"MOTOR: Right motor set to: {speed}%", True)
            except (OSError, RuntimeError, ValueError) as e:
                GPIO.cleanup()
                self._log.logAdd(f"MOTOR: Right motor speed set caused error: {e}", True)
        else:
            self._log.logAdd(f"MOTOR: Attempted to set right motor to invalid speed: {speed!s}%", True)

    def motorSetReverseLeftSpeed(self, speed: int):
        """Set the left motor's reverse speed using inverted PWM."""
        if speed >= 0 or speed <= 100:
            try:
                inverse_speed = 100 - speed
                if self._left_forward:
                    self._left_pwm.ChangeDutyCycle(inverse_speed)
                else:
                    self._left_forward = True
                    self._left_pwm.start(inverse_speed)
                if not self._left_reverse:
                    self._left_reverse = True
                    GPIO.output(self._left_pin, GPIO.HIGH)
                # self._log.logAdd(f"MOTOR: Left motor reverse set to: {speed}%", True)
            except (OSError, RuntimeError, ValueError) as e:
                GPIO.cleanup()
                self._log.logAdd(f"MOTOR: Left motor reverse speed set caused error: {e}", True)
        else:
            self._log.logAdd(f"MOTOR: Attempted to set left motor to invalid reverse speed: {speed}%", True)

    def motorSetReverseRightSpeed(self, speed: int):
        """Set the right motor's reverse speed using inverted PWM."""
        if speed >= 0 or speed <= 100:
            try:
                inverse_speed = 100 - speed
                if self._right_forward:
                    self._right_pwm.ChangeDutyCycle(inverse_speed)
                else:
                    self._right_forward = True
                    self._right_pwm.start(inverse_speed)
                if not self._right_reverse:
                    self._right_reverse = True
                    GPIO.output(self._right_pin, GPIO.HIGH)
                # self._log.logAdd(f"MOTOR: Right motor reverse set to: {speed}%", True)
            except (OSError, RuntimeError, ValueError) as e:
                GPIO.cleanup()
                self._log.logAdd(f"MOTOR: Right motor reverse speed set caused error: {e}", True)
        else:
            self._log.logAdd(f"MOTOR: Attempted to set right motor to invalid reverse speed: {speed}%", True)

    def motorForward(self):
        """Run both motors forward at the configured speed."""
        try:
            if self._left_reverse:
                self._left_reverse = False
                GPIO.output(self._left_pin, GPIO.LOW)
            if not self._left_forward:
                self._left_forward = True
                self._left_pwm.start(self._pwm_speed)
            else:
                self._left_pwm.ChangeDutyCycle(self._pwm_speed)

            if self._right_reverse:
                self._right_reverse = False
                GPIO.output(self._right_pin, GPIO.LOW)
            if not self._right_forward:
                self._right_forward = True
                self._right_pwm.start(self._pwm_speed)
            else:
                self._right_pwm.ChangeDutyCycle(self._pwm_speed)
            self._log.logAdd(f"MOTOR: Forwards at: {self._pwm_speed!s}%", True)
        except (OSError, RuntimeError, ValueError) as e:
            GPIO.cleanup()
            self._log.logAdd(f"MOTOR: Forwards caused error: {e}", True)

    def motorSmoothForward(self):
        """Gradually bring stopped motors up to the configured forward speed."""
        try:
            if self._left_reverse:
                self._left_reverse = False
                GPIO.output(self._left_pin, GPIO.LOW)
            if self._right_reverse:
                self._right_reverse = False
                GPIO.output(self._right_pin, GPIO.LOW)

            if not self._left_forward and not self._right_forward:
                step = round(self._pwm_speed / 5)
                currentPwmLevel = step
                self._left_pwm.start(currentPwmLevel)
                self._right_pwm.start(currentPwmLevel)
                sleep(0.1)
                for _ in range(3):
                    currentPwmLevel += step
                    self._left_pwm.ChangeDutyCycle(currentPwmLevel)
                    self._right_pwm.ChangeDutyCycle(currentPwmLevel)
                    sleep(0.1)
                self._left_pwm.ChangeDutyCycle(self._pwm_speed)
                self._right_pwm.ChangeDutyCycle(self._pwm_speed)
                self._left_forward = True
                self._right_forward = True
                self._log.logAdd(f"MOTOR: Smooth forwards at: {self._pwm_speed!s}%", True)
        except (OSError, RuntimeError, ValueError) as e:
            GPIO.cleanup()
            self._log.logAdd(f"MOTOR: Smooth forwards caused error: {e}", True)

    def motorReverse(self):
        """Run both motors in reverse at the configured speed."""
        try:
            inverse_speed = 100 - self._pwm_speed
            if self._left_forward:
                self._left_pwm.ChangeDutyCycle(inverse_speed)
            else:
                self._left_forward = True
                self._left_pwm.start(inverse_speed)
            if not self._left_reverse:
                self._left_reverse = True
                GPIO.output(self._left_pin, GPIO.HIGH)

            if self._right_forward:
                self._right_pwm.ChangeDutyCycle(inverse_speed)
            else:
                self._right_forward = True
                self._right_pwm.start(inverse_speed)
            if not self._right_reverse:
                self._right_reverse = True
                GPIO.output(self._right_pin, GPIO.HIGH)
            self._log.logAdd(f"MOTOR: Reverse at: {self._pwm_speed!s}%", True)
        except (OSError, RuntimeError, ValueError) as e:
            GPIO.cleanup()
            self._log.logAdd(f"MOTOR: Reverse caused error: {e}", True)

    def motorSmoothReverse(self):
        """Gradually bring stopped motors up to the configured reverse speed."""
        try:
            if not self._left_reverse:
                self._left_reverse = True
                GPIO.output(self._left_pin, GPIO.HIGH)
            if not self._right_reverse:
                self._right_reverse = True
                GPIO.output(self._right_pin, GPIO.HIGH)

            if not self._left_forward and not self._right_forward:
                step = round(self._pwm_speed / 5)
                currentPwmLevel = 100 - step
                self._left_pwm.start(currentPwmLevel)
                self._right_pwm.start(currentPwmLevel)
                sleep(0.1)
                for _ in range(3):
                    currentPwmLevel -= step
                    self._left_pwm.ChangeDutyCycle(currentPwmLevel)
                    self._right_pwm.ChangeDutyCycle(currentPwmLevel)
                    sleep(0.1)
                self._left_pwm.ChangeDutyCycle(100 - self._pwm_speed)
                self._right_pwm.ChangeDutyCycle(100 - self._pwm_speed)
                self._left_forward = True
                self._right_forward = True
                self._log.logAdd(f"MOTOR: Smooth reverse at: {self._pwm_speed!s}%", True)
        except (OSError, RuntimeError, ValueError) as e:
            GPIO.cleanup()
            self._log.logAdd(f"MOTOR: Smooth reverse caused error: {e}", True)

    def motorTurnLeft(self):
        """Turn left by driving the right motor and stopping the left motor."""
        try:
            if self._right_reverse:
                self._right_reverse = False
                GPIO.output(self._right_pin, GPIO.LOW)
            if not self._right_forward:
                self._right_forward = True
                self._right_pwm.start(self._pwm_speed)
            else:
                self._right_pwm.ChangeDutyCycle(self._pwm_speed)

            if self._left_reverse:
                self._left_reverse = False
                GPIO.output(self._left_pin, GPIO.LOW)
            if self._left_forward:
                self._left_forward = False
                self._left_pwm.ChangeDutyCycle(0)
            self._log.logAdd("MOTOR: Turning left", True)
        except (OSError, RuntimeError, ValueError) as e:
            GPIO.cleanup()
            self._log.logAdd(f"MOTOR: Turning left caused error: {e}", True)

    def motorTurnTightLeft(self):
        """Turn sharply left by running the motors in opposite directions."""
        try:
            inverse_speed = 100 - self._pwm_speed
            if self._right_reverse:
                self._right_reverse = False
                GPIO.output(self._right_pin, GPIO.LOW)
            if not self._right_forward:
                self._right_forward = True
                self._right_pwm.start(self._pwm_speed)
            else:
                self._right_pwm.ChangeDutyCycle(self._pwm_speed)

            if self._left_forward:
                self._left_pwm.ChangeDutyCycle(inverse_speed)
            else:
                self._left_forward = True
                self._left_pwm.start(inverse_speed)
            if not self._left_reverse:
                self._left_reverse = True
                GPIO.output(self._left_pin, GPIO.HIGH)
            self._log.logAdd("MOTOR: Turning tight left", True)
        except (OSError, RuntimeError, ValueError) as e:
            GPIO.cleanup()
            self._log.logAdd(f"MOTOR: Turning tight left caused error: {e}", True)

    def motorTurnRight(self):
        """Turn right by driving the left motor and stopping the right motor."""
        try:
            if self._left_reverse:
                self._left_reverse = False
                GPIO.output(self._left_pin, GPIO.LOW)
            if not self._left_forward:
                self._left_forward = True
                self._left_pwm.start(self._pwm_speed)
            else:
                self._left_pwm.ChangeDutyCycle(self._pwm_speed)

            if self._right_reverse:
                self._right_reverse = False
                GPIO.output(self._right_pin, GPIO.LOW)
            if self._right_forward:
                self._right_forward = False
                self._right_pwm.ChangeDutyCycle(0)
            self._log.logAdd("MOTOR: Turning right", True)
        except (OSError, RuntimeError, ValueError) as e:
            GPIO.cleanup()
            self._log.logAdd(f"MOTOR: Turning right caused error: {e}", True)

    def motorTurnTightRight(self):
        """Turn sharply right by running the motors in opposite directions."""
        try:
            inverse_speed = 100 - self._pwm_speed
            if self._left_reverse:
                self._left_reverse = False
                GPIO.output(self._left_pin, GPIO.LOW)
            if not self._left_forward:
                self._left_forward = True
                self._left_pwm.start(self._pwm_speed)
            else:
                self._left_pwm.ChangeDutyCycle(self._pwm_speed)

            if self._right_forward:
                self._right_pwm.ChangeDutyCycle(inverse_speed)
            else:
                self._right_forward = True
                self._right_pwm.start(inverse_speed)
            if not self._right_reverse:
                self._right_reverse = True
                GPIO.output(self._right_pin, GPIO.HIGH)
            self._log.logAdd("MOTOR: Turning tight right", True)
        except (OSError, RuntimeError, ValueError) as e:
            GPIO.cleanup()
            self._log.logAdd(f"MOTOR: Turning tight right caused error: {e}", True)

    def motorStop(self):
        """Immediately stop both motors."""
        try:
            GPIO.output(self._right_pin, GPIO.LOW)
            GPIO.output(self._left_pin, GPIO.LOW)
            self._left_pwm.ChangeDutyCycle(0)
            self._right_pwm.ChangeDutyCycle(0)
            self._left_forward = False
            self._left_reverse = False
            self._right_forward = False
            self._right_reverse = False
            self._log.logAdd("MOTOR: Both motors stopped", True)
        except (OSError, RuntimeError, ValueError) as e:
            GPIO.cleanup()
            self._log.logAdd(f"MOTOR: Stopping both motors caused error: {e}", True)

    def motorSmoothStop(self):
        """Gradually stop both motors from any state."""
        try:
            # reversing
            if self._left_reverse and self._right_reverse:
                inversePwmLevel = 100 - self._pwm_speed
                step = round(self._pwm_speed / 5)
                for _ in range(4):
                    inversePwmLevel += step
                    self._left_pwm.ChangeDutyCycle(inversePwmLevel)
                    self._right_pwm.ChangeDutyCycle(inversePwmLevel)
                    sleep(0.1)
                GPIO.output(self._right_pin, GPIO.LOW)
                GPIO.output(self._left_pin, GPIO.LOW)
                self._left_pwm.ChangeDutyCycle(0)
                self._right_pwm.ChangeDutyCycle(0)
                self._left_forward = False
                self._left_reverse = False
                self._right_forward = False
                self._right_reverse = False
                self._log.logAdd("MOTOR: Both motors smooth stopped from reverse", True)
            # tight turning left
            elif self._left_reverse and self._right_forward:
                currentPwmLevel = self._pwm_speed
                step = round(currentPwmLevel / 5)
                for i in range(1, 5):
                    hi = (100 - currentPwmLevel) + (step * i)
                    lo = currentPwmLevel - (step * i)
                    self._left_pwm.ChangeDutyCycle(hi)
                    self._right_pwm.ChangeDutyCycle(lo)
                    sleep(0.1)
                GPIO.output(self._right_pin, GPIO.LOW)
                GPIO.output(self._left_pin, GPIO.LOW)
                self._left_pwm.ChangeDutyCycle(0)
                self._right_pwm.ChangeDutyCycle(0)
                self._left_forward = False
                self._left_reverse = False
                self._right_forward = False
                self._right_reverse = False
                self._log.logAdd("MOTOR: Both motors smooth stopped from tight turn left", True)
            # tight turning right
            elif self._right_reverse and self._left_forward:
                currentPwmLevel = self._pwm_speed
                step = round(currentPwmLevel / 5)
                for i in range(1, 5):
                    hi = (100 - currentPwmLevel) + (step * i)
                    lo = currentPwmLevel - (step * i)
                    self._left_pwm.ChangeDutyCycle(lo)
                    self._right_pwm.ChangeDutyCycle(hi)
                    sleep(0.1)
                GPIO.output(self._right_pin, GPIO.LOW)
                GPIO.output(self._left_pin, GPIO.LOW)
                self._left_pwm.ChangeDutyCycle(0)
                self._right_pwm.ChangeDutyCycle(0)
                self._left_forward = False
                self._left_reverse = False
                self._right_forward = False
                self._right_reverse = False
                self._log.logAdd("MOTOR: Both motors smooth stopped from tight turn right", True)
            # forwards
            elif self._left_forward and self._right_forward:
                currentPwmLevel = self._pwm_speed
                step = round(currentPwmLevel / 5)
                for _ in range(4):
                    currentPwmLevel -= step
                    self._left_pwm.ChangeDutyCycle(currentPwmLevel)
                    self._right_pwm.ChangeDutyCycle(currentPwmLevel)
                    sleep(0.1)
                self._left_pwm.ChangeDutyCycle(0)
                self._right_pwm.ChangeDutyCycle(0)
                self._left_forward = False
                self._right_forward = False
                self._log.logAdd("MOTOR: Both motors smooth stopped from forwards", True)
            # turning left
            elif self._right_forward:
                currentPwmLevel = self._pwm_speed
                step = round(currentPwmLevel / 5)
                for _ in range(4):
                    currentPwmLevel -= step
                    self._right_pwm.ChangeDutyCycle(currentPwmLevel)
                    sleep(0.1)
                self._right_pwm.ChangeDutyCycle(0)
                self._right_forward = False
                self._log.logAdd("MOTOR: Right motors smooth stopped from left turn", True)
            # turning right
            elif self._left_forward:
                currentPwmLevel = self._pwm_speed
                step = round(currentPwmLevel / 5)
                for _ in range(4):
                    currentPwmLevel -= step
                    self._left_pwm.ChangeDutyCycle(currentPwmLevel)
                    sleep(0.1)
                self._left_pwm.ChangeDutyCycle(0)
                self._left_forward = False
                self._log.logAdd("MOTOR: Right motors smooth stopped from right turn", True)
        except (OSError, RuntimeError, ValueError) as e:
            GPIO.cleanup()
            self._log.logAdd(f"MOTOR: Smooth stopping both motors caused error: {e}", True)

    def motorIncreaseSpeed(self):
        """Increase the configured speed by ten and update motors."""
        if self._pwm_speed <= 90:
            self._pwm_speed += 10
        self._log.logAdd(f"MOTOR: PWM increased to: {self._pwm_speed!s}%", True)
        try:
            if self._left_forward and self._left_reverse:
                self._left_pwm.ChangeDutyCycle(100 - self._pwm_speed)
            elif self._left_forward:
                self._left_pwm.ChangeDutyCycle(self._pwm_speed)
            if self._right_forward and self._right_reverse:
                self._right_pwm.ChangeDutyCycle(100 - self._pwm_speed)
            elif self._right_forward:
                self._right_pwm.ChangeDutyCycle(self._pwm_speed)
        except (OSError, RuntimeError, ValueError) as e:
            GPIO.cleanup()
            self._log.logAdd(f"MOTOR: increasing motor speed caused error: {e}", True)

    def motorDecreasedSpeed(self):
        """Decrease the configured speed by ten and update motors."""
        if self._pwm_speed >= 20:
            self._pwm_speed -= 10
        self._log.logAdd(f"MOTOR: PWM decreased to: {self._pwm_speed!s}%", True)
        try:
            if self._left_forward and self._left_reverse:
                self._left_pwm.ChangeDutyCycle(100 - self._pwm_speed)
            elif self._left_forward:
                self._left_pwm.ChangeDutyCycle(self._pwm_speed)
            if self._right_forward and self._right_reverse:
                self._right_pwm.ChangeDutyCycle(100 - self._pwm_speed)
            elif self._right_forward:
                self._right_pwm.ChangeDutyCycle(self._pwm_speed)
        except (OSError, RuntimeError, ValueError) as e:
            GPIO.cleanup()
            self._log.logAdd(f"MOTOR: decreasing motor speed caused error: {e}", True)

    def motorShutdown(self):
        """Disable motor outputs, stop PWM and clean up GPIO resources."""
        try:
            GPIO.output(self._right_pin, GPIO.LOW)
            GPIO.output(self._left_pin, GPIO.LOW)
            self._left_pwm.stop()
            self._right_pwm.stop()
            self._left_forward = False
            self._left_reverse = False
            self._right_forward = False
            self._right_reverse = False
            self._log.logAdd("MOTOR: Both motors shut down and GPIO cleanup", True)
        finally:
            GPIO.cleanup()

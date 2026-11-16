"""Serializable command messages sent to the robot server."""


class CommandHandler:
    """Wrapper classes for a type of command design pattern implementation."""

    # Main commands
    class StartManual:
        """Request to enter manual mode."""

        def execute(self):
            """Do nothing locally, execution is handled by the server."""

    class QuitManual:
        """Request to leave manual mode."""

        def execute(self):
            """Do nothing locally, execution is handled by the server."""

    class StartAutonomous:
        """Request to enter autonomous mode."""

        def execute(self):
            """Do nothing locally, execution is handled by the server."""

    class QuitAutonomous:
        """Request to leave autonomous mode."""

        def execute(self):
            """Do nothing locally, execution is handled by the server."""

    class SetDestination:
        """Set a destination and its coordinate system."""

        def __init__(self, lat: float, long: float, usingGPS: bool):
            """Store destination coordinates and whether they use GPS."""
            self.lat = lat
            self.long = long
            self.usingGPS = usingGPS

        def execute(self):
            """Do nothing locally, execution is handled by the server."""

    class Ping:
        """Request a keepalive message."""

        def execute(self):
            """Do nothing locally, execution is handled by the server."""

    # Motor commands
    class MotorForward:
        """Request to drive forward."""

        def execute(self):
            """Do nothing locally, execution is handled by the server."""

    class MotorSmoothForward:
        """Request to accelerate forward gradually."""

        def execute(self):
            """Do nothing locally, execution is handled by the server."""

    class MotorReverse:
        """Request to drive in reverse."""

        def execute(self):
            """Do nothing locally, execution is handled by the server."""

    class MotorSmoothReverse:
        """Request to accelerate in reverse gradually."""

        def execute(self):
            """Do nothing locally, execution is handled by the server."""

    class MotorTurnLeft:
        """Request to turn left."""

        def execute(self):
            """Do nothing locally, execution is handled by the server."""

    class MotorTurnTightLeft:
        """Request to turn sharply left."""

        def execute(self):
            """Do nothing locally, execution is handled by the server."""

    class MotorTurnRight:
        """Request to turn right."""

        def execute(self):
            """Do nothing locally, execution is handled by the server."""

    class MotorTurnTightRight:
        """Request to turn sharply right."""

        def execute(self):
            """Do nothing locally, execution is handled by the server."""

    class MotorStop:
        """Request to stop immediately."""

        def execute(self):
            """Do nothing locally, execution is handled by the server."""

    class MotorSmoothStop:
        """Request to stop gradually."""

        def execute(self):
            """Do nothing locally, execution is handled by the server."""

    class MotorIncreaseSpeed:
        """Request to increase motor speed."""

        def execute(self):
            """Do nothing locally, execution is handled by the server."""

    class MotorDecreasedSpeed:
        """Request to decrease motor speed."""

        def execute(self):
            """Do nothing locally, execution is handled by the server."""

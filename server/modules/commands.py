"""Execute client commands on robot state and motor controls."""

import threading


class CommandHandler:
    """Wrapper classes for a type of command design pattern implementation."""

    def __init__(self, event, log, clientData, motorHandler):
        """Store shared services and create a lock for manual motor commands."""
        self._log = log
        self._clientData = clientData
        self._event = event
        self._motor = motorHandler
        self._lock = threading.Lock()  # in manual mode motor cmds can be accessed by different threads.

    # Main commands
    class StartManual:
        """Request to enter manual mode."""

        def execute(self, cmds):
            """Enable manual mode and wake the controller."""
            cmds._clientData.setManualActive(True)
            cmds._event.set()

    class QuitManual:
        """Request to leave manual mode."""

        def execute(self, cmds):
            """Clear the controller event and disable manual mode."""
            cmds._event.clear()
            cmds._clientData.setManualActive(False)

    class StartAutonomous:
        """Request to enter autonomous mode."""

        def execute(self, cmds):
            """Enable autonomous mode and wake the controller."""
            cmds._clientData.setAutonomousActive(True)
            cmds._event.set()

    class QuitAutonomous:
        """Request to leave autonomous mode."""

        def execute(self, cmds):
            """Clear the controller event and disable autonomous mode."""
            cmds._event.clear()
            cmds._clientData.setAutonomousActive(False)

    class SetDestination:
        """Set a destination and its coordinate system."""

        def __init__(self, lat: float, long: float, usingGPS: bool):
            """Store destination coordinates and whether they use GPS."""
            self.lat = lat
            self.long = long
            self.usingGPS = usingGPS

        def execute(self, cmds):
            """Store the client's destination and coordinate system."""
            if self.usingGPS:
                cmds._log.logAdd(f"COMMAND: Destination received, lat: {self.lat} / long: {self.long}", True)
            else:
                cmds._log.logAdd(f"COMMAND: Destination received, x: {self.lat} / y: {self.long}", True)
            cmds._clientData.setDestination(self.lat, self.long)
            cmds._clientData.setUsingGPS(self.usingGPS)

    class Ping:
        """Handle a client keepalive message."""

        def execute(self, cmds):
            """Record the latest client ping."""
            cmds._clientData.setPing()

    # Motor commands
    class MotorForward:
        """Request to drive forward."""

        def execute(self, cmds):
            """Run both motors forward."""
            cmds._log.logAdd("COMMAND: Forward command", True)
            with cmds._lock:
                cmds._motor.motorForward()

    class MotorSmoothForward:
        """Request to accelerate forward gradually."""

        def execute(self, cmds):
            """Gradually start forward motion."""
            cmds._log.logAdd("COMMAND: Smooth forward command", True)
            with cmds._lock:
                cmds._motor.motorSmoothForward()

    class MotorReverse:
        """Request to drive in reverse."""

        def execute(self, cmds):
            """Run both motors in reverse."""
            cmds._log.logAdd("COMMAND: Reverse command", True)
            with cmds._lock:
                cmds._motor.motorReverse()

    class MotorSmoothReverse:
        """Request to accelerate in reverse gradually."""

        def execute(self, cmds):
            """Gradually start reverse motion."""
            cmds._log.logAdd("COMMAND: Smooth reverse command", True)
            with cmds._lock:
                cmds._motor.motorSmoothReverse()

    class MotorTurnLeft:
        """Request to turn left."""

        def execute(self, cmds):
            """Turn left."""
            cmds._log.logAdd("COMMAND: Turn left command", True)
            with cmds._lock:
                cmds._motor.motorTurnLeft()

    class MotorTurnTightLeft:
        """Request to turn sharply left."""

        def execute(self, cmds):
            """Turn sharply left."""
            cmds._log.logAdd("COMMAND: Turn tight left command", True)
            with cmds._lock:
                cmds._motor.motorTurnTightLeft()

    class MotorTurnRight:
        """Request to turn right."""

        def execute(self, cmds):
            """Turn right."""
            cmds._log.logAdd("COMMAND: Turn right command", True)
            with cmds._lock:
                cmds._motor.motorTurnRight()

    class MotorTurnTightRight:
        """Request to turn sharply right."""

        def execute(self, cmds):
            """Turn sharply right."""
            cmds._log.logAdd("COMMAND: Turn tight right command", True)
            with cmds._lock:
                cmds._motor.motorTurnTightRight()

    class MotorStop:
        """Request to stop immediately."""

        def execute(self, cmds):
            """Stop both motors."""
            cmds._log.logAdd("COMMAND: Stop command", True)
            with cmds._lock:
                cmds._motor.motorStop()

    class MotorSmoothStop:
        """Request to stop gradually."""

        def execute(self, cmds):
            """Gradually stop both motors."""
            cmds._log.logAdd("COMMAND: Smooth stop command", True)
            with cmds._lock:
                cmds._motor.motorSmoothStop()

    class MotorIncreaseSpeed:
        """Request to increase motor speed."""

        def execute(self, cmds):
            """Increase motor speed."""
            cmds._log.logAdd("COMMAND: Increase speed command", True)
            with cmds._lock:
                cmds._motor.motorIncreaseSpeed()

    class MotorDecreasedSpeed:
        """Request to decrease motor speed."""

        def execute(self, cmds):
            """Decrease motor speed."""
            cmds._log.logAdd("COMMAND: Decrease speed command", True)
            with cmds._lock:
                cmds._motor.motorDecreasedSpeed()

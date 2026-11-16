"""Translate navigation data into motor commands."""

from math import atan2, degrees
from time import monotonic


class DriveHandler:
    """Drive between route nodes."""

    def __init__(self, log, motor, camera, clientData):
        """Initialize instance references to dependancies."""
        self._log = log
        self._motor = motor
        self._camera = camera
        self._clientData = clientData

    def _calcAngle(self, x1: float, y1: float, x2: float, y2: float) -> float:
        """Calculate the target heading in degrees between two Cartesian points."""
        angle = degrees(atan2((y2 - y1), (x2 - x1))) - 90
        if angle <= -180:
            angle += 360
        return angle * -1

    def driveToNode(self, targetX: float, targetY: float, timeLimit: float, shouldUpdateMesh: bool) -> int:
        """Steer toward a node until we arrive, halt on timeout or a safety issue."""
        start_time = monotonic()
        while monotonic() - start_time < timeLimit:
            if not self._clientData.getAutonomousActive():
                self._log.logAdd("DRIVER: Client has disabled autonomous mode", True)
                return 1

            if (monotonic() - self._clientData.getPing()) > 2:
                self._log.logAdd("DRIVER: Connection has timed out, going back to idle", True)
                return 1

            if shouldUpdateMesh:
                status = self._camera.cameraUpdateMesh()
                if status == 1:
                    return 1
            else:
                status = self._camera.cameraUpdatePose()
                if status == 1:
                    return 1

            if (
                self._camera.calcDistance(targetX, targetY, self._camera.poseData.getX(), self._camera.poseData.getY())
                <= 0.08
            ):  # within 8cm
                self._log.logAdd(f"DRIVER: We reached x={targetX}, y={targetY}", True)
                return 0

            if abs(self._camera.poseData.getPitch()) >= 20 or abs(self._camera.poseData.getRoll()) >= 20:
                self._log.logAdd(
                    f"DRIVER: Dangerous angle detected, pitch: {round(self._camera.poseData.getPitch(), 1)}, roll: {round(self._camera.poseData.getRoll(), 1)}",
                    True,
                )
                return 1

            currentYaw = self._camera.poseData.getYaw()
            targetYaw = self._calcAngle(self._camera.poseData.getX(), self._camera.poseData.getY(), targetX, targetY)

            diff = ((targetYaw - currentYaw + 540) % 360) - 180

            if abs(diff) > 75:  # if we are too far off we turn more aggressively
                self.turnTowardsNode(targetX, targetY, 7)
                continue

            currentPwmLevel = self._motor.motorGetPwmSpeed()
            hi = min(max(currentPwmLevel + self._exponentialSpeed(diff, (100 - currentPwmLevel)), 0), 100)
            lo = min(max(currentPwmLevel - self._exponentialSpeed(diff, currentPwmLevel), 0), 100)

            if diff >= 0:
                self._motor.motorSetLeftSpeed(hi)
                self._motor.motorSetRightSpeed(lo)
            else:
                self._motor.motorSetLeftSpeed(lo)
                self._motor.motorSetRightSpeed(hi)

            self._log.logAdd(
                f"DRIVER: tar: {round(targetYaw, 1)} - curr: {round(currentYaw, 1)} - diff: {round(diff, 1)} - hi: {hi}% - lo: {lo}% - cur_x: {self._camera.poseData.getX()} - cur_y: {self._camera.poseData.getY()}",
                True,
            )

        self._log.logAdd(f"DRIVER: We timed out trying to reach x={round(targetX, 1)}, y={round(targetY, 1)}", True)
        return 1

    def turnTowardsNode(self, targetX: float, targetY: float, timeLimit: float) -> int:
        """Turn towards a node until aligned."""
        start_time = monotonic()
        while monotonic() - start_time < timeLimit:
            if not self._clientData.getAutonomousActive():
                self._log.logAdd("DRIVER: Client has disabled autonomous mode", True)
                return 1

            if (monotonic() - self._clientData.getPing()) > 2:
                self._log.logAdd("DRIVER: Connection has timed out, going back to idle", True)
                return 1

            status = self._camera.cameraUpdatePose()
            if status == 1:
                return 1

            if abs(self._camera.poseData.getPitch()) >= 20 or abs(self._camera.poseData.getRoll()) >= 20:
                self._log.logAdd(
                    f"DRIVER: Dangerous angle detected, pitch: {round(self._camera.poseData.getPitch(), 1)}, roll: {round(self._camera.poseData.getRoll(), 1)}",
                    True,
                )
                return 1

            currentYaw = self._camera.poseData.getYaw()
            targetYaw = self._calcAngle(self._camera.poseData.getX(), self._camera.poseData.getY(), targetX, targetY)
            if abs(currentYaw - targetYaw) <= 6:  # we are within 6 degrees.
                self._log.logAdd(
                    f"DRIVER: We reached the target angle, tar: {round(targetYaw, 1)} - curr: {round(currentYaw, 1)}",
                    True,
                )
                self._motor.motorStop()
                return 0

            diff = ((targetYaw - currentYaw + 540) % 360) - 180

            pwm = 35
            if diff >= 0:
                self._motor.motorSetLeftSpeed(pwm)
                self._motor.motorSetReverseRightSpeed(pwm)
            else:
                self._motor.motorSetReverseLeftSpeed(pwm)
                self._motor.motorSetRightSpeed(pwm)

            self._log.logAdd(
                f"DRIVER: tar: {round(targetYaw, 1)} - curr: {round(currentYaw, 1)} - diff: {round(diff, 1)} - pwm: {pwm}% - cur_x: {self._camera.poseData.getX()} - cur_y: {self._camera.poseData.getY()}",
                True,
            )

        self._log.logAdd(f"DRIVER: We timed out trying to turn towards: x={targetX}, y={targetY}", True)
        return 1

    def calcNewRoute(self, targetX: float, targetY: float, endRange: int):
        """Request a route toward the target from the camera's mesh."""
        return self._camera.cameraUpdateRoute(targetX, targetY, endRange)  # this returns the path

    def _exponentialSpeed(self, diff: float, max: int) -> int:
        """Calculate a nonlinear speed adjustment from the heading error."""
        diff = abs(diff)
        # return round(((diff / 180.0) ** 0.7) * max) # ((x/180)^0.7)*max
        # return round(-abs(diff / (180 / (max * 2)) - max) + max)
        # return round(max*sin((diff/360)*(2*pi))) # TODO this can return negative values if diff is > 180 !!!
        # return round(((diff / 90.0) ** 0.8) * max)
        return round(((diff / 60.0) ** 0.9) * max)  # very aggressive

    def driveStop(self):
        """Bring the motors to a gradual stop."""
        self._motor.motorSmoothStop()

    def driveStart(self):
        """Gradually start both motors moving forward."""
        self._motor.motorSmoothForward()

    def driveSaveMesh(self, route):
        """Save the camera mesh with the recorded route."""
        self._camera.cameraSaveMesh(route)

    def driveUpdateMesh(self):
        """Refresh the camera's spatial mesh."""
        # TODO slow down motion to compensate for the delay?
        self._camera.cameraUpdateMesh()
        # TODO speed back up?

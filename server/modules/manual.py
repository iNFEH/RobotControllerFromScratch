"""Manual control mode operation."""

from time import monotonic, sleep


class ManualHandler:
    """Supervise manual driving and stop the robot when the mode ends."""

    def __init__(self, log, clientData, camera, commands):
        """Initialize instance references to some dependancies."""
        self._log = log
        self._clientData = clientData
        self._camera = camera
        self._cmds = commands

    def runManual(self):
        """Monitor connection timeouts and terrain tilt until manual driving ends."""
        while self._clientData.getManualActive():
            if (monotonic() - self._clientData.getPing()) > 2:
                break

            self._camera.cameraUpdatePose()

            if abs(self._camera.poseData.getPitch()) >= 20 or abs(self._camera.poseData.getRoll()) >= 20:
                self._log.logAdd(
                    f"MANUAL: Dangerous angle detected, pitch: {round(self._camera.poseData.getPitch(), 1)}, roll: {round(self._camera.poseData.getRoll(), 1)}",
                    True,
                )
                break

            sleep(0.5)  # no need to spam like mad

        self._clientData.setManualActive(False)
        self._cmds.MotorSmoothStop().execute(self._cmds)

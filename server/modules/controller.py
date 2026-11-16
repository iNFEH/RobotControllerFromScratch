"""Handles operating modes, the main loop and incoming commands."""

import pickle
import signal
import socket
import sys
import threading
from time import monotonic

from modules import autonomous, camera, commands, driver, gps, logger, manual, motor
import pyzed.sl as sl


class ControlHandler:
    """Initialize the robot and select manual or autonomous mode."""

    def __init__(self):  # NOTE do not touch order!
        """Initialize shutdown handling, hardware, commands and driving modes."""
        self._signalRegister()  # register sigint (ctrl+c)
        self._event = threading.Event()
        self._log = logger.LogHandler(False)  # this bool sets if we should save to disk or not
        self._log.logInit()
        self._motor = motor.MotorHandler(self._log)
        self._zed = sl.Camera()  # NOTE we create the camera object here so we can close it properly
        self._camera = camera.CameraHandler(self._log, self._zed)
        self._camera.cameraInit()
        self._gps = gps.GpsHandler(self._log, 5)
        self._gps.gpsInit()
        self._clientData = self.ClientData(self._log)
        self._driver = driver.DriveHandler(self._log, self._motor, self._camera, self._clientData)
        self._commands = commands.CommandHandler(self._event, self._log, self._clientData, self._motor)
        self._autonomousMode = autonomous.AutonomousHandler(self._log, self._gps, self._clientData, self._driver)
        self._manualMode = manual.ManualHandler(self._log, self._clientData, self._camera, self._commands)

    def _networkReceiver(self):
        """Receive serialized TCP commands and execute them with the command handler."""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            port = 12345  # not really relevant for local use, just pick one.
            # s.bind(('localhost', port))
            s.bind(("0.0.0.0", port))
            s.listen()
            local_ip = socket.gethostbyname(socket.gethostname())
            self._log.logAdd(f"NETWORK: Server started at {local_ip}:{port}", True)

            while True:
                conn, addr = s.accept()
                with conn:
                    self._log.logAdd(f"NETWORK: Received connection from {addr}", False)
                    serialized_command = conn.recv(1024)
                    command = pickle.loads(serialized_command)
                    command.execute(self._commands)

    # this runs the whole program, if we leave this the program shuts down
    def run(self):
        """Start the command receiver and primarly event loop."""
        # start the thread that handles all incoming network traffic from the client
        # NOTE this has no security at all right now!
        network_thread = threading.Thread(target=self._networkReceiver)
        network_thread.daemon = True
        network_thread.start()

        # this is the main loop of the whole program, this allows
        # for selection between manual and autonomous modes continuously
        while True:
            self._log.logAdd("CONTROLLER: Main thread waiting for mode select:", True)
            self._event.wait()
            self._log.logAdd("CONTROLLER: Main thread woke up!", True)

            if self._clientData.getManualActive():
                self._log.logAdd("CONTROLLER: Main thread entered manual mode!", True)
                self._clientData.setPing()
                self._manualMode.runManual()
                self._event.clear()
            elif self._clientData.getAutonomousActive():
                self._log.logAdd("CONTROLLER: Main thread entered autonomous mode!", True)
                self._clientData.setPing()
                self._autonomousMode.runAutonomous()
                self._event.clear()

    def _signalRegister(self):
        """Register the handler for the sigint singal."""
        signal.signal(signal.SIGINT, self._signalHandler)
        # NOTE if we need we can reg SIGTERM here as well

    def _signalHandler(self, sig, frame):
        """Shut down the motors and camera before exiting the process."""
        self._log.logAdd("CONTROLLER: CTRL-C detected, handling shutdown", True)
        self._motor.motorShutdown()
        self._zed.disable_spatial_mapping()
        self._zed.disable_positional_tracking()
        self._zed.close()
        sys.exit(0)

    class ClientData:
        """Store client data such as mode, destination and connection state."""

        def __init__(self, log):
            """Initialize client data."""
            self._log = log
            self._autonomousActive = False
            self._manualActive = False
            self._destination = None
            self._ping = 0.0
            self._usingGPS = False
            self._lock = threading.Lock()

        def setManualActive(self, active: bool):
            """Set whether manual mode is active."""
            with self._lock:
                self._manualActive = active

        def getManualActive(self) -> bool:
            """Return whether manual mode is active."""
            with self._lock:
                return self._manualActive

        def setAutonomousActive(self, active: bool):
            """Set whether autonomous mode is active."""
            with self._lock:
                self._autonomousActive = active

        def getAutonomousActive(self) -> bool:
            """Return whether autonomous mode is active."""
            with self._lock:
                return self._autonomousActive

        def setPing(self):
            """Store the latest client ping time."""
            with self._lock:
                self._ping = monotonic()
            self._log.logAdd(f"PING: Received ping at: {monotonic()}", False)

        def getPing(self) -> float:
            """Return the timestamp of the latest client ping."""
            with self._lock:
                return self._ping

        def setDestination(self, lat: float, long: float):
            """Store the client's destination."""
            with self._lock:
                self._destination = (lat, long)

        def getDestination(self) -> tuple[float, float] | None:
            """Return the stored destination."""
            with self._lock:
                return self._destination

        def setUsingGPS(self, active: bool):
            """Set whether destination coordinates use GPS."""
            with self._lock:
                self._usingGPS = active

        def getUsingGPS(self) -> bool:
            """Return whether destination coordinates use GPS."""
            with self._lock:
                return self._usingGPS

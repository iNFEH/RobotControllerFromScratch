"""Read GPS coordinates."""

from math import cos, radians, sin
import threading
from time import monotonic

import adafruit_gps
import serial

# USAGE
# pip install pyserial
# pip install adafruit-blinka
# pip install adafruit-circuitpython-gps
# sudo chmod a+rw /dev/ttyUSB0   # give yourself permission to access the usb port

# INFO
# https://learn.adafruit.com/circuitpython-libraries-on-linux-and-the-nvidia-jetson-nano/uart-serial
# https://pypi.org/project/adafruit-circuitpython-gps/
# https://learn.adafruit.com/adafruit-ultimate-gps?view=all

# SAMPLE DATA
# 58.064883 11.814953
# 58.064873 11.814971
# 58.064836 11.815006
# 58.064824 11.815026


class GpsHandler:
    """Manage GPS updates."""

    def __init__(self, log, updateRate: int):
        """Initialize coordinate storage and the GPS update interval."""
        self._log = log
        self._lock = threading.Lock()
        self._lat = None
        self._long = None
        self._updateRate = updateRate
        self._uart = None
        self._gps = None

    def gpsInit(self):
        """Configure the serial GPS receiver and start its update thread."""
        self._uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=10)
        self._gps = adafruit_gps.GPS(self._uart)
        self._gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        self._gps.send_command(b"PMTK220,1000")
        gps_thread = threading.Thread(target=self._gpsLoop)
        gps_thread.daemon = True
        gps_thread.start()

    def _gpsLoop(self):
        """Continuously read and store GPS data."""
        assert self._gps is not None
        self._log.logAdd("GPS: Loop starting (on it's own thread)", True)

        last = monotonic()
        while True:
            self._gps.update()

            current = monotonic()
            if current - last >= self._updateRate:
                last = current

                if not self._gps.has_fix:
                    self._log.logAdd("GPS: No sat fix, skipping loop", True)
                    continue

                lat = self._gps.latitude
                long = self._gps.longitude

                # write current gps position
                with self._lock:
                    self._lat = lat
                    self._long = long
                self._log.logAdd(f"GPS: lat: {lat}, long: {long}", True)

    def gpsRequestUpdate(self) -> tuple[float | None, float | None]:
        """Return the latest stored latitude and longitude."""
        with self._lock:
            return self._lat, self._long

    def gpsConvertToCartesian(self, lat1: float, long1: float, lat2: float, long2: float) -> tuple[float, float]:
        """Convert GPS coordinates to the Cartesian format."""
        R = 6371000  # radius of the earth in meters
        latInRads1 = radians(lat1)
        latInRads2 = radians(lat2)
        longInRads1 = radians(long1)
        longInRads2 = radians(long2)
        x1 = R * cos(latInRads1) * sin(longInRads1)
        y1 = R * cos(latInRads1) * cos(longInRads1)
        x2 = R * cos(latInRads2) * sin(longInRads2)
        y2 = R * cos(latInRads2) * cos(longInRads2)
        return (x2 - x1), (y1 - y2)

    def gpsGetLat(self):
        """Return the latest stored latitude."""
        with self._lock:
            return self._lat

    def gpsGetLong(self):
        """Return the latest stored longitude."""
        with self._lock:
            return self._long

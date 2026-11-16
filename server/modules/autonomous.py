"""Autonomous control mode operation."""

from time import sleep


class AutonomousHandler:
    """Route planning and autonomous driving."""

    def __init__(self, log, gps, clientData, driver):
        """Store navigation data and create an empty route."""
        self._log = log
        self._gps = gps
        self._clientData = clientData
        self._driver = driver
        self._route = self.Route()

    def runAutonomous(self):
        """Plan and follow the route toward the selected destination until driving ends."""
        targetInput1, targetInput2 = self._clientData.getDestination()

        # handle both cartesian and gps target coordinates
        targetX = None
        targetY = None
        if self._clientData.getUsingGPS():
            curLat = None
            curLong = None
            while not curLat and not curLong:
                if not self._clientData.getAutonomousActive():
                    return
                curLat = self._gps.gpsGetLat()
                curLong = self._gps.gpsGetLong()
                self._log.logAdd("AUTONOMOUS: Trying to establish our current gps position...", True)
                sleep(1)
            targetX, targetY = self._gps.gpsConvertToCartesian(curLat, curLong, targetInput1, targetInput2)
        else:
            targetX = targetInput1
            targetY = targetInput2

        # do our initial turn towards the target, this is from always starting facing northwards
        self._log.logAdd(f"AUTONOMOUS: Turning towards: x={targetX}, y={targetY}", True)
        status = self._driver.turnTowardsNode(targetX, targetY, 10)
        self._log.logAdd(f"AUTONOMOUS: Finished turning with status={status}", True)
        if status == 1:  # major error, we fastnade i vinkelvolten
            self._clientData.setAutonomousActive(False)
            self._driver.driveStop()
            return

        # this is the main loop of the autonomous mode
        while self._clientData.getAutonomousActive():
            self._log.logAdd(f"AUTONOMOUS: Calculating route(s) towards x={targetX}, y={targetY}", True)
            route = self._driver.calcNewRoute(
                targetX, targetY, 3 if self._clientData.getUsingGPS() else 1
            )  # using gps means larger acceptable target area

            if not route:  # we tried to find a route multiple ways but could not, major error
                self._log.logAdd("AUTONOMOUS: A* was unable to determine a viable route", True)
                break

            self._route.extendRoute(route)

            node = self._route.getNextNode()
            if node is None:
                break

            self._log.logAdd(f"AUTONOMOUS: Turning towards: x={node.getX()}, y={node.getY()}", True)
            status = self._driver.turnTowardsNode(node.getX(), node.getY(), 10)
            self._log.logAdd(f"AUTONOMOUS: Finished turning with status={status}", True)
            if status == 1:  # major error, we fastnade i vinkelvolten
                break

            counter = 1  # to avoid getting an update instantly on 0
            self._driver.driveStart()
            while True:
                if not node:  # route is empty (finished)
                    break

                self._log.logAdd(f"AUTONOMOUS: Entering normal drive towards: x={node.getX()}, y={node.getY()}", True)
                status = self._driver.driveToNode(node.getX(), node.getY(), 10, (counter % 6 == 0))
                self._log.logAdd(f"AUTONOMOUS: Exiting normal drive with status={status}", True)
                if status == 1:
                    self._clientData.setAutonomousActive(False)
                    self._driver.driveStop()
                    self._driver.driveSaveMesh(self._route)
                    self._route.clearRoute()
                    return
                node = self._route.getNextNode()
                counter += 1

            self._driver.driveStop()

        self._clientData.setAutonomousActive(False)
        self._driver.driveStop()  # we have left the main loop and should stop.
        self._driver.driveSaveMesh(self._route)  # save the mesh when we leave autonomous mode so we can analyze
        self._route.clearRoute()

    class Route:
        """Track route nodes and the next node to visit."""

        def __init__(self):
            """Create an empty route and set the pointer to 0."""
            self.route = []
            self._currentNode = 0

        def getNextNode(self):
            """Return the next node and advance the pointer."""
            if self._currentNode < len(self.route):
                point = self.route[self._currentNode]
                self._currentNode += 1
                return point

            return None

        def extendRoute(self, newRoute):
            """Append nodes to the existing route."""
            self.route.extend(newRoute)

        def alterRoute(self, newRoute):
            """Truncate the route after the pointer and append replacement nodes."""
            # this cuts away all points past the current one and adds new
            self.route = self.route[: self._currentNode + 1]
            self.route.extend(newRoute)

        def clearRoute(self):
            """Remove all route nodes and reset the pointer."""
            self.route.clear()
            self._currentNode = 0

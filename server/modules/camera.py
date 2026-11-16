"""Manage camera settings and mesh-based route planning."""

from collections import defaultdict
from functools import cache
from heapq import heappop, heappush
from math import atan2, degrees, sqrt
from pathlib import Path
from random import choice
from time import monotonic

import pyzed.sl as sl


class CameraHandler:
    """Uses the ZED camera to track, build meshes and create routes."""

    def __init__(self, log, zed):
        """Initialize instance references to dependancies."""
        self._log = log
        self._zed = zed
        self._mesh = None
        self._camera_pose = None
        self._runtime_parameters = None
        self.poseData = self.PoseData()

    def cameraInit(self):
        """Open the camera and configure positional tracking and spatial mapping."""
        init_params = sl.InitParameters()
        init_params.depth_mode = (
            sl.DEPTH_MODE.NEURAL
        )  # .PERFORMANCE .QUALITY .ULTRA .NEURAL (https://www.stereolabs.com/docs/depth-sensing/depth-settings)
        init_params.camera_resolution = sl.RESOLUTION.HD720  # .HD720 .HD1080 .HD1200
        init_params.camera_fps = 60  # 15,30,60
        init_params.coordinate_system = (
            sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP
        )  # .IMAGE .RIGHT_HANDED_Y_UP .RIGHT_HANDED_Z_UP
        init_params.coordinate_units = sl.UNIT.METER  # .MILLIMETER
        init_params.depth_maximum_distance = 3  # set the maximum depth perception distance to 3m
        init_params.depth_minimum_distance = 0.5  # set the minimum depth perception distance to 5dm

        # try to open the camera, exit if it fails
        status1 = self._zed.open(init_params)
        if status1 != sl.ERROR_CODE.SUCCESS:
            self._log.logAdd(f"CAMERA: Open {status1!r}. exit program.", True)
            self._zed.close()
            exit()

        # set parameters for Positional Tracking
        tracking_params = sl.PositionalTrackingParameters()
        tracking_params.enable_imu_fusion = True
        tracking_params.set_gravity_as_origin = True
        tracking_params.set_floor_as_origin = False  # this has to be false, why?
        status2 = self._zed.enable_positional_tracking(tracking_params)  # enable Positional Tracking
        if status2 != sl.ERROR_CODE.SUCCESS:
            self._log.logAdd(f"CAMERA: Enable positional tracking: {status2!r}. exit program.", True)
            self._zed.close()
            exit()

        # set parameters for Spatial Mapping
        spatial_mapping_parameters = sl.SpatialMappingParameters()
        # spatial_mapping_parameters.resolution_meter = spatial_mapping_parameters.get_resolution_preset(sl.MAPPING_RESOLUTION.LOW) # low=8cm, medium=5cm, high=2cm
        spatial_mapping_parameters.resolution_meter = 0.15  # set resolution to 15cm
        # spatial_mapping_parameters.range_meter = spatial_mapping_parameters.get_range_preset(sl.MAPPING_RANGE.MEDIUM) # this can simply be an integer also
        spatial_mapping_parameters.range_meter = 3  # 3m
        spatial_mapping_parameters.max_memory_usage = 1024  # 512 seems to occasionally cause issues with empty mesh
        spatial_mapping_parameters.save_texture = False
        spatial_mapping_parameters.use_chunk_only = (
            True  # "This parameter allows to disable mesh update and speed up applications which only use chunk lists."
        )
        spatial_mapping_parameters.reverse_vertex_order = (
            False  # "This parameter allows to retrieve mesh vertices in clockwise or counterclockwise order."
        )
        spatial_mapping_parameters.map_type = sl.SPATIAL_MAP_TYPE.MESH
        status3 = self._zed.enable_spatial_mapping(spatial_mapping_parameters)
        if status3 != sl.ERROR_CODE.SUCCESS:
            self._log.logAdd(f"CAMERA: Enable spatial mapping: {status2!r}. exit program.", True)
            self._zed.close()
            exit()

        # set runtime parameters
        self._runtime_parameters = sl.RuntimeParameters()
        self._runtime_parameters.confidence_threshold = 50
        # self._runtime_parameters.enable_fill_mode = True

        # create the mesh and pose objects
        self._mesh = sl.Mesh()
        self._camera_pose = sl.Pose()

    def cameraUpdateRoute(self, targetX: float, targetY: float, endRange: int):
        """Refresh the mesh and search for a route toward the target."""
        assert self._mesh is not None

        self.calcDistance.cache_clear()  # this clears the memoize cache
        self._angleFilter.cache_clear()
        self._calcDistanceVertexToLine.cache_clear()
        self.cameraUpdateMesh()  # mesh and pose update

        # this is the check to see if we are done with the trip
        if self.calcDistance(self.poseData.getX(), self.poseData.getY(), targetX, targetY) < endRange:
            return None

        currentX = self.poseData.getX()
        currentY = self.poseData.getY()

        startVertex = self._getClosestVertexToPoint(currentX, currentY)
        startNode = self.Node(startVertex, self._mesh.vertices[startVertex])

        adjacencyList = self._buildAdjacencyList()  # calculate all the neighbors of nodes

        for multiplier in range(1, 6):  # 5x times
            checkRange = 0.4 * multiplier  # A* has to be within this distance to a node

            # can we set a course straight towards the end goal?
            goalVertex = self._getClosestVertexToPoint(targetX, targetY)
            if (
                self.calcDistance(
                    startNode.coords[0],
                    startNode.coords[1],
                    self._mesh.vertices[goalVertex, 0],
                    self._mesh.vertices[goalVertex, 1],
                )
                > 1
            ):
                goalNode = self.Node(goalVertex, self._mesh.vertices[goalVertex])
                print("[1]\nstart:", startNode.coords, "\ngoal:", goalNode.coords)  # TODO remove this later
                route = self._aStar(startNode, goalNode, adjacencyList, checkRange)
                if route:
                    return route

            # we couldn't set a course straight towards the end goal, instead check a perpendicular side
            checkLeft = choice([True, False])
            goalVertex = self._getFurthestVertexPerpendicularToLine(currentX, currentY, targetX, targetY, checkLeft)
            if (
                self.calcDistance(
                    startNode.coords[0],
                    startNode.coords[1],
                    self._mesh.vertices[goalVertex, 0],
                    self._mesh.vertices[goalVertex, 1],
                )
                > 1
            ):
                goalNode = self.Node(goalVertex, self._mesh.vertices[goalVertex])
                print("[2]\nstart:", startNode.coords, "\ngoal:", goalNode.coords)  # TODO remove this later
                route = self._aStar(startNode, goalNode, adjacencyList, checkRange)
                if route:
                    return route

            # if that did not work, check the other perpendicular side
            goalVertex = self._getFurthestVertexPerpendicularToLine(currentX, currentY, targetX, targetY, not checkLeft)
            if (
                self.calcDistance(
                    startNode.coords[0],
                    startNode.coords[1],
                    self._mesh.vertices[goalVertex, 0],
                    self._mesh.vertices[goalVertex, 1],
                )
                > 1
            ):
                goalNode = self.Node(goalVertex, self._mesh.vertices[goalVertex])
                print("[3]\nstart:", startNode.coords, "\ngoal:", goalNode.coords)  # TODO remove this later
                route = self._aStar(startNode, goalNode, adjacencyList, checkRange)
                if route:
                    return route

        # TODO we could go towards our start or something? to get a picture that way

        return None  # we could not find anywhere to go, major error

    def cameraUpdateMesh(self) -> int:
        """Refresh the spatial mesh and update available pose data."""
        assert self._mesh is not None
        assert self._camera_pose is not None

        last = monotonic()
        while (monotonic() - last) < 1:
            if self._zed.grab(self._runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                tracking_state = self._zed.get_position(self._camera_pose, sl.REFERENCE_FRAME.WORLD)
                if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                    position = self._camera_pose.get_translation(sl.Translation())
                    self.poseData.setX(position.get()[0])
                    self.poseData.setY(position.get()[1])
                    rotation = self._camera_pose.get_rotation_vector()
                    self.poseData.setYaw(degrees(rotation[2] * -1))
                    self.poseData.setPitch(degrees(rotation[0]))
                    self.poseData.setRoll(degrees(rotation[1]))

                if (monotonic() - last) > 0.5:  # update rate
                    self._zed.request_spatial_map_async()
                    last = monotonic()

                if self._zed.get_spatial_map_request_status_async() == sl.ERROR_CODE.SUCCESS:
                    self._zed.retrieve_spatial_map_async(self._mesh)
                    self._mesh.update_mesh_from_chunklist()
                    return 0

        self._log.logAdd("CAMERA: Update mesh failed for 1 second, major error", True)
        return 1

    def cameraUpdatePose(self) -> int:
        """Refresh the stored camera position and orientation."""
        assert self._camera_pose is not None

        last = monotonic()
        while (monotonic() - last) < 1:
            if self._zed.grab(self._runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                tracking_state = self._zed.get_position(self._camera_pose, sl.REFERENCE_FRAME.WORLD)
                if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                    position = self._camera_pose.get_translation(sl.Translation())
                    self.poseData.setX(position.get()[0])
                    self.poseData.setY(position.get()[1])
                    rotation = self._camera_pose.get_rotation_vector()
                    self.poseData.setYaw(degrees(rotation[2] * -1))
                    self.poseData.setPitch(degrees(rotation[0]))
                    self.poseData.setRoll(degrees(rotation[1]))
                    return 0

        self._log.logAdd("CAMERA: Update pose failed for 1 second, major error", True)
        return 1

    def cameraSaveMesh(self, route):
        """Save source and route-highlighted meshes."""
        assert self._mesh is not None

        self._zed.extract_whole_spatial_map(self._mesh)
        status = self._mesh.save("autonomousModeMesh_source.obj")
        if not status:
            self._log.logAdd(f"CAMERA: Saving the mesh failed with status: {status}", True)
        else:
            self._cameraWriteCustomObj(route)
            self._log.logAdd("CAMERA: Saved both source and projected meshes", True)
        self._mesh.clear()

    def _cameraWriteCustomObj(self, route):
        """Write a .OBJ mesh file with route vertices highlighted in red."""
        assert self._mesh is not None

        vertices = self._mesh.vertices
        colors = self._mesh.colors
        normals = self._mesh.normals
        path = route.route

        def pathContains(i):
            """Return True if the route contains the given vertex ID."""
            return any(node.id == i for node in path)

        with Path("autonomousModeMesh_projected.obj").open("w") as f:
            f.write("#Generated by the ugliest hack eu.\n")

            for i in range(len(vertices)):
                if pathContains(i):
                    f.write(
                        f"v {vertices[i, 0]:.6f} {vertices[i, 1]:.6f} {vertices[i, 2]:.6f} {1:.6f} {0:.6f} {0:.6f}\n"
                    )
                else:
                    f.write(
                        f"v {vertices[i, 0]:.6f} {vertices[i, 1]:.6f} {vertices[i, 2]:.6f} {colors[i, 2] / 255:.6f} {colors[i, 1] / 255:.6f} {colors[i, 0] / 255:.6f}\n"
                    )

            for normal in normals:
                f.write(f"vn {normal[0]:.6f} {normal[1]:.6f} {normal[2]:.6f}\n")

        lines = None
        with Path("autonomousModeMesh_source.obj").open() as source:
            lines = source.readlines()

        with Path("autonomousModeMesh_projected.obj").open("a") as dest:
            for line in lines:
                if line.startswith("f "):
                    dest.write(line)

    @cache
    def _calcDistanceVertexToLine(
        self, currentX: float, currentY: float, startX: float, startY: float, targetX: float, targetY: float
    ) -> float:
        """Calculate the perpendicular distance from a point to a line."""
        a = targetY - startY
        b = startX - targetX
        c = targetX * startY - startX * targetY

        return abs(a * currentX + b * currentY + c) / sqrt(a**2 + b**2)

    def _getFurthestVertexPerpendicularToLine(
        self, startX: float, startY: float, targetX: float, targetY: float, checkLeft: bool
    ) -> int | None:
        """Find the valid vertex farthest from a line on the selected side."""
        assert self._mesh is not None

        filteredVertices = [
            (vertex_id, vertex)
            for vertex_id, vertex in enumerate(self._mesh.vertices)
            if self._angleFilter(
                self._mesh.normals[vertex_id, 0], self._mesh.normals[vertex_id, 1], self._mesh.normals[vertex_id, 2]
            )
        ]

        maxDistance = 0  # TODO use max() instead
        furthestId = None

        for vertex_id, vertex in filteredVertices:
            currentX = vertex[0]
            currentY = vertex[1]
            signedDistance = (currentX - startX) * (targetY - startY) - (currentY - startY) * (targetX - startX)
            distance = self._calcDistanceVertexToLine(currentX, currentY, startX, startY, targetX, targetY)

            if (checkLeft and signedDistance < 0) or (not checkLeft and signedDistance > 0):
                if distance > maxDistance:
                    maxDistance = distance
                    furthestId = vertex_id

        return furthestId

    @cache
    def calcDistance(self, x1: float, y1: float, x2: float, y2: float) -> float:
        """Calculate the euclidean distance between two points."""
        return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    @cache
    def _angleFilter(self, x: float, y: float, z: float) -> bool:
        """Check whether a surface normal (3d) has an acceptable slope."""
        pitchAngle = abs(degrees(atan2(y, z)))
        rollAngle = abs(degrees(atan2(x, z)))
        return pitchAngle < 25 and rollAngle < 25 and z > 0

    def _getClosestVertexToPoint(self, targetX: float, targetY: float) -> int:
        """Find the valid mesh vertex nearest to the target point."""
        mesh = self._mesh
        assert mesh is not None

        filtered_indices = [
            i for i, normal in enumerate(mesh.normals) if self._angleFilter(normal[0], normal[1], normal[2])
        ]
        return min(
            filtered_indices,
            key=lambda i: self.calcDistance(targetX, targetY, mesh.vertices[i, 0], mesh.vertices[i, 1]),
        )  # NOTE can be None

    def _buildAdjacencyList(self):
        """Build vertex neighbor lists from the mesh."""
        assert self._mesh is not None

        adjacencyList = defaultdict(list)

        for face in self._mesh.triangles:
            for i, v1 in enumerate(face):
                v2 = face[(i + 1) % len(face)]  # wrap around for last vertex
                adjacencyList[int(v1)].append(int(v2))

        return adjacencyList

    def _aStar(
        self,
        startNode: CameraHandler.Node,
        goalNode: CameraHandler.Node,
        adjacencyList: dict[int, list[int]],
        range: float,
    ):
        """Search for a traversable path within the given distance of the goal using A*."""
        assert self._mesh is not None

        openSet = []  # nodes left to check ("list" to maintain order)
        closedSet = set()  # visited nodes ("set" since order does not matter)

        heappush(openSet, startNode)  # this is basically the same as a priority queue

        while openSet:  # as long as the openSet is not empty
            currentNode = heappop(openSet)  # pop the top node from the heap

            if (
                self.calcDistance(currentNode.coords[0], currentNode.coords[1], goalNode.coords[0], goalNode.coords[1])
                < range
            ):
                path = []
                while currentNode:
                    path.append(currentNode)
                    currentNode = currentNode.parent
                return path[::-1]  # this returns the list in reverse (which is correct)

            closedSet.add(currentNode.id)  # node has been visited

			# sanity check
            if len(closedSet) > 300:
                self._log.logAdd("CAMERA: A* added too many nodes to closed set, likely no path", True)
                return None

            # generate neighbors
            for neighborId in adjacencyList[currentNode.id]:
                if neighborId in closedSet:
                    continue

                # check that each node has a reasonable surface normal
                if not self._angleFilter(
                    self._mesh.normals[neighborId, 0],
                    self._mesh.normals[neighborId, 1],
                    self._mesh.normals[neighborId, 2],
                ):
                    continue

                # if a node has a neighbor node that is invalid, we skip the node (too narrow).
                skipNode = False
                for neighborOfNeighborId in adjacencyList[neighborId]:
                    if neighborOfNeighborId in closedSet:
                        continue
                    if not self._angleFilter(
                        self._mesh.normals[neighborOfNeighborId, 0],
                        self._mesh.normals[neighborOfNeighborId, 1],
                        self._mesh.normals[neighborOfNeighborId, 2],
                    ):
                        skipNode = True
                        break
                if skipNode:
                    continue

                # calculate the cost
                neighborCoords = self._mesh.vertices[neighborId]
                neighborCost = currentNode.cost + self.calcDistance(
                    currentNode.coords[0], currentNode.coords[1], neighborCoords[0], neighborCoords[1]
                )
                heuristicCost = self.calcDistance(
                    neighborCoords[0], neighborCoords[1], goalNode.coords[0], goalNode.coords[1]
                )
                totalCost = neighborCost + heuristicCost  # total estimated cost

                neighbor = self.Node(neighborId, neighborCoords, totalCost, currentNode)  # create our new node object
                if neighbor not in openSet:
                    heappush(openSet, neighbor)
                elif totalCost < neighbor.cost:  # update the cost and parent of the neighbor
                    neighbor.cost = totalCost
                    neighbor.parent = currentNode

        return None  # no path found

    # a class to consolidate pose data
    class PoseData:
        """Store the robot's position and orientation data."""

        def __init__(self):
            """Initialize all position and orientation values."""
            self._x = 0
            self._y = 0
            self._yaw = 0
            self._pitch = 0
            self._roll = 0

        def getX(self) -> float:
            """Return the stored X coordinate."""
            return self._x

        def getY(self) -> float:
            """Return the stored Y coordinate."""
            return self._y

        def getYaw(self) -> float:
            """Return the stored yaw angle in degrees."""
            return self._yaw

        def getPitch(self) -> float:
            """Return the stored pitch angle in degrees."""
            return self._pitch

        def getRoll(self) -> float:
            """Return the stored roll angle in degrees."""
            return self._roll

        def setX(self, x: float):
            """Store the X coordinate."""
            self._x = x

        def setY(self, y: float):
            """Store the Y coordinate."""
            self._y = y

        def setYaw(self, yaw: float):
            """Store the yaw angle in degrees."""
            self._yaw = yaw

        def setPitch(self, pitch: float):
            """Store the pitch angle in degrees."""
            self._pitch = pitch

        def setRoll(self, roll: float):
            """Store the roll angle in degrees."""
            self._roll = roll

    class Node:
        """A mesh vertex and its pathfinding cost and parent."""

        def __init__(self, id, coords, cost=0, parent=None):
            """Initialize the vertex identity, coordinate, cost and parent node."""
            self.id = id
            self.coords = coords
            self.cost = cost
            self.parent = parent

        def __eq__(self, other):  # equals
            """Compare nodes by vertex Id."""
            return self.id == other.id

        def __lt__(self, other):  # less than for the comparator
            """Order nodes by pathfinding cost."""
            return self.cost < other.cost

        def __hash__(self):  # hash code
            """Hash the node's vertex Id and coordinates."""
            return hash((self.id, tuple(self.coords)))

        def getX(self) -> float:
            """Return the vertex's X coordinate."""
            return self.coords[0]

        def getY(self) -> float:
            """Return the vertex's Y coordinate."""
            return self.coords[1]

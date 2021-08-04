
from bosdyn.client import create_standard_sdk, ResponseError, RpcError

from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive


class SpotArmWrapper():
    """Generic wrapper class to encompass release 1.1.4 API features as well as maintaining leases automatically"""
    def __init__(self, username, password, hostname, logger, rates = {}, callbacks = {}):
        self._username = username
        self._password = password
        self._hostname = hostname
        self._logger = logger
        self._rates = rates
        self._callbacks = callbacks
        self._keep_alive = True
        self._valid = True

        self._mobility_params = RobotCommandBuilder.mobility_params()
        self._is_standing = False
        self._is_sitting = True
        self._is_moving = False
        self._at_goal = False
        self._last_stand_command = None
        self._last_sit_command = None
        self._last_trajectory_command = None
        self._last_velocity_command_time = None

        try:
            self._sdk = create_standard_sdk('ros_spot')
        except Exception as e:
            self._logger.error("Error creating SDK object: %s", e)
            self._valid = False
            return

        self._robot = self._sdk.create_robot(self._hostname)

        try:
            self._robot.authenticate(self._username, self._password)
            self._robot.start_time_sync()
        except RpcError as err:
            self._logger.error("Failed to communicate with robot: %s", err)
            self._valid = False
            return

        if self._robot:
        #     # Clients
        #     try:
        #         self._robot_state_client = self._robot.ensure_client(RobotStateClient.default_service_name)
        #         self._robot_command_client = self._robot.ensure_client(RobotCommandClient.default_service_name)
        #         self._graph_nav_client = self._robot.ensure_client(GraphNavClient.default_service_name)
        #         self._power_client = self._robot.ensure_client(PowerClient.default_service_name)
            self._lease_client = self._robot.ensure_client(LeaseClient.default_service_name)
        #         self._lease_wallet = self._lease_client.lease_wallet
        #         self._image_client = self._robot.ensure_client(ImageClient.default_service_name)
            self._estop_client = self._robot.ensure_client(EstopClient.default_service_name)
        #     except Exception as e:
        #         self._logger.error("Unable to create client service: %s", e)
        #         self._valid = False
        #         return

        #     # Store the most recent knowledge of the state of the robot based on rpc calls.
        #     self._current_graph = None
        #     self._current_edges = dict()  #maps to_waypoint to list(from_waypoint)
        #     self._current_waypoint_snapshots = dict()  # maps id to waypoint snapshot
        #     self._current_edge_snapshots = dict()  # maps id to edge snapshot
        #     self._current_annotation_name_to_wp_id = dict()

        #     # Async Tasks
        #     self._async_task_list = []
        #     self._robot_state_task = AsyncRobotState(self._robot_state_client, self._logger, max(0.0, self._rates.get("robot_state", 0.0)), self._callbacks.get("robot_state", lambda:None))
        #     self._robot_metrics_task = AsyncMetrics(self._robot_state_client, self._logger, max(0.0, self._rates.get("metrics", 0.0)), self._callbacks.get("metrics", lambda:None))
        #     self._lease_task = AsyncLease(self._lease_client, self._logger, max(0.0, self._rates.get("lease", 0.0)), self._callbacks.get("lease", lambda:None))
        #     self._front_image_task = AsyncImageService(self._image_client, self._logger, max(0.0, self._rates.get("front_image", 0.0)), self._callbacks.get("front_image", lambda:None), self._front_image_requests)
        #     self._side_image_task = AsyncImageService(self._image_client, self._logger, max(0.0, self._rates.get("side_image", 0.0)), self._callbacks.get("side_image", lambda:None), self._side_image_requests)
        #     self._rear_image_task = AsyncImageService(self._image_client, self._logger, max(0.0, self._rates.get("rear_image", 0.0)), self._callbacks.get("rear_image", lambda:None), self._rear_image_requests)
        #     self._idle_task = AsyncIdle(self._robot_command_client, self._logger, 10.0, self)

        #     self._estop_endpoint = None

        #     self._async_tasks = AsyncTasks(
        #         [self._robot_state_task, self._robot_metrics_task, self._lease_task, self._front_image_task, self._side_image_task, self._rear_image_task, self._idle_task])

        #     self._robot_id = None
        #     self._lease = None

    def claim(self):
        """Get a lease for the robot, a handle on the estop endpoint, and the ID of the robot."""
        try:
            self._robot_id = self._robot.get_id()
            self.getLease()
            self.resetEStop()
            return True, "Success"
        except (ResponseError, RpcError) as err:
            self._logger.error("Failed to initialize robot communication: %s", err)
            return False, str(err)

    def getLease(self):
        """Get a lease for the robot and keep the lease alive automatically."""
        self._lease = self._lease_client.acquire()
        self._lease_keepalive = LeaseKeepAlive(self._lease_client)

    def resetEStop(self):
        """Get keepalive for eStop"""
        self._estop_endpoint = EstopEndpoint(self._estop_client, 'ros', 9.0)
        self._estop_endpoint.force_simple_setup()  # Set this endpoint as the robot's sole estop.
        self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)
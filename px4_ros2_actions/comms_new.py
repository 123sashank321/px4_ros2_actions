#!/usr/bin/env python3
"""
Developed by : AI Team (Skytex)
Version : 1.0.0
Date : 4th Jul 2025 
Description : This script handles communication with GCS (telem, dive, px, lock, mission commands).
              In this position heartbeat is published in order to send position setpoints to the autopilot
"""

import rclpy
from rclpy.node import Node
import json
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, \
    DurabilityPolicy
import numpy as np
import time
from std_msgs.msg import String
from math import nan
from px4_msgs.msg import VehicleGlobalPosition, VehicleStatus, \
    OffboardControlMode, VehicleCommand, TrajectorySetpoint, VehicleOdometry, SensorGps, BatteryStatus, RadioStatus, \
    VehicleAttitude, LogMessage, VehicleAttitudeSetpoint, AirspeedValidated, HoverThrustEstimate, VehicleLandDetected, \
    PositionSetpointTriplet, PositionSetpoint, ManualControlSwitches, TecsStatus
import math
import utm
from vtol_stack.helper import UavHelper


class Comms(Node):
    def __init__(self,):
        super().__init__("ghost_comms_node")

        # instantiate helper to run basic func (dist, latlon_to_m, etc)
        self.helper = UavHelper()
        self.hostname = self.helper.get_host_ip()
        self.rtsp_address = f"rtsp://{self.hostname}:8554/stream1"

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.load_ghost_params()

# ---------------------------------------------------------------------------------------- subscribers (px4 and gcs subscribers)

        self.vehicle_airspeed_subscriber = self.create_subscription(AirspeedValidated,
                                                                    '/fmu/out/airspeed_validated',
                                                                    self.vehicle_airspeed_callback,
                                                                    qos_profile)

        self.battery_status_subscriber = self.create_subscription(BatteryStatus,
                                                                  '/fmu/out/battery_status',
                                                                  self.battery_cb,
                                                                  qos_profile)

        self.vehicle_odo_subscriber = self.create_subscription(VehicleOdometry,
                                                               f'/fmu/out/vehicle_odometry',
                                                               self.vehicle_odo_callback,
                                                               qos_profile)

        self.vehicle_global_position_subscriber = self.create_subscription(VehicleGlobalPosition,
                                                                           f'/fmu/out/vehicle_global_position',
                                                                           self.vehicle_global_position_callback,
                                                                           qos_profile)

        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus,
                                                                  f'/fmu/out/vehicle_status',
                                                                  self.vehicle_status_callback,
                                                                  qos_profile)

        self.vehicle_att_subscriber = self.create_subscription(VehicleAttitude,
                                                               f'/fmu/out/vehicle_attitude',
                                                               self.vehicle_att_callback,
                                                               qos_profile)

        self.land_detection_subscription = self.create_subscription(VehicleLandDetected,
                                                                    '/fmu/out/vehicle_land_detected',
                                                                    self.land_callback,
                                                                    qos_profile)
        self.rc_subscriber = self.create_subscription(ManualControlSwitches,
                                                      '/fmu/out/manual_control_switches',
                                                      self.rc_callback,
                                                      qos_profile)

        self.tecs_subscriber = self.create_subscription(TecsStatus,
                                                        '/fmu/out/tecs_status',
                                                        self.tecs_callback,
                                                        qos_profile)

        self.gcs_subscriber = self.create_subscription(String,
                                                       "cpp_to_python",
                                                       self.from_gcs_cb,
                                                       10)

        self.rtsp_subscriber = self.create_subscription(String,
                                                        "/rtsp_status",
                                                        self.rtsp_cb,
                                                        10)

        self.alt_change_subscriber = self.create_subscription(String,
                                                              "/alt_change",
                                                              self.alt_change_cb,
                                                              10)

        self.pixels_from_gcs_subscriber = self.create_subscription(String,
                                                                   "/pixels_from_gcs",
                                                                   self.pixels_from_gcs_cb,
                                                                   10)

        self.lock_from_gcs_subscriber = self.create_subscription(
            String,
            "lock_from_gcs",
            self.lock_from_gcs_cb,
            10)

        self.drive_from_gcs_subscriber = self.create_subscription(
            String,
            "dive_from_gcs",
            self.dive_from_gcs_cb,
            10)
        self.recovery_info_subscriber = self.create_subscription(
            String,
            "recovery_info",
            self.recovery_info_cb,
            10)

# ---------------------------------------------------------------------------------------- publishers (to_px4 and other scripts publishers)

        self.vehicle_attitude_setpoint_publisher = self.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)

        self.publish_position_setpoint_triplet_publisher = self.create_publisher(
            PositionSetpointTriplet, "/fmu/in/position_setpoint_triplet", qos_profile)

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)

        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        self.gcs_publisher = self.create_publisher(String, "python_to_cpp", 10)

        self.pixel_publisher = self.create_publisher(
            String, "pixel_values", 10)

        self.lock_publisher = self.create_publisher(String, "lock_values", 10)

        self.dive_publisher = self.create_publisher(String, "dive_values", 10)

        self.command_publisher = self.create_publisher(
            String, "command_publisher", 10)

# ---------------------------------------------------------------------------------------- PX4 Message Objects

        self.pos_setpoint_trip = PositionSetpointTriplet()
        self.command_from_gcs = String()
        self.pixel_gcs = String()
        self.lock_gcs = String()
        self.dive_gcs = String()
        self.pixel_values = String()
        self.lock_value = String()
        self.dive_value = String()
        self.strike_routine = String()
        self.vehicle_airspeed = AirspeedValidated()
        self.vehicle_status = VehicleStatus()
        self.vehicle_global_position = VehicleGlobalPosition()
        self.vehicle_odometry = VehicleOdometry()
        self.radio_status = RadioStatus()
        self.battery = BatteryStatus()
        self.vehicle_land = VehicleLandDetected()
        self.vehicle_att = VehicleAttitude()
        self.rc_modes = ManualControlSwitches()
        self.rtsp = {"address": self.rtsp_address, "status": False}
        self.is_pre_mission_updated = False
        self.new_pre_mission = None
        self.new_pre_mission_ctr = 0
        self.rc_switch_init = False
        self.previous_switch = 0
        self.is_in_rc_control = 0

        self.timer = self.create_timer(0.1, self.timer_cb)

# ---------------------------------------------------------------------------------------- Load Params

    def load_ghost_params(self):
        with open("./src/vtol_stack/config_files/ghost_config.json", 'r') as file:
            ghost_params = json.load(file)
        file.close()

        home_params = ghost_params["home_params"]
        self.home_lat = home_params["home_lat"]
        self.home_lon = home_params["home_lon"]
        self.home_alt = home_params["home_alt"]
        self.home_flag = home_params["home_flag"]

        vehicle_flags = ghost_params["vehicle_flags"]
        self.arming_flag = vehicle_flags["arming_flag"]
        self.takeoff_flag = vehicle_flags["takeoff_flag"]
        self.offboard_flag = vehicle_flags["offboard_flag"]
        self.takeoff_height = vehicle_flags["takeoff_height"]
        self.transition_flag = vehicle_flags["transition_flag"]
        self.cruise_airspeed = vehicle_flags["cruise_airspeed"]
        battery_params = ghost_params["battery"]
        self.max_voltage = battery_params["max_voltage"]

        self.mission_end_commands = ghost_params["gcs_commands"]["mission_end_commands"]

        self.id = ghost_params["id"]

        self.threshold = ghost_params["threshold"]

        self.notification = ghost_params["notification"]

        self.available_modes = {
            int(k): v for k, v in ghost_params["available_modes"].items()}

        comms_gcs = ghost_params["comms_gcs"]
        self.command = comms_gcs["command"]
        self.waypoint = comms_gcs["waypoint"]
        self.mission = comms_gcs["mission"]
        self.curr_lat = comms_gcs["curr_lat"]
        self.curr_lon = comms_gcs["curr_lon"]
        self.curr_wp_id = comms_gcs["curr_wp_id"]
        self.st = comms_gcs["st"]
        self.et = comms_gcs["et"]
        self.command_before_pausing = comms_gcs["command_before_pausing"]

        self.is_in_goto = comms_gcs["is_in_goto"]
        self.is_curr_wp_deleted = comms_gcs["is_curr_wp_deleted"]
        self.all_previous_wp_ids = comms_gcs["all_previous_wp_ids"]
        self.gate_opened = comms_gcs["gate_opened"]
        self.previous_waypoint = comms_gcs["previous_waypoint"]
        self.mission_completed = comms_gcs["mission_completed"]
        self.mission_total = comms_gcs["mission_total"]
        self.is_mission_uploaded = comms_gcs["is_mission_uploaded"]
        self.current_waypoint = comms_gcs["current_waypoint"]
        self.current_wp_id = comms_gcs["current_wp_id"]
        self.is_mission_executing = comms_gcs["is_mission_executing"]
        self.is_mission_paused = comms_gcs["is_mission_paused"]
        self.location_set = comms_gcs["location_set"]
        self.isconnected = comms_gcs["isconnected"]
        self.default_onb_params = ghost_params["nav_dive_params"]
        self.routine = None

# ---------------------------------------------------------------------------------------- ros2 calback functions

    def alt_change_cb(self, msg):
        alt_change = json.loads(msg.data)['change_altitude']
        self.change_loiter_altitude(alt=alt_change)

    def vehicle_airspeed_callback(self, vehicle_air):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_airspeed = vehicle_air

    def rtsp_cb(self, msg):
        status = json.loads(msg.data)
        print("rtsp status received : ", status)
        if status["rtsp_status"]:
            self.rtsp["status"] = True

    def dive_from_gcs_cb(self, msg):
        self.dive_gcs = msg
        print("received dive command from gcs : ", self.dive_gcs)
        self.command_publisher.publish(self.dive_gcs)
        self.command = "dive"
        print("sent dive values to diving script")

    def pixels_from_gcs_cb(self, msg):
        self.pixel_gcs = msg
        print("pixels value received from gcs : ", self.pixel_gcs)
        self.pixel_publisher.publish(msg)
        print("sent pixel values to vision script")

    def lock_from_gcs_cb(self, msg):
        self.lock_gcs = msg
        print("Lock value received from gcs : ", self.lock_gcs)
        self.lock_publisher.publish(msg)

    def land_callback(self, data):
        self.vehicle_land = data

    def vehicle_att_callback(self, vehicle_att_sig):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_att = vehicle_att_sig

    def battery_cb(self, battery_info):
        self.battery = battery_info

    def vehicle_odo_callback(self, odometry):
        self.vehicle_odometry = odometry

    def vehicle_global_position_callback(self, msg):
        self.vehicle_global_position = msg
        if self.vehicle_global_position.lat and self.vehicle_global_position.lon and self.home_lat == 0.0:
            self.home_lat = self.vehicle_global_position.lat
            self.home_lon = self.vehicle_global_position.lon
            self.home_alt = self.vehicle_global_position.alt
            self.get_logger().info(f"Saved Global Home Location")

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def from_gcs_cb(self, msg):
        self.command_from_gcs = msg
        command = json.loads(self.command_from_gcs.data)
        self.send_to_autopilot(command)

    def rc_callback(self, rc_data):
        self.rc_data = rc_data
        self.rc_current_switch = self.rc_data.mode_slot
        if not self.rc_switch_init:
            # self.rc_current_switch = self.rc_data.mode_slot
            self.rc_switch_init = True
            self.previous_switch=self.rc_current_switch

        # self.get_logger().info(f"Current RC switch mode: {self.rc_current_switch}")
        if self.rc_current_switch != self.previous_switch:
            self.command = "rc_control"
            self.previous_switch = self.rc_current_switch
            self.is_in_rc_control = 1
            self.get_logger().info(f"Current RC switch mode: {self.rc_current_switch}")

    def tecs_callback(self, tecs_data):
        """Callback function for TecsStatus topic subscriber."""
        self.tecs_data = tecs_data

    def recovery_info_cb(self, msg):
        self.recovery_info = msg
        self.get_logger().info(f"Recovery Info: {self.recovery_info.data}")

    ############## custom functionalities ##############

    def send_to_autopilot(self, data):

        self.is_in_rc_control = 0
        self.get_logger().info(f'Updating Command from :{self.command}')
        if data["command"] == "takeoff":
            self.command = "takeoff"
            self.takeoff_height = float(data["takeoff_alt"])
            print("received a takeoff height of : ",
                  data["takeoff_alt"], " ", self.takeoff_height)
            self.gate_opened = 1

        elif data["command"] == "land":
            self.command = "land"
            if not self.is_in_rc_control:
                self.is_in_rc_control = 1

        elif data["command"] == "rc_control":
            self.command = "rc_control"

        elif data["command"] == "goto":
            if self.offboard_flag:
                self.offboard_flag = 0

            if not self.is_in_goto:
                self.is_in_goto = True

            # resetting the flags (before starting goto)
            self.current_waypoint = 0
            self.current_wp_id = 0
            self.is_mission_executing = True
            self.mission_completed = False
            self.mission_total = 1
            self.previous_waypoint = 0
            self.mission = None
            self.all_previous_wp_ids.clear()

            self.command = "goto"
            waypoints = data["waypoint"][0]
            lat = waypoints["coordinate"][1]
            lon = waypoints["coordinate"][0]
            height = waypoints["altitude"]
            self.goto_alt = height
            self.goto_lat = lat
            self.goto_lon = lon
            print("height received : ", height)
            print("lat received : ", lat)
            print("lon received : ", lon)
            self.waypoint = [lat, lon, height]

        elif data["command"] == "return":
            self.command = "return"

        elif data["command"] == "mode":
            self.command = data["mode"].lower()
            print("Mode change requested to ", self.command)

        elif data["command"] == "mission":
            self.routine = None
            self.is_mission_paused = False
            # print()
            # print(data["waypoint"])
            if self.offboard_flag:
                self.offboard_flag = 0
            if self.command == "goto":
                self.current_waypoint = 0
                self.curr_wp_id = 0
                self.is_in_goto = False
                # self.previous_waypoint = 0
                self.all_previous_wp_ids.clear()
                cmd, status = self.update_command_status()
                for _ in range(5):
                    self.send_data_to_gcs(cmd, status)

            self.command = "mission"
            self.mission = data["waypoint"]
            self.mission_total = len(self.mission)
            self.is_mission_executing = True

            if self.mission_completed or self.is_mission_executing:
                self.mission_completed = False

            self.id = 0
            self.previous_waypoint = 0
            if len(self.all_previous_wp_ids) != 0:
                self.previous_waypoint = self.all_previous_wp_ids[-1]
            self.update_waypoint(self.id, self.mission)

        elif data["command"] == "mission_upload" or data["command"] == "re_execute":

            self.takeoff_height = data["takeoff_alt"]
            self.routine = None
            self.is_mission_uploaded = True
            self.is_mission_paused = False

            if self.offboard_flag:
                self.offboard_flag = 0
            if data["command"] == "re_execute":
                print("is_in_re_ex")
                if self.gate_opened == 0:
                    self.gate_opened = 1
                    self.is_mission_executing = True

                if self.mission_completed or self.is_mission_executing:
                    self.mission_completed = False

            if self.command == "goto":
                self.current_waypoint = 0
                self.curr_wp_id = 0
                self.is_in_goto = False
                self.all_previous_wp_ids.clear()
                cmd, status = self.update_command_status()
                for _ in range(5):
                    self.send_data_to_gcs(cmd, status)

            self.mission = data["waypoint"]
            self.mission_total = len(self.mission)
            self.strike_wp = next(
                (wp for wp in self.mission if wp.get("routine") == "strike"), None)
            if self.strike_wp:
                self.strike_wp_id = self.strike_wp["id"]
                self.routine = self.strike_wp["routine"]
            else:
                self.strike_wp_id = None
                self.routine = None

            self.id = 0
            self.previous_waypoint = 0
            if len(self.all_previous_wp_ids) != 0:
                self.previous_waypoint = self.all_previous_wp_ids[-1]

            if self.routine == "strike":
                print("\n\n routine : ", self.routine)

                strike_index = next((i for i, wp in enumerate(
                    self.mission) if wp.get("id") == self.strike_wp_id), None)

                if strike_index is not None and strike_index > 0:
                    last_but_one_wp = self.mission[strike_index - 1]
                    self.command = "mission"
                else:
                    self.command = "mission"
                    if self.arming_flag:
                        print("taking vehicle's own position as prev wp")
                        ref = {"coordinate": [
                            self.vehicle_global_position.lon, self.vehicle_global_position.lat], "id": 0}
                    else:
                        print("taking home as prev wp")
                        ref = {"coordinate": [
                            self.home_lon, self.home_lat], "id": 0}
                    last_but_one_wp = ref

                dive_wp = self.strike_wp

                last_but_one_global_x, last_but_one_global_y = self.helper.latlon_to_m(
                    last_but_one_wp["coordinate"][1], last_but_one_wp["coordinate"][0])
                dive_wp_global_x, dive_wp_global_y = self.helper.latlon_to_m(
                    dive_wp["coordinate"][1], dive_wp["coordinate"][0])

                vec_lwp_to_dive = np.array(
                    [dive_wp_global_x - last_but_one_global_x, dive_wp_global_y - last_but_one_global_y, 0.0])
                mag_vec_lwp_to_dive = np.linalg.norm(vec_lwp_to_dive)
                # self.get_logger().info(f"magnitude of the vector : {mag_vec_lwp_to_dive}")

                if mag_vec_lwp_to_dive <= 800.0:
                    vec_lwp_to_dive /= mag_vec_lwp_to_dive
                    vec_lwp_to_dive *= 850.0
                    # virtual_global_lat, virtual_global_lon = self.helper.m_to_latlon(
                    #     float(last_but_one_global_x + vec_lwp_to_dive[0]),
                    #     float(last_but_one_global_y + vec_lwp_to_dive[1]))
                    virtual_global_lat, virtual_global_lon = self.helper.m_to_latlon(
                        float(dive_wp_global_x + vec_lwp_to_dive[0]),
                        float(dive_wp_global_y + vec_lwp_to_dive[1]))
                    vec_lwp_to_dive_global = [
                        float(virtual_global_lon), float(virtual_global_lat)]

                    intermediate_virtual_wp = {
                        "altitude": 300.0,
                        "coordinate": vec_lwp_to_dive_global,
                        "id": last_but_one_wp["id"] + 1,
                        "routine": "waypoint",
                        "status": False
                    }

                    self.mission.insert(strike_index, intermediate_virtual_wp)
                    self.mission[strike_index + 1]["id"] += 1
                    print("computed intermediate virtual waypoint : ",
                          self.mission, "\n\n")
                    self.is_pre_mission_updated = True
                    self.new_pre_mission = vec_lwp_to_dive_global
            else:
                self.command = "mission"

            self.update_waypoint(self.id, self.mission)
            self.get_logger().info("Received mission")

        elif data["command"] == "execute_mission" and self.is_mission_uploaded:
            if self.gate_opened == 0:
                self.gate_opened = 1
                self.is_mission_uploaded = False

                self.is_mission_executing = True
                if self.mission_completed or self.is_mission_executing:
                    self.mission_completed = False

        elif data["command"] == "update":
            if self.mission != None:
                updated_wp_ids = [wp['id'] for wp in self.mission]
                self.mission = data["waypoint"]

                if self.is_mission_paused:
                    self.current_waypoint = 0
                    self.current_wp_id = 0
                    cmd, status = self.update_command_status()
                    for _ in range(5):
                        self.send_data_to_gcs(cmd, status)

                    if len(self.mission) > 0:
                        if self.current_wp_id not in updated_wp_ids:
                            self.is_curr_wp_deleted = True
                        else:
                            self.id = 0
                            self.previous_waypoint = 0
                            if len(self.all_previous_wp_ids) != 0:
                                self.previous_waypoint = self.all_previous_wp_ids[-1]
                            self.mission_total = len(self.mission)
                            self.update_waypoint(self.id, self.mission)
                    else:
                        self.mission = None
                        self.change_mode(4.0, 3.0, "hold")
                else:
                    self.id = 0
                    self.previous_waypoint = 0
                    if len(self.all_previous_wp_ids) != 0:
                        self.previous_waypoint = self.all_previous_wp_ids[-1]
                    self.mission_total = len(self.mission)
                    self.update_waypoint(self.id, self.mission)

        elif data["command"] == "skip":
            skip_wp_id = data["goto_wp_id"]
            if self.current_wp_id != 0:
                for idx, wp in enumerate(self.mission):
                    if wp["id"] == skip_wp_id:
                        prev_id = self.mission[idx-1]["id"]
                        self.mission = self.mission[idx:]
                        self.id = 0
                        self.previous_waypoint = prev_id
                        self.mission_total = len(self.mission)
                        self.update_waypoint(self.id, self.mission)
                        break

            elif self.current_wp_id == None:
                print(self.current_waypoint)

        elif data["command"] == "pause":
            self.command_before_pausing = self.command
            self.is_mission_paused = True
            self.command = "pause"

        elif data["command"] == "resume":
            self.is_mission_paused = False
            if self.command_before_pausing != None:
                self.command = self.command_before_pausing
            self.offboard_flag = 0

            if self.is_curr_wp_deleted:
                self.id = 0
                self.previous_waypoint = 0
                self.mission_total = len(self.mission)
                self.update_waypoint(self.id, self.mission)
                self.is_curr_wp_deleted = False

        elif data["command"] == "orbit":
            self.command = "orbit"
            if not self.gate_opened:
                self.gate_opened = 1
            if data["radius"] > 0.0:
                self.orbit_radius = data["radius"]
                self.orbit_waypoint = [data["lat"],
                                       data["lon"], self.orbit_radius]
                self.get_logger().info(f"orbit lat : {data['lat']}")
                self.get_logger().info(f"orbit lon : {data['lon']}")
                self.get_logger().info(f"orbit radius : {self.orbit_radius}")

        elif data["command"] == "cancel":
            self.reset_mission_params(data["command"])

        elif data["command"] == "strike_lat_long":
            self.command = data["command"]
            vehicle_global_x, vehicle_global_y = self.helper.latlon_to_m(
                self.vehicle_global_position.lat, self.vehicle_global_position.lon)
            dive_lat_x, dive_lon_y = self.helper.latlon_to_m(
                data["coordinate"][1], data["coordinate"][0])

            vec_vehicle_to_dive = np.array(
                [dive_lat_x - vehicle_global_x, dive_lon_y - vehicle_global_y, 0.0])
            mag_vec_vehicle_to_dive = np.linalg.norm(vec_vehicle_to_dive)

            if mag_vec_vehicle_to_dive >= 800.0:
                self.command_publisher.publish(self.command_from_gcs)
                print("published coordinated strike command immediately....magnitude : ",
                      mag_vec_vehicle_to_dive)
            else:
                print("will have to compute a custom pre-mission....magnitude : ",
                      mag_vec_vehicle_to_dive)
                vec_vehicle_to_dive /= mag_vec_vehicle_to_dive
                vec_vehicle_to_dive *= 850.0
                virtual_global_lat, virtual_global_lon = self.helper.m_to_latlon(float(
                    vehicle_global_x + vec_vehicle_to_dive[0]), float(vehicle_global_y + vec_vehicle_to_dive[1]))
                vec_vehicle_to_dive_global = [
                    float(virtual_global_lon), float(virtual_global_lat)]

                self.routine = None

                if self.offboard_flag:
                    self.offboard_flag = 0

                # change gate value
                if self.gate_opened == 0:
                    self.gate_opened = 1
                    print("opening gate for custom premission (it's redundant) ")

                if self.command == "goto":
                    self.current_waypoint = 0
                    self.curr_wp_id = 0
                    self.is_in_goto = False
                    # self.previous_waypoint = 0
                    self.all_previous_wp_ids.clear()
                    cmd, status = self.update_command_status()
                    for _ in range(5):
                        self.send_data_to_gcs(cmd, status)

                self.mission = [{"altitude": 300.0, "coordinate": vec_vehicle_to_dive_global, "id": 1, "routine": "waypoint", "status": False},
                                {"altitude": 0.0, "coordinate": data["coordinate"], "id": 2, "routine": "strike", "status": False}]
                print("computed pre-mission : ", self.mission)

                self.mission_total = len(self.mission)
                self.is_mission_executing = True

                if self.mission_completed or self.is_mission_executing:
                    self.mission_completed = False

                last_waypoint = self.mission[self.mission_total - 1]
                self.routine = last_waypoint["routine"]

                # send the custom computed pre-mission to genga
                self.is_pre_mission_updated = True
                self.new_pre_mission = vec_vehicle_to_dive_global

                self.id = 0
                self.previous_waypoint = 0
                if len(self.all_previous_wp_ids) != 0:
                    self.previous_waypoint = self.all_previous_wp_ids[-1]

                self.command = "mission"
                self.update_waypoint(self.id, self.mission)

        else:
            pass

    def update_waypoint(self, id, mission):
        wp_id = mission[id]["id"]
        self.current_waypoint = wp_id
        self.current_wp_id = wp_id
        self.curr_lat = mission[id]["coordinate"][1]
        self.curr_lon = mission[id]["coordinate"][0]
        self.curr_alt = mission[id]["altitude"]
        self.routine = mission[id].get("routine", "waypoint")

    def reset_mission_params(self, cmd):
        self.command = cmd
        self.id = 0
        self.mission = None
        self.previous_waypoint = 0
        self.mission_completed = False
        self.mission_total = 0
        self.current_waypoint = 0
        self.current_wp_id = 0
        self.is_mission_executing = False
        self.is_mission_paused = False
        self.all_previous_wp_ids.clear()

    def get_attitude(self):
        r, p, y = self.helper.euler_from_quaternion(
            self.vehicle_att.q[1], self.vehicle_att.q[2], self.vehicle_att.q[3], self.vehicle_att.q[0])

        return str(np.rad2deg(r)), str(np.rad2deg(p)), str(np.rad2deg(y))

    def get_current_heading(self):
        r, p, y = self.helper.euler_from_quaternion(
            self.vehicle_att.q[1], self.vehicle_att.q[2], self.vehicle_att.q[3], self.vehicle_att.q[0])
        return y

    def waypoints(self, curr_lat, curr_lon, curr_alt):

        target_lat = curr_lat
        target_lon = curr_lon
        target_lon_m, target_lat_m, _, _ = utm.from_latlon(
            target_lat, target_lon)
        home_lon_m, home_lat_m, _, _ = utm.from_latlon(
            self.home_lat, self.home_lon)
        waypoint = np.array([target_lat_m, target_lon_m, -curr_alt]
                            ) - np.array([home_lat_m, home_lon_m, 0.0])
        return float(waypoint[0]), float(waypoint[1]), float(curr_alt)

    def get_flight_status(self):
        flying = False
        takeoff = False
        landing = False
        landed = False

        if self.vehicle_status.takeoff_time > 0 and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER or self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD or self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_RTL and (-self.vehicle_odometry.position[2] > self.takeoff_height - 2):
            flying = True
        elif self.vehicle_status.takeoff_time > 0 and self.vehicle_global_position.alt < ((self.home_alt + self.takeoff_height) - 2) and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
            takeoff = True
        elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND or self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_RTL:
            if (self.vehicle_global_position.alt - self.home_alt > 1.0):
                landing = True
            else:
                landed = True

        flight_status = {"flying": flying,
                         "takeoff": takeoff,
                         "landing": landing,
                         "landed": landed}

        return flight_status

    def send_data_to_gcs(self, cmd, status):

        # armable status
        armable = None
        if self.vehicle_status.pre_flight_checks_pass:
            armable = True
        else:
            armable = False

        # armed status
        armed = None
        if self.vehicle_status.arming_state == 1:
            armed = False
        else:
            armed = True

        # in air status
        self.in_air_status = None
        if self.vehicle_status.nav_state == 14:
            self.in_air_status = True
        else:
            self.in_air_status = False

        # flight status
        flight_status = self.get_flight_status()
        flight_status["armable"] = armable
        flight_status["armed"] = armed
        flight_status["return_stat"] = False
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_RTL:
            flight_status["return_stat"] = True
        # print(flight_status)

        # attitude
        r, p, y = self.get_attitude()

        # flight time
        flight_time = 0.0
        if self.st != None:
            self.et = time.time()
            flight_time = self.et - self.st
            flight_time = str(time.strftime(
                "%H:%M:%S", time.gmtime(flight_time)))
        else:
            flight_time = "00:00:00"

        data = {
            "vehicle_telem_data": {
                "local_position": {
                    "latitude": str(self.vehicle_global_position.lat),
                    "longitude": str(self.vehicle_global_position.lon),
                    "altitude": str(-self.vehicle_odometry.position[2])
                },
                "home_position": {
                    "latitude": str(self.home_lat),
                    "longitude": str(self.home_lon),
                    "altitude": str(self.home_alt)
                },
                "speed": {
                    "airspeed": str(self.helper.get_resultant(self.vehicle_odometry.velocity[0], self.vehicle_odometry.velocity[1])),
                    "ground_speed": str(self.helper.get_resultant(self.vehicle_odometry.velocity[0], self.vehicle_odometry.velocity[1]))
                },
                "euler_angles": {
                    "pitch": r,
                    "roll": p,
                    "yaw": y
                },
                "velocity_ned": {
                    "velocity_n": str(self.vehicle_odometry.velocity[0]),
                    "velocity_e": str(self.vehicle_odometry.velocity[1]),
                    "velocity_d": str(-self.vehicle_odometry.velocity[2])
                },
                "gps_info": {
                    "no_of_sat": str(0),
                    "fix_type": str(0)
                },
                "battery": {
                    "voltage": str(self.battery.voltage_v),
                    "temperature": str(0.0),
                    "current": str(self.battery.current_a),
                    "capacity": str(self.battery.capacity),
                    "batt_percent": str(self.helper.get_battery_percentage(self.battery.voltage_v, self.max_voltage))
                },
                "rc_status": str(self.radio_status.remote_rssi),
                "all_altitude": {
                    "altitude_a": str(-self.vehicle_odometry.position[2]),
                    "altitude_relative_asml_m": str(self.vehicle_global_position.alt)
                }
            },


            "vehicle_status": {
                "armable": armable,
                "armed": armed,
                "connection_status": True,
                "in_air_status": self.in_air_status,
                "id": str(self.vehicle_status.system_id),
                "boot_flight_time": str(flight_time),
                "flight_time": str(flight_time)
            },

            "flight_status": flight_status,

            "rtsp": self.rtsp,

            "flight_modes": {
                "available_mode": ["position", "loiter"],
                "current_mode": self.available_modes[self.vehicle_status.nav_state],
            },

            "mode_flag": self.is_in_rc_control,
            "user_command": {
                "command": cmd,
                "status": status
            },
            "name": "Onboard",

            "notification": None,

            "mission_data": {
                "mission_completed": self.mission_completed,
                "mission_total": self.mission_total,
                "is_mission_completed": self.mission_completed,
                "current_waypoint": self.current_waypoint,
                "current_wp_id": self.current_wp_id,
                "is_mission_executing": self.is_mission_executing,
                "is_mission_paused": self.is_mission_paused,
                "previous_wp_id": self.previous_waypoint,
                "is_mission_uploaded": self.is_mission_uploaded,
                "is_in_goto": self.is_in_goto
            },

            "isconnected": self.isconnected,

            "location_set": self.location_set,

            "updated_pre_mission": self.new_pre_mission,

            "onboard_params": self.default_onb_params
        }

        if self.new_pre_mission != None and self.is_pre_mission_updated:
            self.new_pre_mission_ctr += 1
            if self.new_pre_mission_ctr >= 5:
                self.new_pre_mission = None
                self.is_pre_mission_updated = False
                self.new_pre_mission_ctr = 0
                print("send the new premission....now resetting the flags")
        gcs_data = json.dumps(data)
        pub_data = String()
        pub_data.data = gcs_data
        self.gcs_publisher.publish(pub_data)

    def arm(self, val):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=val)
        self.get_logger().info('Arm command sent')

    def takeoff_mc(self):
        self.get_logger().info('MC_takeoff')
        self.get_logger().info(f'Takeoff height: {self.takeoff_height}')
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param1=0.0,
            param4=nan,
            param5=self.vehicle_global_position.lat,
            param6=self.vehicle_global_position.lon,
            param7=self.home_alt + self.takeoff_height)
        self.get_logger().info('MC_takeoff command sent')

    def vtol_transition(self, mode):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION,
            param1=mode
        )
        print(f'transition command sent')

    def land(self,):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND
        )
        print("Land command sent")

    def publish_trajectory_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan')
        msg.yawspeed = float('nan')
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def publish_offboard_control_heartbeat_signal(
        self,
        position=False,
        attitude=False,
        velocity=False,
        acceleration=False,
        body_rate=False,
    ):
        # """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = position
        msg.velocity = velocity
        msg.acceleration = acceleration
        msg.attitude = attitude
        msg.body_rate = body_rate
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def change_mode(self, mode, submode, name):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=mode, param3=submode)
        self.get_logger().info(f"Switching to {name} mode")

    def change_loiter_altitude(self, alt):
        # self.home_alt+float(abs(alt))
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_CHANGE_ALTITUDE, param1=self.home_alt+float(abs(alt)), param2=float(0)
        )
        self.get_logger().info(
            f"increasing altitude with a give value of : {alt}")

    def update_command_status(self):
        cmd = self.command
        status = 0.0
        if self.command == None:
            cmd = 'none'
            status = 'none'
        elif self.command == "hold":
            cmd = "takeoff"
            if self.vehicle_global_position.alt < ((self.home_alt + self.takeoff_height) - 1):
                status = "Taking Off"
            elif self.vehicle_global_position.alt > ((self.home_alt + self.takeoff_height) - 1):
                status = "Takeoff Completed"
        elif self.command == "goto":
            if self.vehicle_status.nav_state == 14:
                status = "Navigating to the given Lat and Lon"
            elif self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                status = "Changing to Offboard flight mode"
            # print(status)
        elif self.command == "return":
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_RTL:
                status = "Vehicle returning to home"
            elif self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_RTL:
                status = "Changing to Return flight mode"
        elif self.command == "land":
            if self.vehicle_status == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                status = "Landing the Vehicle at the current position"
            elif self.vehicle_status != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                status = "Changing to Land flight mode"
        elif self.command == "Loiter":
            if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                status = "Changing to Hold flight mode"
            elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                status = "Vehicle is in Hold flight mode"
        elif self.command == "position":
            if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_POSCTL:
                status = "Changing to Position flight mode"
            elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL:
                status = "Vehicle is in Position flight mode"
        elif self.command == "rc_control":
            status = "Vehicle is in RC Input mode"

        return cmd, status

    ############## timer callback ##############

    def timer_cb(self):
        
        if self.command != "strike_lat_long" and self.command != "dive":
            self.publish_offboard_control_heartbeat_signal(position=True)

        if not self.home_flag and self.vehicle_global_position.lat > 0.0:
            self.home_alt = self.vehicle_global_position.alt
            self.home_flag = 1
            self.location_set = True
            self.get_logger().info(f"\nSaved home alt : {self.home_alt}\n"
                                   f"Saved home lat : {self.home_lat}\n"
                                   f"Saved home lon : {self.home_lon}")

        cmd, status = self.update_command_status()
        #self.send_data_to_gcs(cmd, status)
        if self.home_flag:
            self.send_data_to_gcs(cmd, status)
            # self.gate_opened = True #Just for checking
            
        if self.command == "rc_control":
            self.get_logger().info("In RC Control")
            return

        if self.gate_opened:

            if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED and not self.arming_flag and self.home_flag:

                self.arm(1.0)
                self.arming_flag = 1
                if self.st == None:
                    self.st = time.time()
                self.get_logger().info('arm_check')

            if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED and self.takeoff_flag == 0 and self.arming_flag:
                self.takeoff_mc()
                # self.takeoff_vtol()
                self.get_logger().info('Takeoff_check')
                self.takeoff_flag = 1
                self.command = "hold"

            # HOLD (after takeoff to initiate transition) vtol_change
            # if self.vehicle_global_position.alt > (self.home_alt + self.takeoff_height) - 1 and self.vehicle_status.is_vtol and not self.transition_flag and self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
            #     if self.command == "hold":
            #         self.change_mode(4.0, 3.0, "hold")
            if self.vehicle_global_position.alt > (self.home_alt + self.takeoff_height) - 1 and self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                if self.command == "hold":
                    self.change_mode(4.0, 3.0, "hold")

            # TRANSITION vtol_change
            # if self.vehicle_global_position.alt > (self.home_alt + self.takeoff_height) - 1 and self.vehicle_status.is_vtol and not self.transition_flag and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
            #     self.vtol_transition(4.0)
            #     time.sleep(0.5)
            #     if self.vehicle_status.in_transition_to_fw:
            #         self.transition_flag = 1
            #     if self.routine != None:
            #         self.command = "mission"
            if self.vehicle_global_position.alt > (self.home_alt + self.takeoff_height) - 1  and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                # self.vtol_transition(4.0)
                # time.sleep(0.5)
                # if self.vehicle_status.in_transition_to_fw:
                #     self.transition_flag = 1
                if self.routine != None:
                    self.command = "mission"

            if self.command == "mission" and self.mission != None and not self.is_mission_paused and self.vehicle_status.vehicle_type == VehicleStatus.VEHICLE_TYPE_FIXED_WING:
                if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD and not self.offboard_flag and not self.mission_completed:
                    self.change_mode(6.0, 0.0, "offboard")
                    self.get_logger().info("changing mode to offboard")
                    self.get_logger().info(f"current command : {self.command}")
                elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    if not self.offboard_flag:
                        self.offboard_flag = 1

                    # logic for routine
                    if self.routine != None:
                        if self.routine == "strike":
                            self.command = "strike_lat_long"
                            strike_wp_lon_lat = self.mission[self.id]["coordinate"]
                            dive_message = json.dumps({"command": self.command,
                                                       "coordinate": strike_wp_lon_lat})
                            self.strike_routine.data = dive_message
                            self.command_publisher.publish(self.strike_routine)

                            self.get_logger().info("published strike routine to dive script")
                        # return routine
                        if self.routine == "returnWaypoint":
                            if self.helper.calculate_reached_threshold(self.vehicle_global_position.lat,
                                                                       self.vehicle_global_position.lon,
                                                                       self.curr_lat,
                                                                       self.curr_lon,
                                                                       self.cruise_airspeed):
                                self.command = "return"
                                self.get_logger().info("applying return routine")

                    # waypoint update
                    if self.helper.calculate_reached_threshold(self.vehicle_global_position.lat,
                                                               self.vehicle_global_position.lon,
                                                               self.curr_lat,
                                                               self.curr_lon,
                                                               self.cruise_airspeed):

                        self.previous_waypoint = self.current_wp_id
                        self.all_previous_wp_ids.append(self.previous_waypoint)  # new change
                        self.id += 1
                        if self.id <= len(self.mission) - 1:
                            self.get_logger().info("Updating Waypoint")
                            self.update_waypoint(self.id, self.mission)
                        else:
                            self.get_logger().info("all wps reached")
                            self.id = 0
                            self.mission_completed = True
                            self.is_mission_executing = False
                            self.is_mission_uploaded = False
                            self.current_waypoint = 0
                            self.current_wp_id = 0
                            if self.routine == "waypoint" or self.routine == None:
                                self.change_mode(4.0, 3.0, "hold")
                                self.command = "hold"

                    xw, yw, zw = self.waypoints(self.curr_lat,
                                                self.curr_lon,
                                                self.curr_alt)
                    self.publish_trajectory_setpoint(xw, yw, -zw)
                    
                    # params = {
                    #         "xw": xw,
                    #         "yw": yw
                    # }
                    # self.helper.log_params_to_csv(params, filename="com_test_data.csv")

            if self.command == "goto" and self.waypoint != None and not self.is_mission_paused and self.vehicle_status.vehicle_type == VehicleStatus.VEHICLE_TYPE_FIXED_WING:
                if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD and not self.offboard_flag and not self.mission_completed:
                    self.change_mode(6.0, 0.0, "offboard")
                    self.get_logger().info("changing mode to offboard")
                elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    if not self.offboard_flag:
                        self.offboard_flag = 1
                    if self.helper.calculate_reached_threshold(self.vehicle_global_position.lat,
                                                               self.vehicle_global_position.lon,
                                                               self.waypoint[0],
                                                               self.waypoint[1],
                                                               self.cruise_airspeed):
                        self.get_logger().info("goto wps reached....switching to hold mode")
                        self.change_mode(4.0, 3.0, "hold")
                        self.command = "hold"
                        # resetting the flags (after reaching goto)
                        self.current_waypoint = 0
                        self.current_wp_id = 0
                        self.is_mission_executing = False
                        self.mission_completed = True
                        self.mission_total = 0
                        self.previous_waypoint = 0
                        self.is_in_goto = False

                    xw, yw, zw = self.waypoints(self.goto_lat,
                                                self.goto_lon,
                                                self.goto_alt)
                    # self.get_logger().info('Navigating to GO-TO setpoint')
                    self.publish_trajectory_setpoint(xw, yw, -zw)

            if self.command == "land" and self.takeoff_flag and self.vehicle_status.nav_state != 18:
                self.land()
                if self.vehicle_status.nav_state == 18:
                    self.command = None

            elif self.command == "return" and self.takeoff_flag and self.vehicle_status.nav_state != 5:
                self.change_mode(4.0, 5.0, 'Return')
                self.reset_mission_params(self.command)
                if self.vehicle_status.nav_state == 5:
                    self.command = None

            elif self.command == "loiter" or self.command == "position":
                if self.command == "loiter" and self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                    self.change_mode(4.0, 3.0, 'hold')
                if self.command == "position" and self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_POSCTL:
                    self.change_mode(3.0, 0.0, 'position')
                if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER or self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL:
                    self.command = None

            elif self.command == "pause" and self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                self.change_mode(4.0, 3.0, "hold")

                if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                    self.command = None

            elif self.command == "cancel" and self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                self.change_mode(4.0, 3.0, "hold")
                self.mission = None
                print("cleared all mission wps")
                


def main(args=None) -> None:
    print('Starting gcs comms node...')
    rclpy.init(args=args)
    ghost_comms = Comms()
    rclpy.spin(ghost_comms)
    ghost_comms.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)

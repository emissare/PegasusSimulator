"""
| File: px4_backend.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| Description: File that implements the Mavlink Backend for communication/control with/of the vehicle simulation
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
"""

__all__ = ["PX4Backend", "PX4BackendConfig"]

import carb
import time
import numpy as np
from pymavlink import mavutil

from typing import Optional
from pegasus.simulator.logic.vehicle_state import VehicleState
from pegasus.simulator.logic.sensors.sensor_models import (
    GPSState,
    IMUState,
    BarometerState,
    MagnetometerState,
    SimulationState,
)

# Removed backend abstraction - no longer needed
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.px4_launch_tool import PX4LaunchTool


class SensorSource:
    """The binary codes to signal which simulated data is being sent through mavlink

    Atribute:
        | ACCEL (int): mavlink binary code for the accelerometer (0b0000000000111 = 7)
        | GYRO (int): mavlink binary code for the gyroscope (0b0000000111000 = 56)
        | MAG (int): mavlink binary code for the magnetometer (0b0000111000000=448)
        | BARO (int): mavlink binary code for the barometer (0b1101000000000=6656)
        | DIFF_PRESS (int): mavlink binary code for the pressure sensor (0b0010000000000=1024)
    """

    ACCEL: int = 7
    GYRO: int = 56
    MAG: int = 448
    BARO: int = 6656
    DIFF_PRESS: int = 1024


class SensorMsg:
    """
    An auxiliary data class where we store all the sensor data that is going to be sent through mavlink.
    Uses Pydantic BaseModels for type safety and clear documentation of units and coordinate systems.
    """

    def __init__(self):
        # Sensor states using BaseModels
        self.imu_state: Optional[IMUState] = None
        self.gps_state: Optional[GPSState] = None
        self.barometer_state: Optional[BarometerState] = None
        self.magnetometer_state: Optional[MagnetometerState] = None

        # Simulation groundtruth state
        self.simulation_state: Optional[SimulationState] = None

        # Flags indicating new data is available
        self.new_imu_data: bool = False
        self.new_gps_data: bool = False
        self.new_bar_data: bool = False
        self.new_mag_data: bool = False
        self.new_sim_state: bool = False

        # Special flags for IMU
        self.received_first_imu: bool = False

        # Airspeed data (not yet in a model)
        self.new_press_data: bool = False
        self.diff_pressure: float = 0.0


class ThrusterControl:
    """
    An auxiliary data class that saves the thrusters command data received via mavlink and
    scales them into individual angular velocities expressed in rad/s to apply to each rotor
    """

    def __init__(
        self,
        num_rotors: int = 4,
        input_offset=[0, 0, 0, 0],
        input_scaling=[0, 0, 0, 0],
        zero_position_armed=[100, 100, 100, 100],
    ):
        """Initialize the ThrusterControl object

        Args:
            num_rotors (int): The number of rotors that the actual system has 4.
            input_offset (list): A list with the offsets to apply to the rotor values received via mavlink. Defaults to [0, 0, 0, 0].
            input_scaling (list): A list with the scaling to apply to the rotor values received via mavlink. Defaults to [0, 0, 0, 0].
            zero_position_armed (list): Another list of offsets to apply to the rotor values received via mavlink. Defaults to [100, 100, 100, 100].
        """

        self.num_rotors: int = num_rotors

        # Values to scale and offset the rotor control inputs received from PX4
        assert len(input_offset) == self.num_rotors
        self.input_offset = input_offset

        assert len(input_scaling) == self.num_rotors
        self.input_scaling = input_scaling

        assert len(zero_position_armed) == self.num_rotors
        self.zero_position_armed = zero_position_armed

        # The actual speed references to apply to the vehicle rotor joints
        self._input_reference = [0.0 for i in range(self.num_rotors)]

    @property
    def input_reference(self):
        """A list of floats with the angular velocities in rad/s

        Returns:
            list: A list of floats with the angular velocities to apply to each rotor, expressed in rad/s
        """
        return self._input_reference

    def update_input_reference(self, controls):
        """Takes a list with the thrust controls received via mavlink and scales them in order to generated
        the equivalent angular velocities in rad/s

        Args:
            controls (list): A list of ints with thrust controls received via mavlink
        """

        # Check if the number of controls received is correct
        if len(controls) < self.num_rotors:
            carb.log_warn("Did not receive enough inputs for all the rotors")
            return

        # Update the desired reference for every rotor (and saturate according to the min and max values)
        for i in range(self.num_rotors):

            # Compute the actual velocity reference to apply to each rotor
            self._input_reference[i] = (
                controls[i] + self.input_offset[i]
            ) * self.input_scaling[i] + self.zero_position_armed[i]

    def zero_input_reference(self):
        """
        When this method is called, the input_reference is updated such that every rotor is stopped
        """
        self._input_reference = [0.0 for i in range(self.num_rotors)]


class PX4BackendConfig:
    """
    An auxiliary data class used to store all the configurations for the mavlink communications.
    """

    def __init__(self, config={}):
        """
        Initialize the PX4MavlinkBackendConfig class

        Args:
            config (dict): A Dictionary that contains all the parameters for configuring the Mavlink interface - it can be empty or only have some of the parameters used by this backend.

        Examples:
            The dictionary default parameters are

            >>> {"vehicle_id": 0,
            >>>  "connection_type": "tcpin",
            >>>  "connection_ip": "localhost",
            >>>  "connection_baseport": 4560,
            >>>  "px4_autolaunch": True,
            >>>  "px4_dir": "PegasusInterface().px4_path",
            >>>  "px4_vehicle_model": "gazebo-classic_iris",
            >>>  "enable_lockstep": True,
            >>>  "num_rotors": 4,
            >>>  "input_offset": [0.0, 0.0, 0.0, 0.0],
            >>>  "input_scaling": [1000.0, 1000.0, 1000.0, 1000.0],
            >>>  "zero_position_armed": [100.0, 100.0, 100.0, 100.0],
            >>>  "update_rate": 250.0
            >>> }
        """

        # Configurations for the mavlink communication protocol (note: the vehicle id is sumed to the connection_baseport)
        self.config = config

        self.vehicle_id = self.config.get("vehicle_id", 0)
        self.connection_type = self.config.get("connection_type", "tcpin")
        self.connection_ip = self.config.get("connection_ip", "localhost")
        self.connection_baseport = self.config.get("connection_baseport", 4560)

        # Configure whether to launch px4 in the background automatically or not for every vehicle launched
        self.px4_autolaunch: bool = self.config.get("px4_autolaunch", True)
        self.px4_dir: str = self.config.get("px4_dir", PegasusInterface().px4_path)
        self.px4_vehicle_model: str = self.config.get(
            "px4_vehicle_model", "gazebo-classic_iris"
        )

        # Configurations to interpret the rotors control messages coming from mavlink
        self.enable_lockstep: bool = self.config.get("enable_lockstep", True)
        self.num_rotors: int = self.config.get("num_rotors", 4)
        self.input_offset = self.config.get("input_offset", [0.0, 0.0, 0.0, 0.0])
        self.input_scaling = self.config.get(
            "input_scaling", [1000.0, 1000.0, 1000.0, 1000.0]
        )
        self.zero_position_armed = self.config.get(
            "zero_position_armed", [100.0, 100.0, 100.0, 100.0]
        )

        # The update rate at which we will be sending data to mavlink (TODO - remove this from here in the future
        # and infer directly from the function calls)
        self.update_rate: float = self.config.get("update_rate", 250.0)  # [Hz]


class PX4Backend:
    """The Mavlink Backend used to receive the vehicle's state and sensor data in order to send to PX4 through mavlink. It also
    receives via mavlink the thruster commands to apply to each vehicle rotor.
    """

    def __init__(self, config: PX4BackendConfig = PX4BackendConfig()):
        """Initialize the PX4Backend

        Args:
            config (PX4BackendConfig): The configuration class for the PX4Backend. Defaults to PX4BackendConfig().
        """

        # Store configuration and initialize vehicle reference
        self._vehicle = None
        self.config: PX4BackendConfig = config

        # Setup the desired mavlink connection port
        # The connection will only be created once the simulation starts
        self._vehicle_id = self.config.vehicle_id
        self._connection = None
        self._connection_port = (
            self.config.connection_type
            + ":"
            + self.config.connection_ip
            + ":"
            + str(self.config.connection_baseport + self.config.vehicle_id)
        )

        # Check if we need to autolaunch px4 in the background or not
        self.px4_autolaunch: bool = self.config.px4_autolaunch
        self.px4_vehicle_model: str = (
            self.config.px4_vehicle_model
        )  # only needed if px4_autolaunch == True
        self.px4_tool: PX4LaunchTool = None
        self.px4_dir: str = self.config.px4_dir

        # Set the update rate used for sending the messages (TODO - remove this hardcoded value from here)
        self._update_rate: float = self.config.update_rate
        self._time_step: float = 1.0 / self._update_rate  # s

        self._is_running: bool = False

        # Vehicle Sensor data to send through mavlink
        self._sensor_data: SensorMsg = SensorMsg()

        # Vehicle Rotor data received from mavlink
        self._rotor_data: ThrusterControl = ThrusterControl(
            self.config.num_rotors,
            self.config.input_offset,
            self.config.input_scaling,
            self.config.zero_position_armed,
        )

        # Vehicle actuator control data
        self._num_inputs: int = self.config.num_rotors
        self._input_reference: np.ndarray = np.zeros((self._num_inputs,))
        self._armed: bool = False

        self._input_offset: np.ndarray = np.zeros((self._num_inputs,))
        self._input_scaling: np.ndarray = np.zeros((self._num_inputs,))

        # Select whether lockstep is enabled
        self._enable_lockstep: bool = self.config.enable_lockstep

        # Auxiliar variables to handle the lockstep between receiving sensor data and actuator control
        self._received_first_actuator: bool = False

        self._received_actuator: bool = False

        # Auxiliar variables to check if we have already received an hearbeat from the software in the loop simulation
        self._received_first_hearbeat: bool = False

        self._last_heartbeat_sent_time = 0

        # Auxiliar variables for setting the u_time when sending sensor data to px4
        self._current_utime: int = 0

    @property
    def vehicle(self):
        """A reference to the vehicle associated with this backend.

        Returns:
            Vehicle: A reference to the vehicle associated with this backend.
        """
        return self._vehicle

    def initialize(self, vehicle):
        """A method that can be invoked when the simulation is starting to give access to the control backend
        to the entire vehicle object.

        Args:
            vehicle (Vehicle): A reference to the vehicle that this sensor is associated with
        """
        self._vehicle = vehicle

    def update_sensor(self, sensor_type: str, data):
        """Method that is used as callback for the vehicle for every iteration that a sensor produces new data.
        Only the IMU, GPS, Barometer and  Magnetometer sensor data are stored to be sent through mavlink. Every other
        sensor data that gets passed to this function is discarded.

        Args:
            sensor_type (str): A name that describes the type of sensor
            data (dict): A dictionary that contains the data produced by the sensor
        """

        if sensor_type == "IMU":
            self.update_imu_data(data)
        elif sensor_type == "GPS":
            self.update_gps_data(data)
        elif sensor_type == "Barometer":
            self.update_bar_data(data)
        elif sensor_type == "Magnetometer":
            self.update_mag_data(data)
        # If the data received is not from one of the above sensors, then this backend does
        # not support that sensor and it will just ignore it
        else:
            pass

    def update_imu_data(self, data: IMUState):
        """Gets called by the 'update_sensor' method to update the current IMU data

        Args:
            data (IMUState): The data produced by an IMU sensor
        """
        self._sensor_data.imu_state = data
        self._sensor_data.new_imu_data = True
        self._sensor_data.received_first_imu = True

    def update_gps_data(self, data: GPSState):
        """Gets called by the 'update_sensor' method to update the current GPS data

        Args:
            data (GPSState): The data produced by a GPS sensor
        """
        self._sensor_data.gps_state = data
        self._sensor_data.new_gps_data = True

    def update_bar_data(self, data: BarometerState):
        """Gets called by the 'update_sensor' method to update the current barometer data

        Args:
            data (BarometerState): The data produced by a barometer sensor
        """
        self._sensor_data.barometer_state = data
        self._sensor_data.new_bar_data = True

    def update_mag_data(self, data: MagnetometerState):
        """Gets called by the 'update_sensor' method to update the current magnetometer data

        Args:
            data (MagnetometerState): The data produced by a magnetometer sensor
        """
        self._sensor_data.magnetometer_state = data
        self._sensor_data.new_mag_data = True

    def update_state(self, state: VehicleState):
        """Method that is used as callback and gets called at every physics step with the current state of the vehicle.
        This state is then stored in order to be sent as groundtruth via mavlink

        Args:
            state (VehicleState): The current state of the vehicle.
        """

        # Get the quaternion in the convention [x, y, z, w]
        attitude_quat = state.attitude_frd_ned_quat

        # Convert to mavlink format [qw, qx, qy, qz]
        attitude_wxyz = [
            attitude_quat[3],  # qw
            attitude_quat[0],  # qx
            attitude_quat[1],  # qy
            attitude_quat[2],  # qz
        ]

        angular_vel_frd = state.angular_velocity_frd_rps
        acceleration_ned = state.acceleration_ned_mpss
        velocity_ned = state.velocity_ned_mps
        body_velocity_frd = state.body_velocity_frd_mps

        # Convert NED position to geodetic coordinates
        import numpy as np

        # For now, use simple approximation if GPS hasn't been initialized
        if self._sensor_data.gps_state:
            latitude_deg = self._sensor_data.gps_state.latitude_groundtruth_deg
            longitude_deg = self._sensor_data.gps_state.longitude_groundtruth_deg
            altitude_msl_m = self._sensor_data.gps_state.altitude_groundtruth_msl_m
        else:
            # Default origin coordinates
            latitude_deg = 0.0
            longitude_deg = 0.0
            altitude_msl_m = 488.0

        # Create SimulationState object
        self._sensor_data.simulation_state = SimulationState(
            attitude_quat_wxyz_frd_ned=attitude_wxyz,
            angular_velocity_frd_body_rps=angular_vel_frd.tolist(),
            acceleration_ned_mpss=acceleration_ned.tolist(),
            velocity_ned_mps=velocity_ned.tolist(),
            latitude_deg=latitude_deg,
            longitude_deg=longitude_deg,
            altitude_msl_m=altitude_msl_m,
            indicated_airspeed_mps=body_velocity_frd[0],  # Assumed aligned with body X
            true_airspeed_mps=np.linalg.norm(velocity_ned),  # TODO: add wind
        )

        self._sensor_data.new_sim_state = True

    def input_reference(self):
        """Method that when implemented, should return a list of desired angular velocities to apply to the vehicle rotors"""
        return self._rotor_data.input_reference

    def __del__(self):
        """Gets called when the PX4MavlinkBackend object gets destroyed. When this happens, we make sure
        to close any mavlink connection open for this vehicle.
        """

        # When this object gets destroyed, close the mavlink connection to free the communication port
        try:
            self._connection.close()
            self._connection = None
        except:
            carb.log_info(
                "Mavlink connection was not closed, because it was never opened"
            )

    def start(self):
        """Method that handles the begining of the simulation of vehicle. It will try to open the mavlink connection
        interface and also attemp to launch px4 in a background process if that option as specified in the config class
        """

        # If we are already running the mavlink interface, then ignore the function call
        if self._is_running == True:
            return

        # If the connection no longer exists (we stoped and re-started the stream, then re_intialize the interface)
        if self._connection is None:
            self.re_initialize_interface()

        # Set the flag to signal that the mavlink transmission has started
        self._is_running = True

        # Launch the PX4 in the background if needed
        if self.px4_autolaunch and self.px4_tool is None:
            carb.log_info("Attempting to launch PX4 in background process")

            # Get GPS coordinates from PegasusInterface
            pegasus_interface = PegasusInterface()
            latitude = pegasus_interface.latitude_deg
            longitude = pegasus_interface.longitude_deg

            self.px4_tool = PX4LaunchTool(
                self.px4_dir,
                self._vehicle_id,
                self.px4_vehicle_model,
                latitude,
                longitude,
            )
            self.px4_tool.launch_px4()

    def stop(self):
        """Method that when called will handle the stopping of the simulation of vehicle. It will make sure that any open
        mavlink connection will be closed and also that the PX4 background process gets killed (if it was auto-initialized)
        """

        # If the simulation was already stoped, then ignore the function call
        if self._is_running == False:
            return

        # Set the flag so that we are no longer running the mavlink interface
        self._is_running = False

        # Close the mavlink connection
        self._connection.close()
        self._connection = None

        # Close the PX4 if it was running
        if self.px4_autolaunch and self.px4_autolaunch is not None:
            carb.log_info("Attempting to kill PX4 background process")
            self.px4_tool.kill_px4()
            self.px4_tool = None

    def reset(self):
        """For now does nothing. Here for compatibility purposes only"""
        return

    def re_initialize_interface(self):
        """Auxiliar method used to get the MavlinkInterface to reset the MavlinkInterface to its initial state"""

        self._is_running = False

        # Restart the sensor data
        self._sensor_data = SensorMsg()

        # Restart the connection
        carb.log_info(f"Connection to backend at {self._connection_port}")
        self._connection = mavutil.mavlink_connection(self._connection_port)

        # Auxiliar variables to handle the lockstep between receiving sensor data and actuator control
        self._received_first_actuator: bool = False
        self._received_actuator: bool = False

        # Auxiliar variables to check if we have already received an hearbeat from the software in the loop simulation
        self._received_first_hearbeat: bool = False

        self._last_heartbeat_sent_time = 0

    def _configure_mavlink_telemetry(self):
        """Configure PX4 to send full telemetry to port 14540 for MAVSDK"""
        if not self.px4_tool or not self.px4_tool.px4_process:
            carb.log_warn("Cannot configure MAVLink telemetry - PX4 not running")
            return

        try:
            carb.log_info("🔧 Configuring MAVLink telemetry stream for MAVSDK...")

            # Get the PX4 process stdin handle
            px4_stdin = self.px4_tool.px4_process.stdin

            # Check current MAVLink status
            status_cmd = "mavlink status\n"
            px4_stdin.write(status_cmd)
            px4_stdin.flush()

            # Brief pause to let status print
            import time

            time.sleep(0.25)

            # Start MAVLink onboard instance for full telemetry to MAVSDK
            # -u 14557: UDP receive port
            # -o 14540: UDP output port (where MAVSDK listens)
            # -m onboard: Full telemetry mode (includes position/velocity)
            # -r 4000000: 4 Mbps data rate
            mavlink_cmd = "mavlink start -u 14557 -o 14540 -m onboard -r 4000000\n"
            carb.log_info(f"📡 Sending command: {mavlink_cmd.strip()}")

            px4_stdin.write(mavlink_cmd)
            px4_stdin.flush()

            # Verify configuration
            time.sleep(1.0)
            px4_stdin.write(status_cmd)
            px4_stdin.flush()

            carb.log_info(
                "✅ MAVLink telemetry configured - MAVSDK should now receive position/velocity on port 14540"
            )

        except Exception as e:
            carb.log_error(f"Failed to configure MAVLink telemetry: {e}")

    def wait_for_first_hearbeat(self):
        """
        Responsible for waiting for the first hearbeat. This method is locking and will only return
        if an hearbeat is received via mavlink. When this first heartbeat is received poll for mavlink messages
        """

        # Wait for the connection to be established
        if self._connection is None:
            return

        carb.log_warn("Waiting for first hearbeat")
        result = self._connection.wait_heartbeat(blocking=False)

        if result is not None:
            self._received_first_hearbeat = True
            carb.log_warn("Received first hearbeat")

            # Configure MAVLink telemetry for MAVSDK after heartbeat
            self._configure_mavlink_telemetry()

    def update(self, dt):
        """
        Method that is called at every physics step to send data to px4 and receive the control inputs via mavlink

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """

        # Check for the first hearbeat on the first few iterations
        if not self._received_first_hearbeat:
            self.wait_for_first_hearbeat()
            return

        # Check if we have already received IMU data. If not, start the lockstep and wait for more data
        if self._sensor_data.received_first_imu:
            while not self._sensor_data.new_imu_data and self._is_running:
                # Just go for the next update and then try to check if we have new simulated sensor data
                # DO not continue and get mavlink thrusters commands until we have simulated IMU data available
                return

        # Check if we have received any mavlink messages
        self.poll_mavlink_messages()

        # Send hearbeats at 1Hz
        if (
            time.time() - self._last_heartbeat_sent_time
        ) > 1.0 or self._received_first_hearbeat == False:
            self.send_heartbeat()
            self._last_heartbeat_sent_time = time.time()

        # Update the current u_time for px4
        self._current_utime += int(dt * 1000000)

        # Send sensor messages
        self.send_sensor_msgs(self._current_utime)

        # Send the GPS messages
        self.send_gps_msgs(self._current_utime)

    def poll_mavlink_messages(self):
        """
        Method that is used to check if new mavlink messages were received
        """

        # If we have not received the first hearbeat yet, do not poll for mavlink messages
        if self._received_first_hearbeat == False:
            return

        # Check if we need to lock and wait for actuator control data
        needs_to_wait_for_actuator: bool = (
            self._received_first_actuator and self._enable_lockstep
        )

        # Start by assuming that we have not received data for the actuators for the current step
        self._received_actuator = False

        # Use this loop to emulate a do-while loop (make sure this runs at least once)
        while True:

            # Try to get a message
            msg = self._connection.recv_match(blocking=needs_to_wait_for_actuator)

            # If a message was received
            if msg is not None:

                # Check if it is of the type that contains actuator controls
                if msg.id == mavutil.mavlink.MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:

                    self._received_first_actuator = True
                    self._received_actuator = True

                    # Handle the control of the actuation commands received by PX4
                    self.handle_control(
                        msg.time_usec, msg.controls, msg.mode, msg.flags
                    )

            # Check if we do not need to wait for an actuator message or we just received actuator input
            # If so, break out of the infinite loop
            if not needs_to_wait_for_actuator or self._received_actuator:
                break

    def send_heartbeat(self, mav_type=mavutil.mavlink.MAV_TYPE_GENERIC):
        """
        Method that is used to publish an heartbear through mavlink protocol

        Args:
            mav_type (int): The ID that indicates the type of vehicle. Defaults to MAV_TYPE_GENERIC=0
        """

        # carb.log_info("Sending heartbeat")

        # Note: to know more about these functions, go to pymavlink->dialects->v20->standard.py
        # This contains the definitions for sending the hearbeat and simulated sensor messages
        self._connection.mav.heartbeat_send(
            mav_type, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
        )

    def send_sensor_msgs(self, time_usec: int):
        """
        Method that when invoked, will send the simulated sensor data through mavlink

        Args:
            time_usec (int): The total time elapsed since the simulation started
        """
        # carb.log_info("Sending sensor msgs")

        # Check which sensors have new data to send
        fields_updated: int = 0

        if self._sensor_data.new_imu_data:
            # Set the bit field to signal that we are sending updated accelerometer and gyro data
            fields_updated = fields_updated | SensorSource.ACCEL | SensorSource.GYRO
            self._sensor_data.new_imu_data = False

        if self._sensor_data.new_mag_data:
            # Set the bit field to signal that we are sending updated magnetometer data
            fields_updated = fields_updated | SensorSource.MAG
            self._sensor_data.new_mag_data = False

        if self._sensor_data.new_bar_data:
            # Set the bit field to signal that we are sending updated barometer data
            fields_updated = fields_updated | SensorSource.BARO
            self._sensor_data.new_bar_data = False

        if self._sensor_data.new_press_data:
            # Set the bit field to signal that we are sending updated diff pressure data
            fields_updated = fields_updated | SensorSource.DIFF_PRESS
            self._sensor_data.new_press_data = False

        # Prepare sensor values with defaults if states are None
        xacc = yacc = zacc = 0.0
        xgyro = ygyro = zgyro = 0.0
        xmag = ymag = zmag = 0.0
        abs_pressure = pressure_alt = 0.0
        temperature = 0.0

        if self._sensor_data.imu_state:
            xacc = self._sensor_data.imu_state.linear_acceleration_frd_body_mpss[0]
            yacc = self._sensor_data.imu_state.linear_acceleration_frd_body_mpss[1]
            zacc = self._sensor_data.imu_state.linear_acceleration_frd_body_mpss[2]
            xgyro = self._sensor_data.imu_state.angular_velocity_frd_body_rps[0]
            ygyro = self._sensor_data.imu_state.angular_velocity_frd_body_rps[1]
            zgyro = self._sensor_data.imu_state.angular_velocity_frd_body_rps[2]

        if self._sensor_data.magnetometer_state:
            xmag = self._sensor_data.magnetometer_state.magnetic_field_frd_body_gauss[0]
            ymag = self._sensor_data.magnetometer_state.magnetic_field_frd_body_gauss[1]
            zmag = self._sensor_data.magnetometer_state.magnetic_field_frd_body_gauss[2]

        if self._sensor_data.barometer_state:
            abs_pressure = (
                self._sensor_data.barometer_state.pressure_pa * 0.01
            )  # Convert Pa to hPa
            pressure_alt = self._sensor_data.barometer_state.altitude_msl_m
            temperature = self._sensor_data.barometer_state.temperature_celsius

        # Get altitude from GPS if available
        altitude = 0.0
        if self._sensor_data.gps_state:
            altitude = self._sensor_data.gps_state.altitude_msl_m

        try:
            self._connection.mav.hil_sensor_send(
                time_usec,
                xacc,
                yacc,
                zacc,
                xgyro,
                ygyro,
                zgyro,
                xmag,
                ymag,
                zmag,
                abs_pressure,
                self._sensor_data.diff_pressure,  # Still using old format for airspeed
                pressure_alt,
                altitude,
                fields_updated,
            )
        except:
            carb.log_warn("Could not send sensor data through mavlink")

    def send_gps_msgs(self, time_usec: int):
        """
        Method that is used to send simulated GPS data through the mavlink protocol.

        Args:
            time_usec (int): The total time elapsed since the simulation started
        """
        # carb.log_info("Sending GPS msgs")

        # Do not send GPS data, if no new data was received
        if not self._sensor_data.new_gps_data or not self._sensor_data.gps_state:
            return

        self._sensor_data.new_gps_data = False

        gps: GPSState = self._sensor_data.gps_state

        # Convert to mavlink integer format
        latitude_deg_e7 = int(gps.latitude_deg * 10000000)
        longitude_deg_e7 = int(gps.longitude_deg * 10000000)
        altitude_mm = int(gps.altitude_msl_m * 1000)
        eph = int(gps.horizontal_position_error_m * 100)  # Convert m to cm
        epv = int(gps.vertical_position_error_m * 100)  # Convert m to cm
        velocity = int(gps.ground_speed_mps * 100)  # Convert m/s to cm/s
        velocity_north = int(gps.velocity_north_mps * 100)  # Convert m/s to cm/s
        velocity_east = int(gps.velocity_east_mps * 100)  # Convert m/s to cm/s
        velocity_down = int(gps.velocity_down_mps * 100)  # Convert m/s to cm/s

        try:
            self._connection.mav.hil_gps_send(
                time_usec,
                gps.fix_type,
                latitude_deg_e7,
                longitude_deg_e7,
                altitude_mm,
                eph,
                epv,
                velocity,
                velocity_north,
                velocity_east,
                velocity_down,
                gps.course_over_ground_cdeg,  # Already in centidegrees
                gps.satellites_visible,
            )
        except:
            carb.log_warn("Could not send gps data through mavlink")

    def send_ground_truth(self, time_usec: int):
        """
        Method that is used to send the groundtruth data of the vehicle through mavlink

        Args:
            time_usec (int): The total time elapsed since the simulation started
        """

        carb.log_info("Sending groundtruth msgs")

        # Do not send if no new data or no simulation state
        if (
            not self._sensor_data.new_sim_state
            or not self._sensor_data.simulation_state
        ):
            return

        if self._sensor_data.simulation_state.altitude_msl_m == 0:
            return  # Don't send if altitude is exactly zero

        self._sensor_data.new_sim_state = False

        sim = self._sensor_data.simulation_state

        # Convert to mavlink integer format
        latitude_deg_e7 = int(sim.latitude_deg * 10000000)
        longitude_deg_e7 = int(sim.longitude_deg * 10000000)
        altitude_mm = int(sim.altitude_msl_m * 1000)
        velocity_n_cm = int(sim.velocity_ned_mps[0] * 100)
        velocity_e_cm = int(sim.velocity_ned_mps[1] * 100)
        velocity_d_cm = int(sim.velocity_ned_mps[2] * 100)
        ind_airspeed_cm = int(sim.indicated_airspeed_mps * 100)
        true_airspeed_cm = int(sim.true_airspeed_mps * 100)
        xacc_mg = int(sim.acceleration_ned_mpss[0] * 1000)
        yacc_mg = int(sim.acceleration_ned_mpss[1] * 1000)
        zacc_mg = int(sim.acceleration_ned_mpss[2] * 1000)

        try:
            self._connection.mav.hil_state_quaternion_send(
                time_usec,
                sim.attitude_quat_wxyz_frd_ned,  # Already in [qw, qx, qy, qz] format
                sim.angular_velocity_frd_body_rps[0],
                sim.angular_velocity_frd_body_rps[1],
                sim.angular_velocity_frd_body_rps[2],
                latitude_deg_e7,
                longitude_deg_e7,
                altitude_mm,
                velocity_n_cm,
                velocity_e_cm,
                velocity_d_cm,
                ind_airspeed_cm,
                true_airspeed_cm,
                xacc_mg,
                yacc_mg,
                zacc_mg,
            )
        except:
            carb.log_warn("Could not send groundtruth through mavlink")

    def handle_control(self, time_usec, controls, mode, flags):
        """
        Method that when received a control message, compute the forces simulated force that should be applied
        on each rotor of the vehicle

        Args:
            time_usec (int): The total time elapsed since the simulation started - Ignored argument
            controls (list): A list of ints which contains the thrust_control received via mavlink
            flags: Ignored argument
        """

        # Check if the vehicle is armed - Note: here we have to add a +1 since the code for armed is 128, but
        # pymavlink is return 129 (the end of the buffer)
        if mode == mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED + 1:

            # carb.log_info("Parsing control input")

            # Set the rotor target speeds
            self._rotor_data.update_input_reference(controls)

        # If the vehicle is not armed, do not rotate the propellers
        else:
            self._rotor_data.zero_input_reference()

    def update_graphical_sensor(self, sensor_type: str, data):
        """Method that when implemented, should handle the receival of graphical sensor data

        Args:
            sensor_type (str): A name that describes the type of sensor (for example MonocularCamera)
            data (dict): A dictionary that contains the data produced by the sensor
        """
        pass

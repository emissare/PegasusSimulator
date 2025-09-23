"""
| File: px4_launch_tool.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| Description: Defines an auxiliary tool to launch the PX4 process in the background
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
"""

# System tools used to launch the px4 process in the brackground
import os
import tempfile
import subprocess
import threading

# Omniverse logging
import carb


class PX4LaunchTool:
    """
    A class that manages the start/stop of a px4 process. It requires only the path to the PX4 installation (assuming that
    PX4 was already built with 'make px4_sitl_default none'), the vehicle id and the vehicle model. 
    """

    def __init__(self, px4_dir, vehicle_id: int = 0, px4_model: str = "gazebo-classic_iris", latitude: float = 0.0, longitude: float = 0.0):
        """Construct the PX4LaunchTool object

        Args:
            px4_dir (str): A string with the path to the PX4-Autopilot directory
            vehicle_id (int): The ID of the vehicle. Defaults to 0.
            px4_model (str): The vehicle model. Defaults to "iris".
            latitude (float): The latitude of the GPS origin. Defaults to 0.0.
            longitude (float): The longitude of the GPS origin. Defaults to 0.0.
        """

        # Attribute that will hold the px4 process once it is running
        self.px4_process = None

        # The vehicle id (used for the mavlink port open in the system)
        self.vehicle_id = vehicle_id

        # GPS origin coordinates
        self.latitude = latitude
        self.longitude = longitude

        # Flag to track if monitoring thread is running
        self._monitoring = False

        # Configurations to whether autostart px4 (SITL) automatically or have the user launch it manually on another
        # terminal
        self.px4_dir = px4_dir
        self.rc_script = self.px4_dir + "/ROMFS/px4fmu_common/init.d-posix/rcS"

        # Create a temporary filesystem for px4 to write data to/from (and modify the origin rcS files)
        self.root_fs = tempfile.TemporaryDirectory()

        # Set the environement variables that let PX4 know which vehicle model to use internally
        self.environment = os.environ
        self.environment["PX4_SIM_MODEL"] = px4_model

        # Set GPS origin coordinates for PX4
        self.environment["PX4_HOME_LAT"] = str(self.latitude)
        self.environment["PX4_HOME_LON"] = str(self.longitude)

    def launch_px4(self):
        """
        Method that will launch a px4 instance with the specified configuration
        """
        carb.log_info(f"Launching PX4 SITL (Vehicle ID: {self.vehicle_id})")
        carb.log_info(f"GPS Origin: {self.latitude}, {self.longitude}")

        self.px4_process = subprocess.Popen(
            [
                self.px4_dir + "/build/px4_sitl_default/bin/px4",
                self.px4_dir + "/ROMFS/px4fmu_common/",
                "-s",
                self.rc_script,
                "-i",
                str(self.vehicle_id),
                "-d",
            ],
            cwd=self.root_fs.name,
            shell=False,
            env=self.environment,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )

        # Start monitoring thread to capture and log PX4 output
        self._monitoring = True
        monitor_thread = threading.Thread(target=self._monitor_px4_output, daemon=True)
        monitor_thread.start()

        carb.log_info(f"PX4 SITL started (PID: {self.px4_process.pid})")

    def _monitor_px4_output(self):
        """Monitor PX4 process output and log to Isaac Sim console"""
        carb.log_info("Starting PX4 output monitoring...")

        try:
            while self._monitoring and self.px4_process and self.px4_process.poll() is None:
                try:
                    line = self.px4_process.stdout.readline()
                    if line:
                        line = line.strip()
                        if line:
                            # Color-code different types of messages for better visibility
                            if any(keyword in line.lower() for keyword in ['error', 'failed', 'illegal']):
                                carb.log_error(f"PX4: {line}")
                            elif any(keyword in line.lower() for keyword in ['warning', 'warn']):
                                carb.log_warn(f"PX4: {line}")
                            elif any(keyword in line.lower() for keyword in ['mavlink', 'udp', 'tcp', 'port', 'stream']):
                                carb.log_info(f"📡 PX4: {line}")
                            elif any(keyword in line.lower() for keyword in ['simulator', 'sih', 'sensors']):
                                carb.log_info(f"🎮 PX4: {line}")
                            elif any(keyword in line.lower() for keyword in ['startup', 'init', 'loading']):
                                carb.log_info(f"🚀 PX4: {line}")
                            else:
                                carb.log_info(f"PX4: {line}")
                    else:
                        # No more output and process might have ended
                        break
                except Exception as e:
                    carb.log_error(f"Error reading PX4 output line: {e}")
                    break

            # Process has ended, capture any remaining output
            if self.px4_process and self.px4_process.poll() is not None:
                carb.log_warn(f"PX4 process ended with code: {self.px4_process.poll()}")
                try:
                    # Read any remaining buffered output
                    remaining_output = self.px4_process.stdout.read()
                    if remaining_output:
                        for line in remaining_output.split('\n'):
                            line = line.strip()
                            if line:
                                carb.log_info(f"PX4 (final): {line}")
                except Exception as e:
                    carb.log_error(f"Error reading final PX4 output: {e}")

        except Exception as e:
            carb.log_error(f"Error monitoring PX4 output: {e}")

        carb.log_info("PX4 output monitoring ended")

    def kill_px4(self):
        """
        Method that will kill a px4 instance with the specified configuration
        """
        if self.px4_process is not None:
            carb.log_info("Stopping PX4 SITL...")

            # Stop monitoring thread
            self._monitoring = False

            try:
                # Try graceful termination first
                self.px4_process.terminate()
                self.px4_process.wait(timeout=5)
                carb.log_info("PX4 SITL stopped gracefully")
            except subprocess.TimeoutExpired:
                carb.log_warn("PX4 didn't stop gracefully, forcing kill")
                self.px4_process.kill()
                self.px4_process.wait()
                carb.log_info("PX4 SITL force killed")
            except Exception as e:
                carb.log_error(f"Error stopping PX4: {e}")

            self.px4_process = None

    def __del__(self):
        """
        If the px4 process is still running when the PX4 launch tool object is whiped from memory, then make sure
        we kill the px4 instance so we don't end up with hanged px4 instances
        """

        # Stop monitoring thread
        self._monitoring = False

        # Make sure the PX4 process gets killed
        if self.px4_process:
            self.kill_px4()

        # Make sure we clean the temporary filesystem used for the simulation
        self.root_fs.cleanup()


# ---- Code used for debugging the px4 tool ----
def main():

    px4_tool = PX4LaunchTool(os.environ["HOME"] + "/PX4-Autopilot", 0, "gazebo-classic_iris", 32.77463, -117.07953)
    px4_tool.launch_px4()

    import time

    time.sleep(60)


if __name__ == "__main__":
    main()

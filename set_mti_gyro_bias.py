import sys
import xsensdeviceapi as xda
from threading import Lock
import numpy as np
import math
import struct
import time
import datetime
import os

# Set up logging functionality
class Logger:
    def __init__(self):
        # Create logs directory if it doesn't exist
        if not os.path.exists('logs'):
            os.makedirs('logs')
            
        # Create log file with timestamp in filename
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_filename = f"logs/log_{timestamp}.txt"
        
        # Open the log file
        self.log_file = open(self.log_filename, 'w')
        
        # Store the original stdout
        self.original_stdout = sys.stdout
        
    def write(self, message):
        # Write to both console and log file
        self.original_stdout.write(message)
        self.log_file.write(message)
        self.log_file.flush()  # Ensure data is written immediately
        
    def flush(self):
        # Required for compatibility with stdout
        self.original_stdout.flush()
        self.log_file.flush()
        
    def close(self):
        # Close the log file
        self.log_file.close()
        print(f"Log file saved to: {self.log_filename}")

class XdaCallback(xda.XsCallback):
    def __init__(self, max_buffer_size = 5):
        xda.XsCallback.__init__(self)
        self.m_maxNumberOfPacketsInBuffer = max_buffer_size
        self.m_packetBuffer = list()
        self.m_lock = Lock()

    def packetAvailable(self):
        self.m_lock.acquire()
        res = len(self.m_packetBuffer) > 0
        self.m_lock.release()
        return res

    def getNextPacket(self):
        self.m_lock.acquire()
        assert(len(self.m_packetBuffer) > 0)
        oldest_packet = xda.XsDataPacket(self.m_packetBuffer.pop(0))
        self.m_lock.release()
        return oldest_packet

    def onLiveDataAvailable(self, dev, packet):
        self.m_lock.acquire()
        assert(packet != 0)
        while len(self.m_packetBuffer) >= self.m_maxNumberOfPacketsInBuffer:
            self.m_packetBuffer.pop()
        self.m_packetBuffer.append(xda.XsDataPacket(packet))
        self.m_lock.release()

def requestGyroBias(device):
    snd = xda.XsMessage(0x78, 0x02)
    snd.setDataByte(0x05, 1)
    print(f"request the gyro bias, full xbus message to send: {snd.toHexString()}")
    rcv = xda.XsMessage()
    if device.sendCustomMessage(snd, True, rcv, 500):
        if rcv.getMessageId() == 0x79:
            print(f"Message RequestRateOfTurnOffset ACK received successfully: {rcv.toHexString()}")
            raw_rcv = bytes(rcv.rawMessage())
            gyro_bias_x = struct.unpack('>f', raw_rcv[6:10])[0]
            gyro_bias_y = struct.unpack('>f', raw_rcv[10:14])[0]
            gyro_bias_z = struct.unpack('>f', raw_rcv[14:18])[0]
            print(f"Received gyro bias (deg/sec): X = {gyro_bias_x}, Y = {gyro_bias_y}, Z = {gyro_bias_z}")
            gyro_bias = np.array([gyro_bias_x, gyro_bias_y, gyro_bias_z])
            return gyro_bias
    return None  # Return None if the request failed or no data received

def adjustGyroBias(device, biasX=0.0, biasY=0.0, biasZ=0.0):
    snd = xda.XsMessage(0x78, 0x0E)
    snd.setDataByte(0x05, 1)
    print(f"set the gyro bias X/Y/Z to = {biasX}, {biasY}, {biasZ} deg/sec.")
    snd.setDataFloat(biasX, 2)
    snd.setDataFloat(biasY, 6)
    snd.setDataFloat(biasZ, 10)

    print(f"set the gyro bias, full xbus message to send: {snd.toHexString()}")
    rcv = xda.XsMessage()
    if device.sendCustomMessage(snd, True, rcv, 500):
        if rcv.getMessageId() == 0x79:
            print(f"Message setRateOfTurnOffset ACK received successfully, {rcv.toHexString()}")
            return True
    return False


def adjust_gyro_bias_interactively(device, original_gyro_bias, calculated_gyro_bias, calculated_std):
    """Interactive function to adjust gyro bias based on user choice"""
    # Present options to the user
    print("\n===== APPLY CALIBRATION =====")
    print("Options:")
    print("  1) Apply the calculated gyro bias values")
    print(f"     Calculated values (deg/sec): X={calculated_gyro_bias[0]:.4f}, Y={calculated_gyro_bias[1]:.4f}, Z={calculated_gyro_bias[2]:.4f}")
    print("  2) Keep the original gyro bias values")
    print(f"     Original values (deg/sec): X={original_gyro_bias[0]:.4f}, Y={original_gyro_bias[1]:.4f}, Z={original_gyro_bias[2]:.4f}")
    print("  3) Enter your own custom gyro bias values")
    print("  4) Revert to factory default values (0.0, 0.0, 0.0)")
    
    # Get user input
    user_choice = input("Enter your choice (1, 2, 3, or 4): ")
    
    # Process user choice
    if user_choice == '1':
        # Option 1: Use calculated values
        if adjustGyroBias(device, calculated_gyro_bias[0], calculated_gyro_bias[1], calculated_gyro_bias[2]):
            print("Successfully applied calculated gyro bias values.")
            return True
        else:
            print("Failed to apply calculated gyro bias values.")
            return False
    elif user_choice == '2':
        # Option 2: Keep original values
        if adjustGyroBias(device, original_gyro_bias[0], original_gyro_bias[1], original_gyro_bias[2]):
            print("Restored original gyro bias values.")
            return True
        else:
            print("Failed to restore original gyro bias values.")
            return False
    elif user_choice == '3':
        # Option 3: User provides custom values
        print("Enter custom gyro bias values (in deg/sec):")
        try:
            bias_x = float(input("X-axis bias: "))
            bias_y = float(input("Y-axis bias: "))
            bias_z = float(input("Z-axis bias: "))
            
            if adjustGyroBias(device, bias_x, bias_y, bias_z):
                print("Successfully applied custom gyro bias values.")
                return True
            else:
                print("Failed to apply custom gyro bias values.")
                return False
        except ValueError:
            print("Invalid input. Values must be numbers.")
            return False
    elif user_choice == '4':
        # Option 4: Factory defaults (zeros)
        if adjustGyroBias(device, 0.0, 0.0, 0.0):
            print("Successfully reverted to factory default gyro bias values (0.0, 0.0, 0.0).")
            return True
        else:
            print("Failed to revert to factory default gyro bias values.")
            return False
    else:
        print("Invalid choice. No changes made to gyro bias values.")
        return False
    
def collect_gyro_data(device, callback, position_name, seconds_to_measure=5):
    """Collect gyro data for a specific position"""
    print(f"\nPlease place the sensor in the {position_name} position.")
    user_ready = input("Type 'Y' when the sensor is correctly positioned: ")
    
    if user_ready.upper() != 'Y':
        print("Skipping this position...")
        return None
    
    print(f"Collecting data for {position_name} for {seconds_to_measure} seconds...")
    gyro_data = []
    rad2deg = 180.0 / math.pi
    
    # Clear any old packets
    while callback.packetAvailable():
        callback.getNextPacket()
    
    startTime = xda.XsTimeStamp_nowMs()
    measurement_time = seconds_to_measure * 1000  # Convert to milliseconds
    
    while xda.XsTimeStamp_nowMs() - startTime <= measurement_time:
        if callback.packetAvailable():
            packet = callback.getNextPacket()
            
            if packet.containsCalibratedGyroscopeData():
                gyr = packet.calibratedGyroscopeData()
                gyr *= rad2deg
                gyro_data.append(gyr)
                
                # Print current reading (overwrite line)
                s = f"Gyr X: {gyr[0]:.2f}, Gyr Y: {gyr[1]:.2f}, Gyr Z: {gyr[2]:.2f}"
                print(f"{s}\r", end="", flush=True)
    
    print(f"\nCollected {len(gyro_data)} samples for {position_name}")
    
    if len(gyro_data) == 0:
        print("Warning: No data collected for this position!")
        return None
    
    return np.array(gyro_data)

def calculate_gyro_bias(position_data):
    """Calculate gyro bias from all positions data"""
    # Combine all data from all positions
    all_x_data = np.concatenate([pos[:, 0] for pos in position_data if pos is not None])
    all_y_data = np.concatenate([pos[:, 1] for pos in position_data if pos is not None])
    all_z_data = np.concatenate([pos[:, 2] for pos in position_data if pos is not None])
    
    # Calculate offsets
    offset_x = np.mean(all_x_data)
    offset_y = np.mean(all_y_data)
    offset_z = np.mean(all_z_data)
    
    # Calculate standard deviations for quality assessment
    std_x = np.std(all_x_data)
    std_y = np.std(all_y_data)
    std_z = np.std(all_z_data)
    
    return np.array([offset_x, offset_y, offset_z]), np.array([std_x, std_y, std_z])

def print_position_stats(position_name, gyro_data):
    """Print statistics for a single position"""
    if gyro_data is None or len(gyro_data) == 0:
        print(f"{position_name}: No data collected")
        return
        
    # Calculate means
    gyro_x_mean = np.mean(gyro_data[:, 0])
    gyro_y_mean = np.mean(gyro_data[:, 1])
    gyro_z_mean = np.mean(gyro_data[:, 2])
    
    # Calculate standard deviations
    gyro_x_std = np.std(gyro_data[:, 0])
    gyro_y_std = np.std(gyro_data[:, 1])
    gyro_z_std = np.std(gyro_data[:, 2])
    
    print(f"\n{position_name} Statistics:")
    print(f"  X-axis (deg/sec) - Mean: {gyro_x_mean:.4f}, Std: {gyro_x_std:.4f}")
    print(f"  Y-axis (deg/sec) - Mean: {gyro_y_mean:.4f}, Std: {gyro_y_std:.4f}")
    print(f"  Z-axis (deg/sec) - Mean: {gyro_z_mean:.4f}, Std: {gyro_z_std:.4f}")


if __name__ == '__main__':
    # Initialize logger
    logger = Logger()
    sys.stdout = logger
    
    print(f"Starting IMU gyroscope calibration at {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("Creating XsControl object...")
    control = xda.XsControl_construct()
    assert(control != 0)

    xdaVersion = xda.XsVersion()
    xda.xdaVersion(xdaVersion)
    print("Using XDA version %s" % xdaVersion.toXsString())

    try:
        print("Scanning for devices...")
        portInfoArray = xda.XsScanner_scanPorts()

        # Find an MTi device
        mtPort = xda.XsPortInfo()
        for i in range(portInfoArray.size()):
            if portInfoArray[i].deviceId().isMti() or portInfoArray[i].deviceId().isMtig():
                mtPort = portInfoArray[i]
                break

        if mtPort.empty():
            raise RuntimeError("No MTi device found. Aborting.")

        did = mtPort.deviceId()
        print("Found a device with:")
        print(" Device ID: %s" % did.toXsString())
        print(" Port name: %s" % mtPort.portName())

        print("Opening port...")
        if not control.openPort(mtPort.portName(), mtPort.baudrate()):
            raise RuntimeError("Could not open port. Aborting.")

        # Get the device object
        device = control.device(did)
        assert(device != 0)

        print("Device: %s, with ID: %s opened." % (device.productCode(), device.deviceId().toXsString()))

        # Create and attach callback handler to device
        callback = XdaCallback()
        device.addCallbackHandler(callback)

        # Put the device into configuration mode before configuring the device
        print("Putting device into configuration mode...")
        if not device.gotoConfig():
            raise RuntimeError("Could not put device into configuration mode. Aborting.")

        # Get current gyro bias values
        original_gyro_bias_values = requestGyroBias(device)
        if original_gyro_bias_values is None:
            print("Failed to retrieve gyro bias values.")
            original_gyro_bias_values = np.array([0.0, 0.0, 0.0])
        
        print("Temporarily setting gyro offset to zeros for calibration...")
        adjustGyroBias(device, 0.0, 0.0, 0.0)
        
        print("Configuring the device...")
        configArray = xda.XsOutputConfigurationArray()
        configArray.push_back(xda.XsOutputConfiguration(xda.XDI_PacketCounter, 0xFFFF))
        configArray.push_back(xda.XsOutputConfiguration(xda.XDI_SampleTimeFine, 0xFFFF))
        configArray.push_back(xda.XsOutputConfiguration(xda.XDI_StatusWord, 0xFFFF))
        configArray.push_back(xda.XsOutputConfiguration(xda.XDI_Acceleration, 400))
        configArray.push_back(xda.XsOutputConfiguration(xda.XDI_RateOfTurn, 400))
        configArray.push_back(xda.XsOutputConfiguration(xda.XDI_MagneticField, 100))

        if device.deviceId().isVru() or device.deviceId().isAhrs():
            configArray.push_back(xda.XsOutputConfiguration(xda.XDI_Quaternion, 400))
            configArray.push_back(xda.XsOutputConfiguration(xda.XDI_FreeAcceleration, 400))
        elif device.deviceId().isGnssIns():
            configArray.push_back(xda.XsOutputConfiguration(xda.XDI_Quaternion, 400))
            configArray.push_back(xda.XsOutputConfiguration(xda.XDI_FreeAcceleration, 400))
            configArray.push_back(xda.XsOutputConfiguration(xda.XDI_LatLon | xda.XDI_SubFormatFp1632, 400))
            configArray.push_back(xda.XsOutputConfiguration(xda.XDI_AltitudeEllipsoid | xda.XDI_SubFormatFp1632, 400))
            configArray.push_back(xda.XsOutputConfiguration(xda.XDI_VelocityXYZ | xda.XDI_SubFormatFp1632, 400))
            configArray.push_back(xda.XsOutputConfiguration(xda.XDI_GnssPvtData, 0xFFFF))


        if not device.setOutputConfiguration(configArray):
            raise RuntimeError("Could not configure the device. Aborting.")

        print("Putting device into measurement mode...")
        if not device.gotoMeasurement():
            raise RuntimeError("Could not put device into measurement mode. Aborting.")

        # Define the six positions to measure
        positions = [
            "Z-up (Z axis pointing upward)",
            "Z-down (Z axis pointing downward)",
            "Y-up (Y axis pointing upward)",
            "Y-down (Y axis pointing downward)",
            "X-up (X axis pointing upward)",
            "X-down (X axis pointing downward)"
        ]

        # Collect data for each position
        seconds_per_position = 5  # Seconds to collect data for each position
        position_data = []
        
        print("\n===== IMU GYROSCOPE CALIBRATION =====")
        print("This procedure will collect gyroscope data in six different orientations")
        print("to calculate the gyroscope bias offsets.\n")
        print("Please ensure the sensor is completely stationary during each measurement.")
        
        for position in positions:
            data = collect_gyro_data(device, callback, position, seconds_per_position)
            position_data.append(data)
            # Print statistics for this position
            if data is not None:
                print_position_stats(position, data)
        
        # Calculate gyro bias based on all positions
        valid_positions = [p for p in position_data if p is not None and len(p) > 0]
        
        if len(valid_positions) == 0:
            print("\nNo valid data collected from any position. Cannot calculate gyro bias.")
        else:
            gyro_bias, gyro_std = calculate_gyro_bias(valid_positions)
            
            print("\n===== CALIBRATION RESULTS =====")
            print(f"Calculated Gyro Bias (deg/sec):")
            print(f"  X-axis: {gyro_bias[0]:.4f} ± {gyro_std[0]:.4f}")
            print(f"  Y-axis: {gyro_bias[1]:.4f} ± {gyro_std[1]:.4f}")
            print(f"  Z-axis: {gyro_bias[2]:.4f} ± {gyro_std[2]:.4f}")
            
            print("\n===== QUALITY ASSESSMENT =====")
            # Check if standard deviations are acceptably low
            std_threshold = 0.20  # deg/sec
            quality_issues = []
            
            if gyro_std[0] > std_threshold:
                quality_issues.append(f"X-axis variability is high ({gyro_std[0]:.4f} deg/sec)")
            if gyro_std[1] > std_threshold:
                quality_issues.append(f"Y-axis variability is high ({gyro_std[1]:.4f} deg/sec)")
            if gyro_std[2] > std_threshold:
                quality_issues.append(f"Z-axis variability is high ({gyro_std[2]:.4f} deg/sec)")
                
            if quality_issues:
                print("Potential calibration quality issues:")
                for issue in quality_issues:
                    print(f"  - {issue}")
                print("Consider repeating the calibration with the sensor more stable.")
            else:
                print("Calibration quality looks good. Standard deviations are within acceptable limits.")
            
            # Put device back into config mode for setting bias
            print("Putting device into configuration mode...")
            if not device.gotoConfig():
                raise RuntimeError("Could not put device into configuration mode. Aborting.")
            
            # Use the interactive function to handle user choice
            adjust_gyro_bias_interactively(device, original_gyro_bias_values, gyro_bias, gyro_std)
        
        print("Removing callback handler...")
        device.removeCallbackHandler(callback)
        
        print("Closing port...")
        control.closePort(mtPort.portName())

        print("Closing XsControl object...")
        control.close()

    except RuntimeError as error:
        print(error)
        sys.exit(1)
    except Exception as e:
        print(f"Unexpected error: {e}")
        sys.exit(1)
    else:
        print("Successful exit.")
    finally:
        # Restore original stdout and close log file
        sys.stdout = logger.original_stdout
        logger.close()      
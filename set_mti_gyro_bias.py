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
    """Request current gyro bias values from the device"""
    snd = xda.XsMessage(0x78, 0x02)
    snd.setDataByte(0x05, 1)
    print(f"Requesting gyro bias, full xbus message to send: {snd.toHexString()}")
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
    """Set gyro bias values on the device"""
    snd = xda.XsMessage(0x78, 0x0E)
    snd.setDataByte(0x05, 1)
    print(f"Setting gyro bias X/Y/Z to = {biasX}, {biasY}, {biasZ} deg/sec.")
    snd.setDataFloat(biasX, 2)
    snd.setDataFloat(biasY, 6)
    snd.setDataFloat(biasZ, 10)

    print(f"Setting gyro bias, full xbus message to send: {snd.toHexString()}")
    rcv = xda.XsMessage()
    if device.sendCustomMessage(snd, True, rcv, 500):
        if rcv.getMessageId() == 0x79:
            print(f"Message setRateOfTurnOffset ACK received successfully, {rcv.toHexString()}")
            return True
    return False

def collect_gyro_data(device, callback, seconds_to_measure=5):
    """Collect gyro data while the device is stationary"""
    print(f"\nCollecting gyroscope data for {seconds_to_measure} seconds...")
    print("Please ensure the device remains completely stationary during measurement.")
    
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
    
    print(f"\nCollected {len(gyro_data)} samples")
    
    if len(gyro_data) == 0:
        print("Warning: No data collected!")
        return None
    
    return np.array(gyro_data)

def calculate_gyro_bias(gyro_data):
    """Calculate gyro bias from collected data"""
    if gyro_data is None or len(gyro_data) == 0:
        return None, None
    
    # Calculate offsets (means)
    offset_x = np.mean(gyro_data[:, 0])
    offset_y = np.mean(gyro_data[:, 1])
    offset_z = np.mean(gyro_data[:, 2])
    
    # Calculate standard deviations for quality assessment
    std_x = np.std(gyro_data[:, 0])
    std_y = np.std(gyro_data[:, 1])
    std_z = np.std(gyro_data[:, 2])
    
    return np.array([offset_x, offset_y, offset_z]), np.array([std_x, std_y, std_z])

def print_gyro_stats(gyro_data):
    """Print statistics for the collected gyro data"""
    if gyro_data is None or len(gyro_data) == 0:
        print("No data available")
        return
        
    # Calculate statistics
    gyro_x_mean = np.mean(gyro_data[:, 0])
    gyro_y_mean = np.mean(gyro_data[:, 1])
    gyro_z_mean = np.mean(gyro_data[:, 2])
    
    gyro_x_std = np.std(gyro_data[:, 0])
    gyro_y_std = np.std(gyro_data[:, 1])
    gyro_z_std = np.std(gyro_data[:, 2])
    
    gyro_x_min = np.min(gyro_data[:, 0])
    gyro_y_min = np.min(gyro_data[:, 1])
    gyro_z_min = np.min(gyro_data[:, 2])
    
    gyro_x_max = np.max(gyro_data[:, 0])
    gyro_y_max = np.max(gyro_data[:, 1])
    gyro_z_max = np.max(gyro_data[:, 2])
    
    print("\n===== MEASUREMENT STATISTICS =====")
    print(f"X-axis (deg/sec) - Mean: {gyro_x_mean:.4f}, Std: {gyro_x_std:.4f}, Min: {gyro_x_min:.4f}, Max: {gyro_x_max:.4f}")
    print(f"Y-axis (deg/sec) - Mean: {gyro_y_mean:.4f}, Std: {gyro_y_std:.4f}, Min: {gyro_y_min:.4f}, Max: {gyro_y_max:.4f}")
    print(f"Z-axis (deg/sec) - Mean: {gyro_z_mean:.4f}, Std: {gyro_z_std:.4f}, Min: {gyro_z_min:.4f}, Max: {gyro_z_max:.4f}")

def view_current_bias(device):
    """View the current gyro bias settings"""
    print("\n===== VIEW CURRENT GYRO BIAS =====")
    
    # Put device into config mode
    print("Putting device into configuration mode...")
    if not device.gotoConfig():
        print("Could not put device into configuration mode.")
        return
    
    bias = requestGyroBias(device)
    
    # Put device back into measurement mode
    print("Putting device back into measurement mode...")
    if not device.gotoMeasurement():
        print("Warning: Could not put device back into measurement mode.")
    
    if bias is not None:
        print(f"\nCurrent Gyro Bias Values (deg/sec):")
        print(f"  X-axis: {bias[0]:.4f}")
        print(f"  Y-axis: {bias[1]:.4f}")
        print(f"  Z-axis: {bias[2]:.4f}")
    else:
        print("Failed to retrieve gyro bias values.")

def reset_bias_to_factory(device):
    """Reset gyro bias to factory defaults (0, 0, 0)"""
    print("\n===== RESET GYRO BIAS TO FACTORY DEFAULTS =====")
    
    confirm = input("Are you sure you want to reset to factory defaults (0.0, 0.0, 0.0)? (Y/N): ")
    if confirm.upper() != 'Y':
        print("Reset cancelled.")
        return
    
    # Put device into config mode
    print("Putting device into configuration mode...")
    if not device.gotoConfig():
        print("Could not put device into configuration mode.")
        return
    
    if adjustGyroBias(device, 0.0, 0.0, 0.0):
        print("Successfully reset gyro bias to factory defaults.")
    else:
        print("Failed to reset gyro bias.")
    
    # Put device back into measurement mode
    print("Putting device back into measurement mode...")
    if not device.gotoMeasurement():
        print("Warning: Could not put device back into measurement mode.")

def set_custom_bias(device):
    """Set custom gyro bias values"""
    print("\n===== SET CUSTOM GYRO BIAS VALUES =====")
    print("Enter custom gyro bias values (in deg/sec):")
    
    try:
        bias_x = float(input("X-axis bias: "))
        bias_y = float(input("Y-axis bias: "))
        bias_z = float(input("Z-axis bias: "))
        
        confirm = input(f"\nApply bias values X={bias_x:.4f}, Y={bias_y:.4f}, Z={bias_z:.4f}? (Y/N): ")
        if confirm.upper() != 'Y':
            print("Operation cancelled.")
            return
        
        # Put device into config mode
        print("Putting device into configuration mode...")
        if not device.gotoConfig():
            print("Could not put device into configuration mode.")
            return
        
        if adjustGyroBias(device, bias_x, bias_y, bias_z):
            print("Successfully applied custom gyro bias values.")
        else:
            print("Failed to apply custom gyro bias values.")
        
        # Put device back into measurement mode
        print("Putting device back into measurement mode...")
        if not device.gotoMeasurement():
            print("Warning: Could not put device back into measurement mode.")
            
    except ValueError:
        print("Invalid input. Values must be numbers.")

def estimate_and_apply_bias(device, callback):
    """Estimate gyro bias and apply it"""
    print("\n===== ESTIMATE AND APPLY GYRO BIAS =====")
    print("The device will collect gyroscope data while stationary to estimate the bias.")
    print("Please ensure:")
    print("  - The device is placed on a stable surface")
    print("  - No vibrations or movements")
    print("  - No people walking nearby")
    print("  - No external disturbances")
    
    ready = input("\nType 'Y' when ready to start measurement: ")
    if ready.upper() != 'Y':
        print("Measurement cancelled.")
        return
    
    # Collect data
    gyro_data = collect_gyro_data(device, callback, seconds_to_measure=5)
    
    if gyro_data is None:
        print("Failed to collect gyroscope data.")
        return
    
    # Print statistics
    print_gyro_stats(gyro_data)
    
    # Calculate bias
    gyro_bias, gyro_std = calculate_gyro_bias(gyro_data)
    
    if gyro_bias is None:
        print("Failed to calculate gyro bias.")
        return
    
    print("\n===== CALCULATED GYRO BIAS =====")
    print(f"X-axis: {gyro_bias[0]:.4f} ± {gyro_std[0]:.4f} deg/sec")
    print(f"Y-axis: {gyro_bias[1]:.4f} ± {gyro_std[1]:.4f} deg/sec")
    print(f"Z-axis: {gyro_bias[2]:.4f} ± {gyro_std[2]:.4f} deg/sec")
    
    # Quality assessment
    print("\n===== QUALITY ASSESSMENT =====")
    std_threshold = 0.20  # deg/sec
    quality_issues = []
    
    if gyro_std[0] > std_threshold:
        quality_issues.append(f"X-axis variability is high ({gyro_std[0]:.4f} deg/sec)")
    if gyro_std[1] > std_threshold:
        quality_issues.append(f"Y-axis variability is high ({gyro_std[1]:.4f} deg/sec)")
    if gyro_std[2] > std_threshold:
        quality_issues.append(f"Z-axis variability is high ({gyro_std[2]:.4f} deg/sec)")
        
    if quality_issues:
        print("Potential calibration quality issues detected:")
        for issue in quality_issues:
            print(f"  - {issue}")
        print("Consider repeating the measurement with better stability.")
    else:
        print("Calibration quality looks good. Standard deviations are within acceptable limits.")
    
    # Ask user if they want to apply
    apply = input("\nApply these calculated bias values? (Y/N): ")
    if apply.upper() != 'Y':
        print("Bias values not applied.")
        return
    
    # Put device into config mode
    print("Putting device into configuration mode...")
    if not device.gotoConfig():
        print("Could not put device into configuration mode.")
        return
    
    # Apply bias
    if adjustGyroBias(device, gyro_bias[0], gyro_bias[1], gyro_bias[2]):
        print("Successfully applied calculated gyro bias values.")
    else:
        print("Failed to apply gyro bias values.")
    
    # Put device back into measurement mode
    print("Putting device back into measurement mode...")
    if not device.gotoMeasurement():
        print("Warning: Could not put device back into measurement mode.")

def display_menu():
    """Display the main menu"""
    print("\n" + "="*50)
    print("MTi GYROSCOPE BIAS CONFIGURATION MENU")
    print("="*50)
    print("1. View current gyro bias values")
    print("2. Reset gyro bias to factory defaults")
    print("3. Set custom gyro bias values")
    print("4. Estimate and apply gyro bias")
    print("5. Exit")
    print("="*50)

def configure_device(device):
    """Configure the device output settings"""
    print("Configuring device output settings...")
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

if __name__ == '__main__':
    # Initialize logger
    logger = Logger()
    sys.stdout = logger
    
    print(f"Starting MTi gyroscope bias configuration at {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("Creating XsControl object...")
    control = xda.XsControl_construct()
    assert(control != 0)

    xdaVersion = xda.XsVersion()
    xda.xdaVersion(xdaVersion)
    print("Using XDA version %s" % xdaVersion.toXsString())

    device = None
    callback = None
    mtPort = None

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

        # Put the device into configuration mode
        print("Putting device into configuration mode...")
        if not device.gotoConfig():
            raise RuntimeError("Could not put device into configuration mode. Aborting.")

        # Configure device
        configure_device(device)

        # Put device into measurement mode
        print("Putting device into measurement mode...")
        if not device.gotoMeasurement():
            raise RuntimeError("Could not put device into measurement mode. Aborting.")

        # Main menu loop
        while True:
            display_menu()
            choice = input("Enter your choice (1-5): ")
            
            if choice == '1':
                view_current_bias(device)
            elif choice == '2':
                reset_bias_to_factory(device)
            elif choice == '3':
                set_custom_bias(device)
            elif choice == '4':
                estimate_and_apply_bias(device, callback)
            elif choice == '5':
                print("\nExiting program...")
                break
            else:
                print("\nInvalid choice. Please enter a number between 1 and 5.")

    except RuntimeError as error:
        print(f"Runtime error: {error}")
        sys.exit(1)
    except Exception as e:
        print(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        # Cleanup
        if device is not None and callback is not None:
            print("Removing callback handler...")
            device.removeCallbackHandler(callback)
        
        if control is not None and mtPort is not None and not mtPort.empty():
            print("Closing port...")
            control.closePort(mtPort.portName())

        if control is not None:
            print("Closing XsControl object...")
            control.close()

        # Restore original stdout and close log file
        sys.stdout = logger.original_stdout
        logger.close()
        print("Program terminated.")
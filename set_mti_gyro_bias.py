
#  Copyright (c) 2003-2024 Movella Technologies B.V. or subsidiaries worldwide.
#  All rights reserved.
#  
#  Redistribution and use in source and binary forms, with or without modification,
#  are permitted provided that the following conditions are met:
#  
#  1.	Redistributions of source code must retain the above copyright notice,
#  	this list of conditions, and the following disclaimer.
#  
#  2.	Redistributions in binary form must reproduce the above copyright notice,
#  	this list of conditions, and the following disclaimer in the documentation
#  	and/or other materials provided with the distribution.
#  
#  3.	Neither the names of the copyright holders nor the names of their contributors
#  	may be used to endorse or promote products derived from this software without
#  	specific prior written permission.
#  
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
#  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
#  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
#  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
#  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
#  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
#  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
#  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
#  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
#  

import sys
import xsensdeviceapi as xda
from threading import Lock
import numpy as np
import math
import struct

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

def adjust_gyro_bias_interactively(device, gyro_bias_values, gyro_means):
    # Present options to the user
    print("Choose an option for setting gyro bias:")
    print("1) Write the measured gyro mean values as the gyro bias")
    print("2) Write your own value to gyro bias")
    print("3) Revert to factory default values")
    print("4) Don't do anything")

    # Get user input
    choice = input("Enter your choice (1, 2, 3 or 4): ")
    
    if choice == '1':
        # Option 1: Use computed mean values
        if not adjustGyroBias(device, gyro_means[0], gyro_means[1], gyro_means[2]):
            print("Failed to set the gyro bias values.")
    elif choice == '2':
        # Option 2: User provides their own values
        user_values = input("Enter three numbers separated by commas without spaces (e.g., 1.0,2.0,3.0): ")
        values = list(map(float, user_values.split(',')))
        if len(values) != 3:
            print("Invalid input. Please enter exactly three numbers.")
        elif not adjustGyroBias(device, *values):
            print("Failed to set the gyro bias values.")
    elif choice == '3':
        # Option 3: Revert to factory default bias values
        if not adjustGyroBias(device, 0.0, 0.0, 0.0):
            print("Failed to revert the gyro bias values.")
    elif choice == '4':
        # Option 4: Revert to original bias values
        if not adjustGyroBias(device, gyro_bias_values[0], gyro_bias_values[1], gyro_bias_values[2]):
            print("Failed to revert the gyro bias values.")
    else:
        print("Invalid choice. No action taken.")


if __name__ == '__main__':
    print("Creating XsControl object...")
    control = xda.XsControl_construct()
    assert(control != 0)

    xdaVersion = xda.XsVersion()
    xda.xdaVersion(xdaVersion)
    print("Using XDA version %s" % xdaVersion.toXsString())

    try:
        print("Scanning for devices...")
        portInfoArray =  xda.XsScanner_scanPorts()

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

        gyro_bias_values = requestGyroBias(device)
        if gyro_bias_values is  None:
            print("Failed to retrieve gyro bias values.")
        
        print("Revert the gyro offset to zeros.")
        adjustGyroBias(device, 0.0, 0.0, 0.0)
        
        print("Configuring the device...")
        configArray = xda.XsOutputConfigurationArray()

        if device.deviceId().isVru() or device.deviceId().isAhrs() or device.deviceId().isGnss():
            configArray.push_back(xda.XsOutputConfiguration(xda.XDI_RateOfTurn, 400))
            configArray.push_back(xda.XsOutputConfiguration(xda.XDI_EulerAngles, 400))
        elif device.deviceId().isImu():
            configArray.push_back(xda.XsOutputConfiguration(xda.XDI_RateOfTurn, 400))
        else:
            raise RuntimeError("Unknown device while configuring. Aborting.")

        if not device.setOutputConfiguration(configArray):
            raise RuntimeError("Could not configure the device. Aborting.")

        print("Putting device into measurement mode...")
        if not device.gotoMeasurement():
            raise RuntimeError("Could not put device into measurement mode. Aborting.")

        gyro_data = []
        rad2deg = 180.0 / math.pi

        startTime = xda.XsTimeStamp_nowMs()
        seconds_to_measure = 100
        print(f"Main loop. Recording data for {seconds_to_measure} seconds.")
        seconds_to_measure *=1000
        
        while xda.XsTimeStamp_nowMs() - startTime <= seconds_to_measure:
            if callback.packetAvailable():
                # Retrieve a packet
                packet = callback.getNextPacket()

                s = ""

                if packet.containsCalibratedGyroscopeData():
                    gyr = packet.calibratedGyroscopeData()
                    gyr *= rad2deg
                    gyro_data.append(gyr)
                    s += "Gyr X: %.2f" % gyr[0] + ", Gyr Y: %.2f" % gyr[1] + ", Gyr Z: %.2f" % gyr[2]

                print("%s\r" % s, end="", flush=True)

        if isinstance(gyro_data, list):
            gyro_data = np.array(gyro_data)
        
        print("\nRemoving callback handler...")
        device.removeCallbackHandler(callback)
        
        print("Putting device into configuration mode again...")
        if not device.gotoConfig():
            raise RuntimeError("Could not put device into configuration mode. Aborting.")
        
        #now we need to calculate the mean, std, min and max of the gyro data.
        # Calculate means
        gyro_x_mean = np.mean(gyro_data[:, 0])
        gyro_y_mean = np.mean(gyro_data[:, 1])
        gyro_z_mean = np.mean(gyro_data[:, 2])
        gyro_means = np.array([gyro_x_mean, gyro_y_mean, gyro_z_mean])
        # Calculate standard deviations
        gyro_x_std = np.std(gyro_data[:, 0])
        gyro_y_std = np.std(gyro_data[:, 1])
        gyro_z_std = np.std(gyro_data[:, 2])

        # Calculate minimums
        gyro_x_min = np.min(gyro_data[:, 0])
        gyro_y_min = np.min(gyro_data[:, 1])
        gyro_z_min = np.min(gyro_data[:, 2])

        # Calculate maximums
        gyro_x_max = np.max(gyro_data[:, 0])
        gyro_y_max = np.max(gyro_data[:, 1])
        gyro_z_max = np.max(gyro_data[:, 2])
                    
        # Print the results
        print("Gyroscope X(deg/sec) - Mean: {:.2f}, Std: {:.2f}, Min: {:.2f}, Max: {:.2f}".format(gyro_x_mean, gyro_x_std, gyro_x_min, gyro_x_max))
        print("Gyroscope Y(deg/sec) - Mean: {:.2f}, Std: {:.2f}, Min: {:.2f}, Max: {:.2f}".format(gyro_y_mean, gyro_y_std, gyro_y_min, gyro_y_max))
        print("Gyroscope Z(deg/sec) - Mean: {:.2f}, Std: {:.2f}, Min: {:.2f}, Max: {:.2f}".format(gyro_z_mean, gyro_z_std, gyro_z_min, gyro_z_max))

        adjust_gyro_bias_interactively(device, gyro_bias_values, gyro_means)
        
        print("Closing port...")
        control.closePort(mtPort.portName())

        print("Closing XsControl object...")
        control.close()

    except RuntimeError as error:
        print(error)
        sys.exit(1)
    # except:
    #     print("An unknown fatal error has occured. Aborting.")
    #     sys.exit(1)
    else:
        print("Successful exit.")

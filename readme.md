# MTi-600 Series Additional Gyro Bias Setting

## Introduction

This code and firmware file is valid for all MTi-600 series devices (MTi-610, MTi-620, MTi-630, MTi-630R, MTi-670, MTi-680, MTi-680G, etc.).

The program provides an interactive menu to manage gyroscope bias settings. It can measure the sensor's rate of turn (in deg/sec) for a certain amount of time while the device is stationary, compute the mean values, and then allow you to apply the additional offset to the gyro.

The command adds extra offset on top of the factory default calibration values on the MTi, with the result written to the Non-volatile memory.

Based on the updated gyro bias calibration values, the orientation performance has a good chance to improve. The key to success is that the extra offset you measure needs to be accurate; in other words, there should be no interference from the surrounding environment.

## Update Firmware to v1.13.0

### Download and Install Firmware Updater
Download from [`Xsens Website`](https://www.movella.com/hubfs/FirmwareUpdater.zip)

Copy the firmware file [mti630_product-v1.13.0-master-b0-r1706185132.xff](./firmware_file/mti630_product-v1.13.0-master-b0-r1706185132.xff) to the installation folder of Firmware Updater Software.

### Update Firmware from Command Line
```
firmwareupdater_gui64.exe -f mti630_product-v1.13.0-master-b0-r1706185132.xff -s
```

![Alt text](./images/firmware_updater_with_xff.png)

Then the GUI Firmware Updater will pop up, click Next:
![Alt text](./images/firmware_updater_found_device.png)

![Alt text](./images/firmware_updater_updating.png)

Now that it is successful, click "Finish".
![Alt text](./images/firmware_updater_success.png)

## Install the Xsens Device API Library with Virtual Environment

```
python -m venv venv
venv\Scripts\activate
pip install -r requirements.txt
```

Note: The wheel file for your Python version must be available. Supported versions are Python 3.8, 3.9, and 3.10.

## How to Run the Program

Please make sure the MTi device is in a very quiet place, without interference from people walking by, knocking on the table, or machine vibrations, then run:
```
python set_mti_gyro_bias.py
```

The program will present an interactive menu with the following options:
- **View current gyro bias values** - Display the current bias settings stored on the device
- **Reset gyro bias to factory defaults** - Revert to factory calibration (0.0, 0.0, 0.0)
- **Set custom gyro bias values** - Manually enter your own bias values
- **Estimate and apply gyro bias** - Measure the bias while the device is stationary and apply the calculated values
- **Exit** - Exit the program without making changes

### Estimating Gyro Bias

When you select the "Estimate and apply gyro bias" option:
1. Ensure the device is completely stationary and free from vibrations
2. The program will collect gyroscope data for 5 seconds
3. Statistical analysis will be performed on the collected data
4. You will be prompted to either apply the calculated bias or exit without applying changes

## Note

In this repo, we included the Xsens Device API Python libraries for Python 3.8, 3.9, and 3.10. If you have a different version of Python, you can download the full MT Software Suite 2025.0 at [`Xsens Website`](https://www.movella.com/support/software-documentation). After installation, the Xsens Device API Python libraries are located at `C:\Program Files\Xsens\MT Software Suite 2025.0\MT SDK\Python\x64`

The library works for `x86 platform` CPUs, but not for ARM devices. For ARM devices, you can adapt the code based on the `C:\Program Files\Xsens\MT Software Suite 2025\MT SDK\Examples\xda_public_cpp`

For more details of the command, refer to [this doc](./firmware_file/Additional-Gyro-Bias-Offset_instructions.pdf)
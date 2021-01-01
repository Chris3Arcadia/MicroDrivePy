# MicroDrive 1.5 Documentation

**MicroDrive 1.5** by **Mad City Labs (MCL)** 

See MCL's [home page](http://www.madcitylabs.com/) or their  [micro-stages](http://www.madcitylabs.com/microstage.html)

[TOC]

## Handle Management

In order to communicate with a Mad City Labs Micro-Drive a handle must be obtained using one of the following functions. The handle obtained will be passed as an argument to every function that needs to communicate with the Micro-Drive. At the conclusion of your program all handles should be released. See examples below.

Single Micro-Drive:
```c
handle = MCL_InitHandle()
MCL_ReleaseHandle(handle)
```

Multiple Micro-Drives:
```c
numDevices = MCL_GrabAllHandles()
handle1 = MCL_GetHandleBySerial(/*serial number*/ 1600)
handle2 = MCL_GetHandleBySerial(/*serial number*/ 1601)
MCL_ReleaseAllHandles()
```


### *int* **MCL_InitHandle**()

Requests control of a single Mad City Labs Micro-Drive.

*Return Value*

Returns a valid handle or returns 0 to indicate failure.

*Notes* 

If multiple Mad City Labs Micro-Drives are attached but not yet controlled, it is indeterminate which of the uncontrolled Micro-Drives this function will gain control of.

Use a combination of MCL_GrabAllHandles, MCL_GetAllHandles, and MCL_GetHandleBySerial to acquire the handle to a specific device.


### *int* **MCL_InitHandleOrGetExisting**()

Request control of a single Mad City Labs Micro-Drive. If all attached Micro-Drives are controlled, this function will return a handle to one of the Micro-Drives currently controlled by the DLL.

This function was developed for LabView VIs as an alternative method of dealing with threads that acquire a handle and then fail to release it properly. It is an alternative to using MCL_NumberOfCurrentHandles and MCL_GetAllHandles.

*Return Value*

Returns a valid handle or returns 0 to indicate failure.

*Notes* 

If multiple Mad City Labs Micro-Drives are attached but not yet controlled, it is indeterminate which of the uncontrolled Micro-Drives this function will gain control of.

If all Micro-Drives are controlled by the DLL, multiple calls to this function will cycle through all of the controlled Micro-Drives.

### *int* **MCL_GrabHandle**(*short* device)

Requests control of a specific type of Mad City Labs Micro-Drive.

*Parameters*

device [IN] Specifies the type of Micro-Drive. See table below for valid types.

| Micro-Drive | 9472 | 0x2500 |
| --- | --- | --- |
| Micro-Drive1 | 9473 | 0x2501 |
| Micro-Drive3 | 9475 | 0x2503 |
| NanoCyte Micro-Drive | 13568 | 0x3500 |

*Return Value*

Returns a valid handle or returns 0 to indicate failure.

*Notes* 

If multiple Mad City Labs Micro-Drives of the same type are attached but not yet controlled, it is indeterminate which of the uncontrolled Micro-Drives this function will gain control of.

Use a combination of MCL_GrabAllHandles, MCL_GetAllHandles, and MCL_GetHandleBySerial to acquire the handle to a specific device.

### *int* **MCL_GrabHandleOrGetExisting**(*short* device)

Requests control of a specific type of Mad City Labs device. If all attached Micro-Drives of the specified type are controlled, this function will return a handle to one of the Micro-Drives of that type currently controlled by the DLL.

This function was developed for LabView VIs as an alternative method of dealing with threads that acquire a handle and then fail to release it properly. It is an alternative to using MCL_NumberOfCurrentHandles and MCL_GetAllHandles.

*Parameters*

device [IN] Specifies the type of Micro-Drive. See table below for valid types.

| Micro-Drive | 9472 | 0x2500 |
| --- | --- | --- |
| Micro-Drive1 | 9473 | 0x2501 |
| Micro-Drive3 | 9475 | 0x2503 |
| NanoCyte Micro-Drive | 13568 | 0x3500 |

*Return Value*

Returns a valid handle or returns 0 to indicate failure.

*Notes* 

If multiple Mad City Labs Micro-Drives of the specified type are attached but not yet controlled, it is indeterminate which of those Micro-Drives this function will gain control of.

If all the Micro-Drives of the specified type are currently controlled by the DLL, multiple calls to this function will cycle through all of the controlled Micro-Drives of the specified type.

### *int* **MCL_GrabAllHandles**()

Requests control of all of the attached Mad City Labs Micro-Drives that are not yet under control.

*Return Value*

Returns the number of Micro-Drives currently controlled by this instance of the DLL.

*Notes* 

After calling this function use MCL_GetHandleBySerialNumber to get the handle of a specific device.

Use MCL_NumberOfCurrentHandles and MCL_GetAllHandles to get a list of the handles acquired by this function.

Remember that this function will take control of all of the attached Micro-Drives not currently under control. Some of the aquired handles may need to be released if those Micro-Drives are needed in other applications.

### *int* **MCL_GetAllHandles**(*int* handles, *int* size)

Fills a list with valid handles to the Micro-Drives currently under the control of this instance of the DLL.

*Parameters*

- handles [IN/OUT] Pointer to an array of &#39;size&#39; integers.

- size [IN] Size of the &#39;handles&#39; array


*Return Value*

Returns the number of valid handles put into the &#39;handles&#39; array.

### *int* **MCL_NumberOfCurrentHandles**()

Returns the number of Micro-Drives currently controlled by this instance of the DLL.

*Return Value*

Number of Micro-Drives controlled.

### *int* **MCL_GetHandleBySerial**(*short* serial)

Searches Micro-Drives currently controlled for a Micro-Drive whose serial number matches &#39;serial&#39;.

*Parameters*

- serial [IN] Serial # of the Micro-Drive whose handle you want to lookup.


*Return Value*

Returns a valid handle or returns 0 to indicate failure.

*Notes* 

Since this function only searches through Micro-Drives which the DLL is controlling, MCL_GrabAllHandles() or multiple calls to MCL_(Init/Grab)Handle should be called before using this function.

### *void* **MCL_ReleaseHandle**(*int* handle)

Releases control of the specified Micro-Drive.

### *void* **MCL_ReleaseAllHandles**()

Releases control of all Micro-Drives controlled by this instance of the DLL.

## Motion Control

### *int* **MCL_MicroDriveMoveStatus**(*int* isMoving, *int* handle)

Queries the device to see if it is moving. This function should be called prior to reading the encoders as the encoders should not be read when the stage is in motion.

*Parameters*

- isMoving [IN/OUT] Is set to &#39;1&#39; if the stage is moving or &#39;0&#39; otherwise. On error isMoving is set to &#39;0&#39;.

- handle [IN] Specifies which Micro-Drive to communicate with.


*Return Value*

Returns MCL_SUCCESS or the appropriate error code.

### *int* **MCL_MicroDriveWait**(*int* handle)

Waits long enough for the previously commanded move to finish.

*Parameters*

- handle [IN] Specifies which Micro-Drive to communicate with.


*Return Value*

Returns MCL_SUCCESS or the appropriate error code.

### *int* **MCL_MicroDriveStatus**(*unsigned char* status, *int* handle)

Reads the limit switches.

*Parameters*

- status [IN/OUT] x6543210. See **Stage Status Flags**.


- handle [IN] Specifies which Micro-Drive to communicate with.


*Return Value*

Returns MCL_SUCCESS or the appropriate error code.

### *int* **MCL_MicroDriveStop**(*unsigned char* status, *int* handle)

Stops the stage from moving.

*Parameters*

- status [IN/OUT] x6543210. See **Stage Status Flags**.

- handle [IN] Specifies which Micro-Drive to communicate with.


*Return Value*

Returns MCL_SUCCESS or the appropriate error code.


## Movement and Encoders for MicroDrive

### *int* **MCL_MicroDriveMoveProfileXYZ_MicroSteps**(*double*  velocityX, *int*  microStepsX, *double* velocityY, *int* microStepsY, *double*  velocityZ, *int* microStepsZ, *int* handle)

Standard movement function. Acceleration and deceleration ramps are generated for the specified motion. In some cases when taking smaller steps the velocity parameter may be coerced to its maximum achievable value. The maximum and minimum velocities can be found using MCL_MicroDriveInformation. The maximum velocity differs depending on how many axes are commaned to move.

*Requirements* **Specific to the following devices Micro-Drive(0x2500), NanoCyte MicroDrive(0x3500), and Micro-Drive3(0x2503)**

*Parameters*

- velocity(X/Y/Z) [IN] Speed in mm/sec.

- microSteps(X/Y/Z) [IN] Number of microsteps to move the stage. A positive number of microsteps moves the stage toward its forward limit switch. A negative number of microsteps moves it toward its reverse limit switch. Zero will result in the axis not moving.

- handle [IN] Specifies which Micro-Drive to communicate with.


*Return Value*

Returns MCL_SUCCESS or the appropriate error code.

*Notes* 

Care should be taken not to access the Micro-Drive while the microstage is moving for any reason other than stopping it. Doing so will adversely affect the internal timers of the Micro-Drive which generate the required step pulses for the specified movement.

### *int* **MCL_MicroDriveMoveProfile_MicroSteps**(*unsigned int* axis, *double* velocity, *int*  microSteps, *int* handle)

Standard movement function. Acceleration and deceleration ramps are generated for the specified motion. In some cases when taking smaller steps the velocity parameter may be coerced to its maximum achievable value. The maximum and minimum velocities can be found using MCL_MicroDriveInformation.

*Requirements* **Specific to the following devices Micro-Drive(0x2500), NanoCyte MicroDrive(0x3500), and Micro-Drive3(0x2503)**

*Parameters*

- axis [IN] Which axis to move. (X=1,Y=2,Z=3)

- velocity [IN] Speed in mm/sec.

- microSteps [IN] Number of microsteps to move the stage. A positive number of microsteps moves the stage toward its forward limit switch. A negative number of microsteps moves it toward its reverse limit switch. Zero will result in the axis not moving.

- handle [IN] Specifies which Micro-Drive to communicate with.


*Return Value*

Returns MCL_SUCCESS or the appropriate error code.

*Notes* 

Care should be taken not to access the Micro-Drive while the microstage is moving for any reason other than stopping it. Doing so will adversely affect the internal timers of the Micro-Drive which generate the required step pulses for the specified movement.

### *int* **MCL_MicroDriveMoveProfileXYZ**(*double* velocityX, *double* distanceX, *int* roundingX, *double* velocityY, *double* distanceY, *int* roundingY, *double* velocityZ, *double* distanceZ, *int* roundingZ, *int* handle)

Standard movement function. Acceleration and deceleration ramps are generated for the specified motion. In some cases when taking smaller steps the velocity parameter may be coerced to its maximum achievable value. The maximum and minimum velocities can be found using MCL_MicroDriveInformation. The maximum velocity differs depending on how many axes are commaned to move.

*Requirements* **Specific to the following devices Micro-Drive(0x2500), NanoCyte MicroDrive(0x3500), and Micro-Drive3(0x2503)**

*Parameters*

- velocity(X/Y/Z) [IN] Speed in mm/sec.

- distance(X/Y/Z) [IN] Distance in mm to move the stage. Positive distances move the stage toward its forward limit switch. Negative distances move it toward its reverse limit switch. A value of 0.0 will result in the axis not moving.

- rounding(X/Y/Z) [IN] Determines how to round the distance parameter. See **Position Rouding Options**.

- handle [IN] Specifies which Micro-Drive to communicate with.


*Return Value*

Returns MCL_SUCCESS or the appropriate error code.

*Notes* 

Care should be taken not to access the Micro-Drive while the microstage is moving for any reason other than stopping it. Doing so will adversely affect the internal timers of the Micro-Drive which generate the required step pulses for the specified movement.

### *int* **MCL_MicroDriveMoveProfile**(*unsigned int* axis, *double* velocity, *double*  distance, *int*  rounding, *int* handle)

Standard movement function. Acceleration and deceleration ramps are generated for the specified motion. In some cases when taking smaller steps the velocity parameter may be coerced to its maximum achievable value. The maximum and minimum velocities can be found using MCL_MicroDriveInformation.

*Requirements* **Specific to the following devices Micro-Drive(0x2500), NanoCyte MicroDrive(0x3500), and Micro-Drive3(0x2503)**

*Parameters*

- axis [IN] Which axis to move. (X=1,Y=2,Z=3)

- velocity [IN] Speed in mm/sec.

- distance [IN] Distance in mm to move the stage. Positive distances move the stage toward its forward limit switch. Negative distances move it toward its reverse limit switch. A value of 0.0 will result in the axis not moving.

- rounding [IN] Determines how to round the distance parameter. See **Position Rouding Options**.

- handle [IN] Specifies which Micro-Drive to communicate with.


*Return Value*

Returns MCL_SUCCESS or the appropriate error code.

*Notes* 

Care should be taken not to access the Micro-Drive while the microstage is moving for any reason other than stopping it. Doing so will adversely affect the internal timers of the Micro-Drive which generate the required step pulses for the specified movement.

### *int* **MCL_MicroDriveSingleStep**(*unsigned int*  axis, *int* direction, *int* handle)

Takes a single step in the specified direction.

*Requirements* **Specific to the following devices Micro-Drive(0x2500), NanoCyte MicroDrive(0x3500), and Micro-Drive3(0x2503)**

*Parameters*

- axis [IN] Which axis to move. (X=1,Y=2, Z=3)

- directions [IN] 1 = move forward, -1 = move reverse

- handle [IN] Specifies which Micro-Drive to communicate with.


*Return Value*

Returns MCL_SUCCESS or the appropriate error code.

### *int* **MCL_MicroDriveResetEncoders**(*unsigned char* status, *int* handle)

### *int* **MCL_MicroDriveResetXEncoder**(*unsigned char* status, *int* handle)

### *int* **MCL_MicroDriveResetYEncoder**(*unsigned char* status, *int* handle)

### *int* **MCL_MicroDriveResetZEncoder**(*unsigned char* status, *int* handle)

Resetting an encoder makes the current position of the microstage the zero position of the encoder. Use _MCL_MicroDriveResetEncoders_ to reset all encoders or _MCL_MicroDriveReset(X/YZ)Encoder_ to reset a specifc encoder.

*Requirements* **Specific to the following devices Micro-Drive(0x2500), NanoCyte MicroDrive(0x3500), and Micro-Drive3(0x2503)**

*Parameters*

- status [IN/OUT] x6543210. See **Stage Status Flags**.

- handle [IN] Specifies which Micro-Drive to communicate with.


*Return Value*

Returns MCL_SUCCESS or the appropriate error code.

### *int* **MCL_MicroDriveReadEncoders**(*double* x, *double* y, *double* z, *int* handle)

Reads all encoders.

*Requirements* **Specific to the following devices Micro-Drive(0x2500), NanoCyte MicroDrive(0x3500), and Micro-Drive3(0x2503)**

*Parameters*

- x [IN/OUT] Set to the value of the X encoder.

- y [IN/OUT] Set to the value of the Y encoder if axis is available.

- z [IN/OUT] Set to the value of the Z encoder if axis is available.

- handle [IN] Specifies which Micro-Drive to communicate with.


*Return Value*

Returns MCL_SUCCESS or the appropriate error code.

*Notes* 

The position values may be inverted. The inversion is a result of how the encoder sensor was attached and varies by microstage design.

### *int* **MCL_MicroDriveCurrentMicroStepPosition**(*int* microStepsX, *int* microStepsY, *int* microStepsZ, *int* handle)

Reads the number of microsteps taken since the beginning of the program.

*Requirements* **Specific to the following devices Micro-Drive(0x2500), NanoCyte MicroDrive(0x3500), and Micro-Drive3(0x2503)**

*Parameters*

- microStepsX [IN/OUT] A valid pointer is set to the number of microsteps taken in X.

- microStepsY [IN/OUT] A valid pointer is set to the number of microsteps taken in Y.

- microStepsZ [IN/OUT] A valid pointer is set to the number of microsteps taken in Y.

- handle [IN] Specifies which Micro-Drive to communicate with.


*Notes* 

This function will fail if the stage is still moving when it is called.

*Return Value*

Returns MCL_SUCCESS or the appropriate error code.

### *int* **MCL_MicroDriveInformation**(*double* encoderResolution, *double* stepSize, *double* maxVelocity, *double* maxVelocityTwoAxis, *double* maxVelocityThreeAxis, *double* minVelocity, *int* handle)

Gather Information about the resolution and speed of the Micro-Drive.

*Requirements* **Specific to the following devices Micro-Drive(0x2500), NanoCyte MicroDrive(0x3500), and Micro-Drive3(0x2503)**

*Parameters*

- encoderResolution [IN/OUT] Set to the encoder resolution in um.

- stepSize [IN/OUT] Set to the size of a single step in mm.

- maxVelocity [IN/OUT] Set to the maximum velocity in mm of a single axis move.

- maxVelocityTwoAxis [IN/OUT] Set to the maximum velocity in mm of a two axis move.

- maxVelocityThreeAxis [IN/OUT] Set to the maximum velocity in mm of a three axis move.

- minVelocity [IN/OUT] Set to the minimum velocity in mm of a move.

- handle [IN] Specifies which Micro-Drive to communicate with.


*Return Value*

Returns MCL_SUCCESS or the appropriate error code.

## Movement and Encoders for MicroDrive1

### *int* **MCL_MD1MoveProfile_MicroSteps**(*double* velocity, *int* microSteps, *int* handle)

Standard movement function. Acceleration and deceleration ramps are generated for the specified motion. In some cases when taking smaller steps the velocity parameter may be coerced to its maximum achievable value. The maximum and minimum velocities can be found using MCL_MicroDriveInformation.

*Requirements* **Specific to the Micro-Drive1(0x2501)**

*Parameters*

- velocity [IN] Speed in mm/sec.

- microSteps [IN] Number of microsteps to move the stage. A positive number of microsteps moves the stage toward its forward limit switch. A negative number of microsteps moves it toward its reverse limit switch.

- handle [IN] Specifies which Micro-Drive to communicate with.


*Return Value*

Returns MCL_SUCCESS or the appropriate error code.

*Notes* 

Care should be taken not to access the Micro-Drive while the microstage is moving for any reason other than stopping it. Doing so will adversely affect the internal timers of the Micro-Drive which generate the required step pulses for the specified movement.

### *int* **MCL_MD1MoveProfile**(*double* velocity, *double* distance, *int* rounding, *int* handle)

Standard movement function. Acceleration and deceleration ramps are generated for the specified motion. In some cases when taking smaller steps the velocity parameter may be coerced to its maximum achievable value. The maximum and minimum velocities can be found using MCL_MicroDriveInformation.

*Requirements*: **Specific to the Micro-Drive1(0x2501)**

*Parameters*

- velocity [IN] Speed in mm/sec.

- distance [IN] Distance in mm to move the stage. Positive distances move the stage toward its forward limit switch. Negative distances move it toward its reverse limit switch.

- rounding [IN] Determines how to round the distance parameter. See **Position Rouding Options**.

- handle [IN] Specifies which Micro-Drive to communicate with.


*Return Value*

Returns MCL_SUCCESS or the appropriate error code.

*Notes* 

Care should be taken not to access the Micro-Drive while the microstage is moving for any reason other than stopping it. Doing so will adversely affect the internal timers of the Micro-Drive which generate the required step pulses for the specified movement.

### *int* **MCL_MD1SingleStep**(*int* direction, *int* handle)

Takes a single step in the specified direction.

*Requirements* **Specific to the Micro-Drive1(0x2501)**

*Parameters*

- directions [IN] 1 = move forward, -1 = move reverse

- handle [IN] Specifies which Micro-Drive to communicate with.


*Return Value*

Returns MCL_SUCCESS or the appropriate error code.

### *int* **MCL_MD1ResetEncoder**(*unsigned char* status, *int* handle)

Resetting an encoder makes the current position of the microstage the zero position of the encoder.

*Requirements* **Specific to the Micro-Drive1(0x2501)**

*Parameters*

- status [IN/OUT] xxx43210. See **Stage Status Flags**.

- handle [IN] Specifies which Micro-Drive to communicate with.


*Return Value*

Returns MCL_SUCCESS or the appropriate error code.

### *Int* **MCL_MD1ReadEncoder**(*double* position, *int* handle)

Reads the encoder.

*Requirements* **Specific to the Micro-Drive1(0x2501)**

*Parameters*

- position [IN/OUT] Set to the value of the encoder.

- handle [IN] Specifies which Micro-Drive to communicate with.


*Return Value*

Returns MCL_SUCCESS or the appropriate error code.

### *int* **MCL_MD1CurrentMicroStepPosition**(*int* microSteps, *int* handle)

Reads the number of microsteps taken since the beginning of the program.

*Requirements* **Specific to the Micro-Drive1(0x2501)**

*Parameters*

- microSteps [IN/OUT] A valid pointer is set to the number of microsteps taken.

- handle [IN] Specifies which Micro-Drive to communicate with.


*Notes* 

This function will fail if the stage is still moving when it is called.

*Return Value*

Returns MCL_SUCCESS or the appropriate error code.

### *int* **MCL_MD1Information**(*double* encoderResolution, *double* stepSize, *double* maxVelocity, *double* minVelocity, *int* handle)

Gather Information about the resolution and speed of the Micro-Drive.

*Requirements* **Specific to the Micro-Drive1(0x2501)**

*Parameters*

- encoderResolution [IN/OUT] Set to the encoder resolution in um.

- stepSize [IN/OUT] Set to the size of a single step in mm.

- maxVelocity [IN/OUT] Set to the maximum velocity in mm of a single axis move.

- minVelocity [IN/OUT] Set to the minimum velocity in mm of a single axis move.

- handle [IN] Specifies which Micro-Drive to communicate with.


*Return Value*

Returns MCL_SUCCESS or the appropriate error code.

## Device Information

### *bool* **MCL_DeviceAttached**(*unsigned int*  milliseconds, *int* handle)

Function waits for a specified number of milliseconds then reports whether or not the Micro-Drive is attached.

*Parameters*

- milliseconds [IN] Indicates how long to wait.

- handle [IN] Specifies which Micro-Drive to communicate with.


*Return Value*

Returns true if the specified Micro-Drive is attached and false if it is not.

### *int* **MCL_GetFirmwareVersion**(*short* version, *short* profile, *int* handle)

Gives access to the Firmware version and profile information.

*Parameters*

- version [IN/OUT] Pointer to the address to be filled with the firmware version number.

- profile [IN/OUT] Pointer to the address to be filled with the firmware profile number.

- handle [IN] Specifies which Micro-Drive to communicate with.


*Return Value*

Returns MCL_SUCCESS or the appropriate error code.

### *void* **MCL_PrintDeviceInfo**(*int* handle)

Prints to the console product name, product id, DLL version, firmware version, and some product specific information.

### *int* **MCL_GetSerialNumber**(*int* handle)

Returns the serial number of the Micro-Drive. This information can be useful if you need support for your device or if you are attempting to tell the difference between two similar Micro-Drives.

*Parameters*

- handle [IN] Specifies which Micro-Drive to communicate with.


*Return Value*

Returns the serial number of the specified Micro-Drive.

### *void* **MCL_DLLVersion**(*short* version, *short* revision)

Gives access to the DLL version information. This information is useful if you need support.

*Parameters*

- version [IN/OUT] Valid addresses filled with DLL version #

- revision [IN/OUT] Valid addresses filled with DLL revision #


### *bool* **MCL_CorrectDriverVersion**()

Checks if the DLL was built against the driver version currently installed.

*Return Value*

True if the DLL was built against the current driver version.

### *int* **MCL_GetProductID**(*unsigned short*  PID,  *int* handle)

Allows the program to query the product id of the device represented by &#39;handle&#39;.

*Parameters*

- PID [IN/OUT] Valid addresses filled with product ID number.

- handle [IN] Specifies which Micro-Drive to communicate with.


*Return Value*

Returns MCL_SUCCESS or the appropriate error code.


## Reference Tables

### Error Codes

| Message | Code | Description |
| --- | --- | --- |
| `MCL_SUCCESS` | 0 | Task has been completed successfully. |
| `MCL_GENERAL_ERROR` | -1  | These errors generally occur due to an internal sanity check failing. |
| `MCL_DEV_ERROR` | -2 | A problem occurred when transferring data to the Micro-Drive. It is likely that the Micro-Drive will have to be power cycled to correct these errors. |
| `MCL_DEV_NOT_ATTACHED` | -3 | The Micro-Drive cannot complete the task because it is not attached. |
| `MCL_USAGE_ERROR` | -4 | Using a function from the library which the Micro-Drive does not support causes these errors. |
| `MCL_DEV_NOT_READY` | -5 | The Micro-Drive is currently completing or waiting to complete another task. |
| `MCL_ARGUMENT_ERROR` | -6 | An argument is out of range or a required pointer is equal to NULL. |
| `MCL_INVALID_AXIS` | -7 | Attempting an operation on an axis that does not exist in the Micro-Drive. |
| `MCL_INVALID_HANDLE` | -8 | The handle is not valid. Or at least is not valid in this instance of the DLL. |



### Stage Status Flags

Micro-Drive3 (0x2503) & Micro-Drive (0x2500):
| Bit  | Description            |
| ---- | ---------------------- |
| 6    | Success/Failure!       |
| 5    | Z Forward Limit Switch |
| 4    | Z Reverse Limit Switch |
| 3    | Y Forward Limit Switch |
| 2    | Y Reverse Limit Switch |
| 1    | X Forward Limit Switch |
| 0    | X Reverse Limit Switch |

NanoCyte Micro-Drive(0x3500)
| Bit  | Description            |
| ---- | ---------------------- |
| 4    | Success/Failure!       |
| 3    | Y Forward Limit Switch |
| 2    | Y Reverse Limit Switch |
| 1    | X Forward Limit Switch |
| 0    | X Reverse Limit Switch |

Micro-Drive1 (0x2501)
| Bit  | Description          |
| ---- | -------------------- |
| 4    | Success/Failure!     |
| 3    | Forward Limit Switch |
| 2    | Reverse Limit Switch |


### Position Rouding Options

| Value | Description |
| --- | --- |
| 0 | Nearest microstep. |
| 1 | Nearest full step. |
|  2 | Nearest half step. |
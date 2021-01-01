# Read Me

The library provides a Python class wrapper for Mad City Labs' MicroDrive Positioning Stage Controller.

By Chris Arcadia 

Created on 2020/12/30. 

Intended for use with the MicroDrive3 controller and the MMP3-Series 3-Axis Stepper Motor Stage.

## Note

This code base adapted on 2020/12/29 from the PipTap repository on Bitbucket (specifically from "piptap/Library/stage/stepper/class") by Chris Arcadia. 

Inspired by the following Repositories: 
  * "StepperStage.m" (from "PipTap" by Chris Arcadia, in MATLAB, on [Bitbucket](https://bitbucket.org/christopher_arcadia/piptap))
  * "mcl_nanodrive.py" (from "ScopeFoundryHW.mcl_stage" by Edward Barnard, in Python, on [GitHub]())
  * "mcl_piezo_lib.py" (from "mclpiezo" by Yurii Morozov, in Python, on [GitHub](https://github.com/yurmor/mclpiezo))


## Software Requirements
* Mad City Labs "MicroDrive" driver
* [Anaconda 3](https://www.anaconda.com/) (`Anaconda3-4.4.0-Windows-x86.exe` from the [installer archive](https://repo.anaconda.com/archive/) was used for development)
* for Python to access the driver, use the [ctypes](https://docs.python.org/3/library/ctypes.html) package


## Hardware Requirements

* Mad City Labs MicroDrive and compatible stepper stage.
* CPU with USB 2 and enough RAM for MATLAB to run.
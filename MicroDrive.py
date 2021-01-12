# # Python Class for Mad City Labs' MicroDrive Positioning Stage Controller
#
# By Chris Arcadia (2020/12/30)
#
# Intended for use with the MMP3-Series 3-Axis Stepper Motor Stage
#
# Inspired by the following Repositories: 
#   * "StepperStage.m" (from "PipTap" by Chris Arcadia, in MATLAB, on [Bitbucket](https://bitbucket.org/christopher_arcadia/piptap))
#   * "mcl_nanodrive.py" (from "ScopeFoundryHW.mcl_stage" by Edward Barnard, in Python, on [GitHub]())
#   * "mcl_piezo_lib.py" (from "mclpiezo" by Yurii Morozov, in Python, on [GitHub](https://github.com/yurmor/mclpiezo))

import ctypes #import cdll, c_int, c_uint, c_double
import time
import numpy
import os

class MicroDrive():
    def __init__(self):
        self.load_options()
        self.load_constants()
        self.load_microdrive3()
        self.set_path()    
        self.load_library()
        self.connect()
        self.load_device_info()
        self.read()

    def load_options(self):
        self.verbose = True
        self.record = True # unimplemented option to save each read step or position value to text (along with a timestamp)

    def load_constants(self):

        self.responses = {
               0: 'MCL_SUCCESS',
              -1: 'MCL_GENERAL_ERROR',
              -2: 'MCL_DEV_ERROR',
              -3: 'MCL_DEV_NOT_ATTACHED',
              -4: 'MCL_USAGE_ERROR',
              -5: 'MCL_DEV_NOT_READY',
              -6: 'MCL_ARGUMENT_ERROR',
              -7: 'MCL_INVALID_AXIS',
              -8: 'MCL_INVALID_HANDL',
        } # Mad City Labs Stage Response Integer to Error Code

    def load_microdrive3(self):
        # load constants specific to Micro-Drive3

        self.movements = {
            7: 'Success/Failure!',
            6: 'Z Forward Limit Switch',
            5: 'Z Reverse Limit Switch',
            4: 'Y Forward Limit Switch',
            3: 'Y Reverse Limit Switch',
            2: 'X Forward Limit Switch',
            1: 'X Reverse Limit Switch',
        } # Mad City Labs Micro-Drive 3 Response Bit to Movement Status

        self.rounding = {
            0: 'Nearest microstep', 
            1: 'Nearest full step', 
            2: 'Nearest half step',
        } # Mad City Labs Micro-Drive 3 Rounding Level to Description

        self.hardware = {
            'interface':'USB 2', 
            'manufacturer': 'Mad City Labs (MCL)', 
            'controller':'MicroDrive',
            'stage':'MMP3',
        } # known hardware details

        self.axes = {
            'name':  ['x', 'y', 'z'],
            'index': [ 1 ,  2 ,  3 ],
        } # channel details
        self.axes['count'] = len(self.axes['name'])

        self.travel = {
            'velocity':     [1e-3, 1e-3, 1e-3], # default travel velocity [m/s]
            'velocityFast': [3e-3, 3e-3, 3e-3],  # default fast velocity [m/s] (for largescale movement like calibrating the stage)
            'rounding':     [2, 2, 2], # default rounding level for stepper motor position 
            'direction':    [1, 1, 1], # [-1,-1,1]; # default travel direction {-1: 'backward', 1: 'forward'}
        } # travel defaults

        self.units = {
            'velocity': 'm/s',
            'time': 's',
            'distance': 'm',
            'step': '',            
        }   

    def set_path(self,pathLib=None,pathOut=None):
        self.path = dict();        
        self.path['root'] = os.path.dirname(__file__)
        if not pathLib:
            # provide default path to the driver (.dll) file. The associated .h and .lib files should also be in the same folder.                
            pathLib = os.path.join(self.path['root'],'driver','MadCityLabs-64bit','MicroDrive.dll') 
        self.path['library'] = pathLib                            
        if not pathOut:
            # provide default output path
            pathOut = os.path.join(self.path['root'],'__temporary__')      
            self.ensure_path(pathOut)                
        self.path['output'] = pathOut                            

    def ensure_path(self,path):
        if not os.path.isdir(path):
            os.mkdir(path)

    def notify(self,message):
        if self.verbose:
            print('MicroDrive: '+message)

    def load_library(self):
        self.loaded = False        
        if os.path.isfile(self.path['library']):
            try:
                self.library = ctypes.cdll.LoadLibrary(self.path['library'])
                self.loaded = True
                self.notify('Loaded driver from "'+self.path['library']+'"')                                
            except:
                pass
            
    def unload_library(self):
        if self.loaded:
            pass # ctypes doesn't really support unloading DLL              
            
    def release(self):
        if self.loaded:
            self.library.MCL_ReleaseAllHandles()      

    def connect(self):
        self.connected = False        
        if self.loaded:
            try:
                self.release()
                #self.handle = self.library.MCL_InitHandle()
                self.handle = self.library.MCL_InitHandleOrGetExisting()
                if self.handle:
                    self.connected = True
                    self.notify('Device connected.')                                                
            except:
                pass
    
    def disconnect(self):
        if self.loaded and self.connected:            
            self.release()
            self.handle = None
            self.connected = False

    def get_success(self,code):
        return not(code < 0) 

    def get_message(self,code):    
        message = 'UNKNOWN'
        if code in self.responses:
            message = self.responses[code]    
        return message            

    def check_response(self,code):    
        success = self.get_success(code)
        message = self.get_message(code)
        if not(success) and self.verbose:
            self.notify('Error: '+message)
        return {'code':code, 'success':success, 'message':message}

    def check_success(self,code):
        check = self.check_response(code)
        return check['success']

    def get_library_version(self):
        version = None
        if self.loaded:
            # MCL_DLLVersion(short *version, short *revision);
            ver = ctypes.c_short(); rev = ctypes.c_short()           
            response = self.library.MCL_DLLVersion(ctypes.byref(ver),ctypes.byref(rev))
            if self.check_success(response):
                version = str(ver.value)+'.'+str(rev.value)
        return version

    def get_firmware_version(self):
        version = None
        if self.loaded and self.connected:
            # MCL_GetFirmwareVersion(short *version, short *profile, int handle);            
            ver = ctypes.c_short(); rev = ctypes.c_short()
            response = self.library.MCL_GetFirmwareVersion(ctypes.byref(ver),ctypes.byref(rev),self.handle)
            if self.check_success(response):
                   version = str(ver.value)+'.'+str(rev.value)
        return version

    def get_product_id(self):
        product = None
        if self.loaded and self.connected:
            # MCL_GetProductID(unsigned short *PID, int handle);
            pid = ctypes.c_ushort(); 
            response = self.library.MCL_GetProductID(ctypes.byref(pid),self.handle)
            if self.check_success(response):
                   product = pid.value
        return product

    def check_driver(self):
        valid = False
        if self.loaded:                
            # MCL_CorrectDriverVersion();
            valid = bool(self.library.MCL_CorrectDriverVersion())
        return valid

    def get_serial_number(self):
        serial = None
        if self.loaded and self.connected:
            # MCL_GetSerialNumber(int handle);        
            serial = self.library.MCL_GetSerialNumber(self.handle)
        return serial

    def get_travel_info(self):
        travel = dict()
        travel['resolution'] = None; # encoder resolution
        travel['stepSizeMin'] = None;
        travel['velocityMax'] = None;  # max velocity when updating 1 axis
        travel['velocityMax2'] = None; # max velocity when updating 2 axes
        travel['velocityMax3'] = None; # max velocity when updating 3 axes
        travel['velocityMin'] = None;
        if self.loaded and self.connected:
            # MCL_MicroDriveInformation(
                # double* encoderResolution,
                # double* stepSize,
                # double* maxVelocity,
                # double* maxVelocityTwoAxis,
                # double* maxVelocityThreeAxis,
                # double* minVelocity,
                # int handle);            
            res = ctypes.c_double()
            step = ctypes.c_double()
            vmax = ctypes.c_double()
            vmax2 = ctypes.c_double()
            vmax3 = ctypes.c_double()
            vmin = ctypes.c_double()
            response = self.library.MCL_MicroDriveInformation(ctypes.byref(res),ctypes.byref(step),ctypes.byref(vmax),ctypes.byref(vmax2),ctypes.byref(vmax3),ctypes.byref(vmin),self.handle)
            if self.check_success(response):
                travel['resolution'] = res.value*1e-6; 
                travel['stepSizeMin'] = step.value*1e-3;
                travel['velocityMax'] = vmax.value*1e-3;
                travel['velocityMax2'] = vmax2.value*1e-3;
                travel['velocityMax3'] = vmax3.value*1e-3;
                travel['velocityMin'] = vmin.value*1e-3;
        return travel
        
    def get_device_info(self):
        info = dict()
        info['libraryVersion'] = self.get_library_version()
        info['firmwareVersion'] = self.get_firmware_version()        
        info['serialNumber'] = self.get_serial_number()
        info['productID'] = self.get_product_id()
        info['driverValid'] = self.check_driver()  
        info['travel'] = self.get_travel_info()
        return info

    def load_device_info(self):
        self.info = self.get_device_info()

    def get_bit(self,value,index):
        return (value&(1<<index)) != 0

    def check_movement(self,value):
        # break response up into bits
        bits = []
        for i in range(8):
            bit = self.get_bit(value,i)
            bits.append(bit)
        #bits.reverse() # flip order of bits
        bits = [not(bit) for bit in bits] # invert bits
        #print(bits)

        # read out flags
        status = dict();
        status['success'] = bits[7-1];  # success flag
        status['forward'] = [bits[i-1] for i in [2,4,6]] # flags for forward limit reached 
        status['reverse'] = [bits[i-1] for i in [1,3,5]] # flags for reverse limit reached 
        return status  

    def get_move_status(self):
        moving = False
        if self.loaded and self.connected:
            # MCL_MicroDriveMoveStatus(int *isMoving, int handle);
            status = ctypes.c_int(); 
            response = self.library.MCL_MicroDriveMoveStatus(ctypes.byref(status),self.handle)
            if self.check_success(response):
                moving = bool(status.value)
        return moving      

    def is_moving(self):
        return self.get_move_status()

    def get_limit_status(self):
        flags = dict()
        if self.loaded and self.connected:
            # MCL_MicroDriveStatus(unsigned char *status, int handle);
            status = ctypes.c_ubyte(); 
            response = self.library.MCL_MicroDriveStatus(ctypes.byref(status),self.handle)
            if self.check_success(response):
                flags = self.check_movement(status.value)                           
        return flags

    def is_at_limit(self):
        flags = self.get_limit_status()
        return any(flags['forward']) or any(flags['reverse'])
    
    def check_limits(self):        
        if self.is_at_limit():
            flags = self.get_limit_status()
            for n in range(self.axes['count']):
                ax = self.axes['name'][n]
                if flags['forward'][n]:
                    self.notify(ax.upper()+'-Axis Forward Limiter Reached.')
                if flags['reverse'][n]:
                    self.notify(ax.upper()+'-Axis Reverse Limiter Reached.')            

    def wait(self):
        # waits long enough for the previously commanded move to finish.
        ready = False        
        if self.loaded and self.connected:
            # MCL_MicroDriveWait(int handle);
            response = self.library.MCL_MicroDriveWait(self.handle)
            ready = self.check_success(response)
        return ready

    def stop(self):
        # stops the stage from moving
        flags = dict()        
        if self.loaded and self.connected:
            # MCL_MicroDriveStop(unsigned char* status, int handle);
            status = ctypes.c_ubyte(); 
            response = self.library.MCL_MicroDriveStop(ctypes.byref(status),self.handle)
            if self.check_success(response):
                flags = self.check_movement(status.value)                           
        return flags

    def read_step(self):
        step = dict()
        if self.loaded and self.connected:
            # MCL_MicroDriveCurrentMicroStepPosition(int *microStepsX, int *microStepsY, int *microStepsZ, int handle);
            x = ctypes.c_int32(); 
            y = ctypes.c_int32(); 
            z = ctypes.c_int32(); 
            response = self.library.MCL_MicroDriveCurrentMicroStepPosition(ctypes.byref(x),ctypes.byref(y),ctypes.byref(z),self.handle)
            if self.check_success(response):         
                step = {'x':x.value,'y':y.value,'z':z.value}
                self.step = step
        return step          

    def read_position(self):
        position = dict()
        if self.loaded and self.connected:
            # MCL_MicroDriveReadEncoders(double* x, double* y, double *z, int handle);
            x = ctypes.c_double(); 
            y = ctypes.c_double(); 
            z = ctypes.c_double(); 
            response = self.library.MCL_MicroDriveReadEncoders(ctypes.byref(x),ctypes.byref(y),ctypes.byref(z),self.handle)
            if self.check_success(response):
                position = {'x':x.value*1e-3,'y':y.value*1e-3,'z':z.value*1e-3}
                self.position = position
        return position

    def read(self):
        # read both step and encoder position
        self.wait()
        success1 = bool(self.read_step())
        success2 = bool(self.read_position())
        return success1 and success2
         
    def get_position_change(self,new_position,old_position):
        change = dict()
        for key in old_position.keys():
            change[key] = new_position[key] - old_position[key]
        return change

    def set_position_change(self,change,old_position):
        new_position = dict()
        for key in old_position.keys():
            new_position[key] = old_position[key] + change[key]
        return new_position

    def format_position(self,values):
        position = dict()
        if len(values) >= self.axes['count']:
            for n in range(self.axes['count']):
                position[self.axes['name'][n]] = values[n]
        return position

    def unformat_position(self,position):
        values = []
        if position:
            for key in position.keys():
                values.append(position[key])
        return values

    def review_position_change(self,new_position,old_position,sent_change):
        measured_change = self.get_position_change(new_position,old_position)        
        mismatch = self.get_position_change(sent_change,measured_change)
        #error = any(self.unformat_position(mismatch))
        return {'position':new_position,'command':sent_change,'change':measured_change,'mismatch':mismatch}

    def move_by_step(self,move,velocity=None):
        # move along each axis by the step amount specified in "move" (can be positive or negative) (move in [#], velocity[m/s])
        position_old = self.step.copy()
        position_new = self.step.copy()
        polarity = self.travel['direction']
        if not move or ( len(move)<self.axes['count'] ) :
            move = [0.0]*self.axes['count']
        move = [move[i]*polarity[i] for i in range(self.axes['count'])]            
        if not velocity or ( len(velocity)<self.axes['count'] ) :
            velocity = self.travel['velocity']      
        if self.loaded and self.connected:
            # MCL_MicroDriveMoveProfileXYZ_MicroSteps(
            #    double velocityX,
            #    int microStepsX,
            #    double velocityY,
            #    int microStepsY,
            #    double velocityZ,
            #    int microStepsZ,
            #    int handle    ); 
            self.wait()         
            mov = self.format_position([ctypes.c_int(round(m)) for m in move])
            vel = self.format_position([ctypes.c_double(v*1e3) for v in velocity])            
            response = self.library.MCL_MicroDriveMoveProfileXYZ_MicroSteps(vel['x'],mov['x'],vel['y'],mov['y'],vel['z'],mov['z'],self.handle)
            if self.check_success(response):
                self.read()
                position_new = self.step.copy()
        return self.review_position_change(position_new,position_old,self.format_position(move))

    def move_by_distance(self,move,velocity=None,rounding=None):
        # move along each axis by the distance specified in "move" (can be positive or negative) (move in [m], velocity[m/s])
        position_old = self.position.copy()
        position_new = self.position.copy()
        polarity = self.travel['direction']
        if not move or ( len(move)<self.axes['count'] ) :
            move = [0.0]*self.axes['count']
        move = [move[i]*polarity[i] for i in range(self.axes['count'])]            
        if not velocity or ( len(velocity)<self.axes['count'] ) :
            velocity = self.travel['velocity'] 
        if not rounding or ( len(rounding)<self.axes['count'] ) :
            rounding = self.travel['rounding']                       
        if self.loaded and self.connected:
            # MCL_MicroDriveMoveProfileXYZ(
            #     double velocityX,
            #     double distanceX,
            #     int roundingX, 
            #     double velocityY,
            #     double distanceY,
            #     int roundingY, 
            #     double velocityZ,
            #     double distanceZ,
            #     int roundingZ,
            #     int handle     );
            self.wait()           
            mov = self.format_position([ctypes.c_double(m*1e3) for m in move])
            vel = self.format_position([ctypes.c_double(v*1e3) for v in velocity])
            rou = self.format_position([ctypes.c_int(r) for r in rounding])                        
            response = self.library.MCL_MicroDriveMoveProfileXYZ(vel['x'],mov['x'],rou['x'],vel['y'],mov['y'],rou['y'],vel['z'],mov['z'],rou['z'],self.handle)
            if self.check_success(response):
                self.read()
                position_new = self.position.copy()
        return self.review_position_change(position_new,position_old,self.format_position(move))

    def reset(self,axis='all'):
        # reset all encoders (makes the current position of the microstage the zero position of the encoder)
        flags = dict()
        if self.loaded and self.connected:
            status = ctypes.c_ubyte(); 
            if axis.lower()=='x':
                # MCL_MicroDriveResetXEncoder(unsigned char* status, int handle);
                response = self.library.MCL_MicroDriveResetXEncoder(ctypes.byref(status),self.handle)                
            elif axis.lower()=='y':  
                # MCL_MicroDriveResetYEncoder(unsigned char* status, int handle);          
                response = self.library.MCL_MicroDriveResetYEncoder(ctypes.byref(status),self.handle)
            elif axis.lower()=='z':
                # MCL_MicroDriveResetZEncoder(unsigned char* status, int handle);
                response = self.library.MCL_MicroDriveResetZEncoder(ctypes.byref(status),self.handle)                
            else: #axis.lower()=='all'
                # MCL_MicroDriveStop(unsigned char* status, int handle);
                response = self.library.MCL_MicroDriveResetEncoders(ctypes.byref(status),self.handle)
            if self.check_success(response):
                flags = self.check_movement(status.value)                           
        return flags

    def run_to_axis_limits(self,direction='forward',axis='x',stepsize=10**3,velocity=None):
        # find forward or reverse axis limit
        axisInd = self.axes['name'].index(axis.lower())
        move = [0]*self.axes['count']
        move[axisInd] = stepsize
        direction = direction.lower()
        if direction=='reverse':
            move[axisInd] = -move[axisInd]
        if not velocity or ( len(velocity)<self.axes['count'] ) :
            velocity = self.travel['velocityFast'] 
        railed = False
        while not railed:
            stage.move_by_step(move,velocity=velocity)
            railed = stage.get_limit_status()[direction][axisInd]
        return {'position':self.read_position(),'step':self.read_step()}
    
    def get_axis_limits(self,axis='x',stepsize=10**3,velocity=None):
        # get the position extremes for an axis
        return {direction:self.run_to_axis_limits(direction=direction,axis=axis,stepsize=stepsize,velocity=velocity)['position'][axis] 
                for direction in ['forward','reverse']}
    
    def get_limits(self,stepsize=10**3,velocity=None):
        # get the position extremes for all axes
        return {ax:self.get_axis_limits(axis=ax,stepsize=stepsize,velocity=velocity) 
                for ax in self.axes['name']}

    def center_axes(self,stepsize=10**3,velocity=None):
        limits = self.get_limits(stepsize=stepsize,velocity=velocity)
        reverses = [limits[ax]['reverse'] for ax in stage.axes['name']]        
        forwards = [limits[ax]['forward'] for ax in stage.axes['name']]
        centers = [(reverses[i]+forwards[i])/2 for i in range(self.axes['count'])]
        moves = [centers[i]-reverses[i] for i in range(self.axes['count'])]
        self.move_by_distance(moves,velocity=velocity)
        self.reset()
        return {'position':self.read_position(),'step':self.read_step(),'limits':limits,'centers':centers,'moves':moves}

if __name__ == "__main__":

    stage = MicroDrive()    
    stage.notify('Device Info: '+str(stage.info))
    
    if stage.is_moving(): # is stage is moving
        stage.stop() # stop the stage from moving
        stage.wait() # wait for stage to stop moving
        # wait_time = 0.01 # time to wait [sec]
        # time.sleep(wait_time)         
    stage.check_limits() # check if at limitss
    stage.read() # read current position
    #stage.center_axes() # center all axis 
    
    # print(stage.move_by_step([1,2,1])) # move by stage steps
    # print(stage.move_by_distance([1e-6,0.0e-6,0.1e-6])) # move in meters
    # print(stage.position)
    # print(stage.step)



# -*- coding: utf-8 -*-

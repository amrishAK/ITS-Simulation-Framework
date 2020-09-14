from collections import deque
import numpy as np
import math
import carla
from misc import get_speed
from Container import Container

class PID(object):
    
    errorBuffer = deque(maxlen=20)

    def UpdateBuffer(self,error):
        
        if self.errorBuffer.count == self.errorBuffer.maxlen:
            self.errorBuffer.popleft()

        self.errorBuffer.append(error)

    def Calculate(self,error,K_P=1.0, K_D=0.0, K_I=0.0, dt=0.03):

        self.UpdateBuffer(error)

        if len(self.errorBuffer) >= 2:
            derivativeError = (self.errorBuffer[-1] - self.errorBuffer[-2]) / dt
            integralError = sum(self.errorBuffer) * dt
        else:
            derivativeError = 0.0
            integralError = 0.0
        
        return np.clip((K_P * error) + (K_D * 0.0) + (K_I * 0.0), -1.0, 1.0)

class PIDHandler (object):
    
    FPS = 20

    def __init__(self):
        """
        Controller initialization.

        dt -- time difference between physics control in seconds.
        This is can be fixed from server side
        using the arguments -benchmark -fps=F, since dt = 1/F

        target_speed -- desired cruise speed in km/h

        min_distance -- minimum distance to remove waypoint from queue

        lateral_dict -- dictionary of arguments to setup the lateral PID controller
                            {'K_P':, 'K_D':, 'K_I':, 'dt'}

        longitudinal_dict -- dictionary of arguments to setup the longitudinal PID controller
                            {'K_P':, 'K_D':, 'K_I':, 'dt'}
        """
        # PID parameters
        self.args_lat_hw_dict = {
            'K_P': 0.75,
            'K_D': 0.02,
            'K_I': 0.4,
            'dt': 1.0 / self.FPS}
        self.args_lat_city_dict = {
            'K_P': 0.58,
            'K_D': 0.02,
            'K_I': 0.5,
            'dt': 1.0 / self.FPS}
        self.args_long_hw_dict = {
            'K_P': 0.37,
            'K_D': 0.024,
            'K_I': 0.032,
            'dt': 1.0 / self.FPS}
        self.args_long_city_dict = {
            'K_P': 0.15,
            'K_D': 0.05,
            'K_I': 0.07,
            'dt': 1.0 / self.FPS}

        #controllers
        self.lateralControl = PID() #PID for trajectory (steer)
        self.longitudinalControl = PID() #PID for throttle (speed)
        
        self.max_brake = None
        self.max_throt = None
        self.max_steer = None
        self.min_steer = None
        self.past_steering = None

    def SetupController(self):
        self.max_brake = 0.3
        self.max_throt = 0.75
        self.max_steer = 0.8
        self.min_steer = self.max_steer * -1.0
        self.past_steering = Container().GetActor().get_control().steer
        print(f"controller setup {self.past_steering}")

    def RunStep(self,vehicle,targetWaypoint, targetSpeed, debug=False):
        """
        Execute one step of local planning which involves
        running the longitudinal and lateral PID controllers to
        follow the waypoints trajectory.

            :param target_speed: desired speed
            :param debug: boolean flag to activate waypoints debugging
            :return: control
        """
        latParam, longParam = self.GetParameters(targetSpeed)
        
        speedError = self.CalculateSpeedError(targetSpeed,get_speed(vehicle))
        steerError = self.CalculateSteerError(targetWaypoint,vehicle.get_transform())

        acceleration = self.longitudinalControl.Calculate(speedError,**longParam)
        current_steering = self.lateralControl.Calculate(steerError,**latParam)
        
        control = carla.VehicleControl()
        
        if acceleration >= 0.0:
            control.throttle = min(acceleration, self.max_throt)
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = min(abs(acceleration), self.max_brake)

        # Steering regulation: changes cannot happen abruptly, can't steer too much.


        if current_steering > self.past_steering + 0.1:
            current_steering = self.past_steering + 0.1
        elif current_steering < self.past_steering - 0.1:
            current_steering = self.past_steering - 0.1

        if current_steering >= 0:
            steering = min(self.max_steer, current_steering)
        else:
            steering = max(self.min_steer, current_steering)

        control.steer = steering
        control.hand_brake = False
        control.manual_gear_shift = False
        self.past_steering = steering
        
        # print(f"Steer {control.steer} |target speed {targetSpeed} | accleration { control.throttle}")

        return control

    def GetParameters(self,targetSpeed):
        '''
        return: lateral parameter (steer), longitude parameter(throttle)
        '''
        return (self.args_lat_hw_dict, self.args_long_hw_dict) if targetSpeed > 50 else (self.args_lat_city_dict, self.args_long_city_dict)

    def CalculateSteerError(self,targetWaypoint,vehicleTransform):
        '''
        Calculates the CTE (steer value error)
            param: targetWaypoin
            return: error value (CTE)
        '''

        v_begin = vehicleTransform.location
        v_end = v_begin + carla.Location(x=math.cos(math.radians(vehicleTransform.rotation.yaw)),
                                         y=math.sin(math.radians(vehicleTransform.rotation.yaw)))

        v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0])
        w_vec = np.array([targetWaypoint.transform.location.x -
                          v_begin.x, targetWaypoint.transform.location.y -
                          v_begin.y, 0.0])
        _dot = 0
        try:
            _dot = math.acos(np.clip(np.dot(w_vec, v_vec) /
                                 (np.linalg.norm(w_vec) * np.linalg.norm(v_vec)), -1.0, 1.0))
        except Exception as ex:
            print(ex)
            print((np.linalg.norm(w_vec) * np.linalg.norm(v_vec)))
    

    

        _cross = np.cross(v_vec, w_vec)

        if _cross[2] < 0: 
            _dot *= -1.0

        return _dot

    def CalculateSpeedError(self,targetSpeed,vehicleSpeed):
        return targetSpeed - vehicleSpeed
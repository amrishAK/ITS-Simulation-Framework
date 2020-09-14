import carla
from abc import ABC, abstractmethod
from RoadOption import RoadOption
from StateHandler import VehicleStateHandler, RSUStateHandler
from Container import Container
from RouteHandler import RouteHandler
from misc import get_speed, positive, compute_distance, is_within_distance
from types_behavior import Cautious, Aggressive, Normal
from CarlaVizUtil import CarlaPainter
from FrameworkException import StopException
import numpy as np

class BaseVehicleAgent (ABC):
    '''
    Abstract method: RunStep(self): return tragetSpeed
    '''

    def __init__(self,container:Container):
        pass

    def SetupAgent(self):
        self.vechileState = Container().GetState()
        self.RouteManager = Container().GetRouteManager()
        
        #behaviour
        self.Behavior = Normal()

    def AgentLaneChange(self, location, waypoint, vehicle_list):
        """
        This method is in charge of overtaking behaviors.

            :param location: current location of the agent
            :param waypoint: current waypoint of the agent
            :param vehicle_list: list of all the nearby vehicles
        """

        status = False

        left_turn = waypoint.left_lane_marking.lane_change
        right_turn = waypoint.right_lane_marking.lane_change

        left_wpt = waypoint.get_left_lane()
        right_wpt = waypoint.get_right_lane()

        if (left_turn == carla.LaneChange.Left or left_turn ==
                carla.LaneChange.Both) and waypoint.lane_id * left_wpt.lane_id > 0 and left_wpt.lane_type == carla.LaneType.Driving:
            new_vehicle_state, _, _ = self.IsVehicleHazard(waypoint, location, vehicle_list, max(
                self.Behavior.min_proximity_threshold, self.vechileState.speedLimit / 3), up_angle_th=180, lane_offset=-1)
            if not new_vehicle_state:
                print("Overtaking to the left!")
                status = True
                self.Behavior.overtake_counter = 200
                self.RouteManager.CreateRoute(left_wpt.transform.location)
        elif right_turn == carla.LaneChange.Right and waypoint.lane_id * right_wpt.lane_id > 0 and right_wpt.lane_type == carla.LaneType.Driving:
            new_vehicle_state, _, _ = self.IsVehicleHazard(waypoint, location, vehicle_list, max(
                self.Behavior.min_proximity_threshold, self.vechileState.speedLimit / 3), up_angle_th=180, lane_offset=1)
            if not new_vehicle_state:
                print("Overtaking to the right!")
                status = True
                self.Behavior.overtake_counter = 200
                self.RouteManager.CreateRoute(right_wpt.transform.location)
        
        return status

    def Tailgating(self, location, waypoint, vehicle_list):
        """
        This method is in charge of tailgating behaviors.

            :param location: current location of the agent
            :param waypoint: current waypoint of the agent
            :param vehicle_list: list of all the nearby vehicles
        """

        left_turn = waypoint.left_lane_marking.lane_change
        right_turn = waypoint.right_lane_marking.lane_change

        left_wpt = waypoint.get_left_lane()
        right_wpt = waypoint.get_right_lane()

        behind_vehicle_state, behind_vehicle, _ = self.IsVehicleHazard(waypoint, location, vehicle_list, max(
            self.Behavior.min_proximity_threshold, self.vechileState.speedLimit / 2), up_angle_th=180, low_angle_th=160)
        if behind_vehicle_state and self.vechileState.speed < get_speed(behind_vehicle):
            if (right_turn == carla.LaneChange.Right or right_turn ==
                    carla.LaneChange.Both) and waypoint.lane_id * right_wpt.lane_id > 0 and right_wpt.lane_type == carla.LaneType.Driving:
                new_vehicle_state, _, _ = self.IsVehicleHazard(waypoint, location, vehicle_list, max(
                    self.Behavior.min_proximity_threshold, self.vechileState.speedLimit / 2), up_angle_th=180, lane_offset=1)
                if not new_vehicle_state:
                    print("Tailgating, moving to the right!")
                    self.Behavior.tailgate_counter = 200
                    self.RouteManager.CreateRoute(right_wpt.transform.location)
            elif left_turn == carla.LaneChange.Left and waypoint.lane_id * left_wpt.lane_id > 0 and left_wpt.lane_type == carla.LaneType.Driving:
                new_vehicle_state, _, _ = self.IsVehicleHazard(waypoint, location, vehicle_list, max(
                    self.Behavior.min_proximity_threshold, self.vechileState.speedLimit / 2), up_angle_th=180, lane_offset=-1)
                if not new_vehicle_state:
                    print("Tailgating, moving to the left!")
                    self.Behavior.tailgate_counter = 200
                    self.RouteManager.CreateRoute(left_wpt.transform.location)

    def CollisionAvoidanceManager(self, location, waypoint):
        """
        This module is in charge of warning in case of a collision
        and managing possible overtaking or tailgating chances.

            :param location: current location of the agent
            :param waypoint: current waypoint of the agent
            :return vehicle_state: True if there is a vehicle nearby, False if not
            :return vehicle: nearby vehicle
            :return distance: distance to nearby vehicle
        """

        vehicle_list = Container().GetWorld().get_actors().filter("*vehicle*")
        def dist(v): return v.get_location().distance(waypoint.transform.location)
        vehicle_list = [v for v in vehicle_list if dist(v) < 45 and v.id != self.vechileState.vehicleId]

        if self.RouteManager.direction == RoadOption.CHANGELANELEFT:
            vehicle_state, vehicle, distance = self.IsVehicleHazard(
                waypoint, location, vehicle_list, max(
                    self.Behavior.min_proximity_threshold, self.vechileState.speedLimit / 2), up_angle_th=180, lane_offset=-1)
        elif self.RouteManager.direction == RoadOption.CHANGELANERIGHT:
            vehicle_state, vehicle, distance = self.IsVehicleHazard(
                waypoint, location, vehicle_list, max(
                    self.Behavior.min_proximity_threshold, self.vechileState.speedLimit / 2), up_angle_th=180, lane_offset=1)
        else:
            vehicle_state, vehicle, distance = self.IsVehicleHazard(
                waypoint, location, vehicle_list, max(
                    self.Behavior.min_proximity_threshold, self.vechileState.speedLimit / 3), up_angle_th=30)

            # Check for overtaking

            if vehicle_state and self.RouteManager.direction == RoadOption.LANEFOLLOW and \
                    not waypoint.is_junction and self.vechileState.speed > 10 \
                    and self.Behavior.overtake_counter == 0 and self.vechileState.speed > get_speed(vehicle):
                self.AgentLaneChange(location, waypoint, vehicle_list)

            # Check for tailgating

            elif not vehicle_state and self.RouteManager.direction == RoadOption.LANEFOLLOW \
                    and not waypoint.is_junction and self.vechileState.speed > 10 \
                    and self.Behavior.tailgate_counter == 0:
                self.Tailgating(location, waypoint, vehicle_list)

        return vehicle_state, vehicle, distance

    def CarFollowingManager(self, vehicle, distance):
        """
        Module in charge of car-following behaviors when there's
        someone in front of us.

            :param vehicle: car to follow
            :param distance: distance from vehicle
            :param debug: boolean for debugging
            :return targetSpeed: target speed
        """

        vehicleSpeed = get_speed(vehicle)
        delta_v = max(1, (self.vechileState.speed - vehicleSpeed) / 3.6)
        ttc = distance / delta_v if delta_v != 0 else distance / np.nextafter(0., 1.)

        # Under safety time distance, slow down.
        if self.Behavior.safety_time > ttc > 0.0:
            targetSpeed = min(positive(vehicleSpeed - self.Behavior.speed_decrease), min(self.Behavior.max_speed, self.vechileState.speedLimit - self.Behavior.speed_lim_dist))
        # Actual safety distance area, try to follow the speed of the vehicle in front.
        elif 2 * self.Behavior.safety_time > ttc >= self.Behavior.safety_time:
            targetSpeed = min(max(self.vechileState.minSpeed, vehicleSpeed),min(self.Behavior.max_speed, self.vechileState.speedLimit - self.Behavior.speed_lim_dist))
        # Normal behavior..
        else:
            targetSpeed= min(self.Behavior.max_speed, self.vechileState.speedLimit - self.Behavior.speed_lim_dist)

        return targetSpeed

    def IsVehicleHazard(self, ego_wpt, ego_loc, vehicle_list, proximity_th, up_angle_th, low_angle_th=0, lane_offset=0):
        """
        Check if a given vehicle is an obstacle in our way. To this end we take
        into account the road and lane the target vehicle is on and run a
        geometry test to check if the target vehicle is under a certain distance
        in front of our ego vehicle. We also check the next waypoint, just to be
        sure there's not a sudden road id change.

        WARNING: This method is an approximation that could fail for very large
        vehicles, which center is actually on a different lane but their
        extension falls within the ego vehicle lane. Also, make sure to remove
        the ego vehicle from the list. Lane offset is set to +1 for right lanes
        and -1 for left lanes, but this has to be inverted if lane values are
        negative.

            :param ego_wpt: waypoint of ego-vehicle
            :param ego_log: location of ego-vehicle
            :param vehicle_list: list of potential obstacle to check
            :param proximity_th: threshold for the agent to be alerted of
            a possible collision
            :param up_angle_th: upper threshold for angle
            :param low_angle_th: lower threshold for angle
            :param lane_offset: for right and left lane changes
            :return: a tuple given by (bool_flag, vehicle, distance), where:
            - bool_flag is True if there is a vehicle ahead blocking us
                   and False otherwise
            - vehicle is the blocker object itself
            - distance is the meters separating the two vehicles
        """

        # Get the right offset
        if ego_wpt.lane_id < 0 and lane_offset != 0:
            lane_offset *= -1

        for target_vehicle in vehicle_list:

            target_vehicle_loc = target_vehicle.get_location()
            # If the object is not in our next or current lane it's not an obstacle

            target_wpt = self.RouteManager.map.get_waypoint(target_vehicle_loc)
            
            if target_wpt.road_id != ego_wpt.road_id or \
                    target_wpt.lane_id != ego_wpt.lane_id + lane_offset:
                next_wpt = self.RouteManager.get_incoming_waypoint_and_direction(steps=5)[0]
                if target_wpt.road_id != next_wpt.road_id or \
                        target_wpt.lane_id != next_wpt.lane_id + lane_offset:
                    continue

            if is_within_distance(target_vehicle_loc, ego_loc,
                                  Container().GetActor().get_transform().rotation.yaw,
                                  proximity_th, up_angle_th, low_angle_th):

                return (True, target_vehicle, compute_distance(target_vehicle_loc, ego_loc))

        return (False, None, -1)

    def BasicMobiltyModel(self):
        targetSpeed  = None
        behaviour = None
    
        if self.Behavior.tailgate_counter > 0:
            self.Behavior.tailgate_counter -= 1
        if self.Behavior.overtake_counter > 0:
            self.Behavior.overtake_counter -= 1

        egoVehicle = Container().GetActor()
        ego_vehicle_loc = egoVehicle.get_location()
        ego_vehicle_wp = Container().GetMap().get_waypoint(ego_vehicle_loc)


        # 2.2: Car following behaviors
        vehicle_state, vehicle, distance = self.CollisionAvoidanceManager(
            ego_vehicle_loc, ego_vehicle_wp)

        if vehicle_state:
            # Distance is computed from the center of the two cars,
            # we use bounding boxes to calculate the actual distance
            distance = distance - max(
                vehicle.bounding_box.extent.y, vehicle.bounding_box.extent.x) - max(
                    egoVehicle.bounding_box.extent.y, egoVehicle.bounding_box.extent.x)

            # Emergency brake if the car is very close.
            if distance < self.Behavior.braking_distance:
                raise StopException
            else:
                targetSpeed = self.CarFollowingManager(vehicle, distance)
            
            behaviour = "follow"

        # 4: Intersection behavior

        # Checking if there's a junction nearby to slow down
        elif self.RouteManager.incomingWaypoint.is_junction and (self.RouteManager.incomingDirection == RoadOption.LEFT or self.RouteManager.incomingDirection == RoadOption.RIGHT):
            
            behaviour = "intersection"
            targetSpeed = min(self.Behavior.max_speed, self.vechileState.speedLimit - 5)

        # 5: Normal behavior

        # Calculate controller based on no turn, traffic light or vehicle in front
        else:
            behaviour = "normal"
            targetSpeed= max(self.Behavior.max_speed, self.vechileState.speedLimit - self.Behavior.speed_lim_dist)
    
        return targetSpeed, behaviour

    @abstractmethod
    def RunStep(self):
        '''
        The vehicle behaviour is controled here
            :return: targetSpeed
        '''
        raise NotImplementedError


class BaseRsuAgent(ABC):
  
    def __init__(self,container:Container):
        pass

    @abstractmethod
    def Setup(self):
        raise NotImplementedError
    
    @abstractmethod
    def RunStep(self):
        raise NotImplementedError



class BasicVehicleAgent(BaseVehicleAgent):

    # painter = CarlaPainter('localhost',8089)

    
    def RunStep(self):
        targetSpeed  = None
    
        if self.Behavior.tailgate_counter > 0:
            self.Behavior.tailgate_counter -= 1
        if self.Behavior.overtake_counter > 0:
            self.Behavior.overtake_counter -= 1

        egoVehicle = Container().GetActor()
        ego_vehicle_loc = egoVehicle.get_location()
        ego_vehicle_wp = Container().GetMap().get_waypoint(ego_vehicle_loc)

        # 1: Red lights and stops behavior

        # if self.traffic_light_manager(ego_vehicle_wp) != 0:
        #     return self.emergency_stop()

        # 2.2: Car following behaviors
        vehicle_state, vehicle, distance = self.CollisionAvoidanceManager(
            ego_vehicle_loc, ego_vehicle_wp)

        if vehicle_state:
            # Distance is computed from the center of the two cars,
            # we use bounding boxes to calculate the actual distance
            distance = distance - max(
                vehicle.bounding_box.extent.y, vehicle.bounding_box.extent.x) - max(
                    egoVehicle.bounding_box.extent.y, egoVehicle.bounding_box.extent.x)

            # Emergency brake if the car is very close.
            if distance < self.Behavior.braking_distance:
                raise StopException
            else:
                targetSpeed = self.CarFollowingManager(vehicle, distance)

        # 4: Intersection behavior

        # Checking if there's a junction nearby to slow down
        elif self.RouteManager.incomingWaypoint.is_junction and (self.RouteManager.incomingDirection == RoadOption.LEFT or self.RouteManager.incomingDirection == RoadOption.RIGHT):
            targetSpeed = min(self.Behavior.max_speed, self.vechileState.speedLimit - 5)

        # 5: Normal behavior

        # Calculate controller based on no turn, traffic light or vehicle in front
        else:
                
            targetSpeed= max(self.Behavior.max_speed, self.vechileState.speedLimit - self.Behavior.speed_lim_dist)

        return targetSpeed
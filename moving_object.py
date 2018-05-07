# -*- coding: utf-8 -*-

import numpy as np
from net import Controller
from director import vtkAll as vtk
from director.debugVis import DebugData
from director import ioUtils, filterUtils


class MovingObject(object):

    """Moving object."""

    def __init__(self, velocity, polydata):
        """Constructs a MovingObject.

        Args:
            velocity: Velocity.
            polydata: Polydata.
        """
        self._state = np.array([0., 0., 0.])
        self._velocity = float(velocity)
        self._raw_polydata = polydata
        self._polydata = polydata
        self._sensors = []

    @property
    def x(self):
        """X coordinate."""
        return self._state[0]

    @x.setter
    def x(self, value):
        """X coordinate."""
        next_state = self._state.copy()
        next_state[0] = float(value)
        self._update_state(next_state)

    @property
    def y(self):
        """Y coordinate."""
        return self._state[1]

    @y.setter
    def y(self, value):
        """Y coordinate."""
        next_state = self._state.copy()
        next_state[1] = float(value)
        self._update_state(next_state)
        
        
        

    @property
    def theta(self):
        """Yaw in radians."""
        return self._state[2]

    @theta.setter
    def theta(self, value):
        """Yaw in radians."""
        next_state = self._state.copy()
        next_state[2] = float(value) % (2 * np.pi)
        self._update_state(next_state)

    @property
    def velocity(self):
        """Velocity."""
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        """Velocity."""
        self._velocity = float(value)

    @property
    def sensors(self):
        """List of attached sensors."""
        return self._sensors

    def attach_sensor(self, sensor):
        """Attaches a sensor.

        Args:
            sensor: Sensor.
        """
        self._sensors = []
        self._sensors.append(sensor)

    def _dynamics(self, state, t, controller=None):
        """Dynamics of the object.

        Args:
            state: Initial condition.
            t: Time.

        Returns:
            Derivative of state at t.
        """
        dqdt = np.zeros_like(state)
        dqdt[0] = self._velocity * np.cos(state[2])
        dqdt[1] = self._velocity * np.sin(state[2])
        dqdt[2] = self._control(state, t)
        return dqdt * t

    def _control(self, state, t):
        """Returns the yaw given state.

        Args:
            state: State.
            t: Time.

        Returns:
            Yaw.
        """
        raise NotImplementedError

    def _simulate(self, dt):
        """Simulates the object moving.

        Args:
            dt: Time length of step.

        Returns:
            New state.
        """
        return self._state + self._dynamics(self._state, dt)

    def move(self, dt=1.0/30.0):
        """Moves the object by a given time step.

        Args:
            dt: Length of time step.
        """
        state = self._simulate(dt)
        self._update_state(state)

    def _update_state(self, next_state):
        """Updates the moving object's state.

        Args:
            next_state: New state.
        """
        t = vtk.vtkTransform()
        t.Translate([next_state[0], next_state[1], 0.])
        t.RotateZ(np.degrees(next_state[2]))
        self._polydata = filterUtils.transformPolyData(self._raw_polydata, t)
        self._state = next_state
        list(map(lambda s: s.update(*self._state), self._sensors))

    def to_positioned_polydata(self):
        """Converts object to visualizable poly data.

        Note: Transformations have been already applied to this.
        """
        return self._polydata

    def to_polydata(self):
        """Converts object to visualizable poly data.

        Note: This is centered at (0, 0, 0) and is not rotated.
        """
        return self._raw_polydata
class Commonship(MovingObject):
    def __init__(self, velocity=25.0, scale=5.0):
        """Constructs a Robot.

        Args:
            velocity: Velocity of the robot in the forward direction.
            scale: Scale of the model.
            exploration: Exploration rate.
            model: Object model to use.
        """
        
        data = DebugData()
        center = [0, 0, 1.0 / 2 - 0.5]
        dimensions=[scale,scale/2,1.0]
        data.addCube(dimensions, center)
        polydata = data.getPolyData()
        super(Commonship, self).__init__(velocity, polydata)
        
    def move(self, dt=1.0/30.0):
         super(Commonship, self).move(dt)
    def setScale(self,scale):
        data = DebugData()
        center = [0, 0, 1.0 / 2 - 0.5]
        dimensions=[scale,scale/2,1.0]
        data.addCube(dimensions, center)
        polydata = data.getPolyData()
        self._polydata = polydata
        self._raw_polydata = polydata
    def _control(self, state, t):
        actions = [-np.pi/2, 0., np.pi/2]    
        optimal_i=1
        optimal_a = actions[optimal_i]
        self._selected_i = optimal_i
        return optimal_a

class Robot(MovingObject):

    """Robot."""

    def __init__(self, velocity=40.0, scale=0.5, exploration=0.5,size=100,
                 model="A10.obj"):
        """Constructs a Robot.

        Args:
            velocity: Velocity of the robot in the forward direction.
            scale: Scale of the model.
            exploration: Exploration rate.
            model: Object model to use.
        """
        self._size = size
        self._target = (0, 0,0)
        self._initPos = (0,0)
        self._exploration = exploration
        self._changetheta = 0
        self._inlaser = True
       # t = vtk.vtkTransform()
       # t.Scale(scale, scale, scale)
       # polydata = ioUtils.readPolyData(model)
       # polydata = filterUtils.transformPolyData(polydata, t)
        
        data = DebugData()
        center = [0, 0, 1.0 / 2 - 0.5]
        dimensions=[10,5,1.0]
        data.addCube(dimensions, center)
        polydata = data.getPolyData()
        
        super(Robot, self).__init__(velocity, polydata)
        self._ctrl = Controller()

    def move(self, dt=1.0/30.0):
        """Moves the object by a given time step.

        Args:
            dt: Length of time step.
        """
        if self._sensors[0].is_laser() and self._is_dist():
     #       self._inlaser = True
            super(Robot, self).move(dt)
        else:
            gamma = 0.9
            prev_xy = self._state[0], self._state[1]
            prev_state = self._get_state()
            prev_utilities = self._ctrl.evaluate(prev_state)
            super(Robot, self).move(dt)
            next_state = self._get_state()
            next_utilities = self._ctrl.evaluate(next_state)
            print("action: {}, utility: {}".format(
                self._selected_i, prev_utilities[self._selected_i]))
        
            terminal = self._sensors[0].has_collided()
            curr_reward = self._get_reward(prev_xy)
            print("reward: {}, angle: {}".format(
                    curr_reward,self._changetheta/np.pi*180))
            print("i",self._selected_i)
            total_reward =\
                curr_reward if terminal else \
                curr_reward + gamma * next_utilities[self._selected_i]
            rewards = [total_reward if i == self._selected_i else prev_utilities[i]
                for i in range(len(next_utilities))]
            self._ctrl.train(prev_state, rewards)
    def init_angle(self):
        self._changetheta = 0
        
    def set_ship(self,ship):
        self._ship = ship
    def set_initPos(self,pos):
        self._initPos = pos
    def set_target(self, target):
        self._target = target

    def set_controller(self, ctrl):
        self._ctrl = ctrl

    def at_target(self, threshold=3):
        """Return whether the robot has reached its target.

        Args:
            threshold: Target distance threshold.

        Returns:
            True if target is reached.
        """
        return (abs(self._state[0] - self._target[0]) <= threshold and
                abs(self._state[1] - self._target[1]) <= threshold)

    def _get_reward(self, prev_state):
        prev_dx = self._target[0] - prev_state[0]
        prev_dy = self._target[1] - prev_state[1]
        prev_distance = np.sqrt(prev_dx ** 2 + prev_dy ** 2)
        new_dx = self._target[0] - self._state[0]
        new_dy = self._target[1] - self._state[1]
        new_distance = np.sqrt(new_dx ** 2 + new_dy ** 2)
     #   if self._inlaser and not self._sensors[0].is_laser():
     #       self._inlaser = False
     #   elif not self._inlaser and min(self._sensors[0].distances)>=0.92 and abs(self._changetheta)<=np.pi/180*5:
     #       print("xxx",self._road_dist())
     #       return 15.0/200.0*self._size
        if self._sensors[0].has_collided():
            return -20
        elif self.at_target():
            return 15
        elif self._state[0]>self._size/2+2:
            return 7 - abs(self._state[1]-self._target[1])/51.0*7.0
        else:
            delta_distance = prev_distance - new_distance
            angle_distance = -abs(self._angle_to_destination()) / 8
            obstacle_ahead = min(self._sensors[0].distances)-1.0
            return delta_distance/2 + angle_distance + obstacle_ahead
         
     #   if self._sensors[0].has_collided():
     #       return -20.0/200.0*self._size 
     #   else:
       #     if abs(self._changetheta)<=np.pi/180*5 and self._sensors[0].is_laser():
      #          return 15.0/200.0*self._size
     #       else:
      #          delta_distance = self._road_dist(prev_state)
      #          print("tttttt",delta_distance)
      #          angle_distance = -abs(self._angle_to_destination())
      #          return (np.pi/180*50-abs(self._changetheta))
    
    def _road_dist(self, prev_state):
        prev_dx = self._target[0] - prev_state[0]
        prev_dy = self._target[1] - prev_state[1]
        prev_distance = np.sqrt(prev_dx ** 2 + prev_dy ** 2)
        c = np.arctan2(prev_dy, prev_dx)
        prve_dist = prev_distance*abs(np.cos(np.pi/2-self._target[2]+c))
        
        new_dx = self._target[0] - self._state[0]
        new_dy = self._target[1] - self._state[1]
        new_distance = np.sqrt(new_dx ** 2 + new_dy ** 2)
        b= np.arctan2(new_dy, new_dx)
        road_dist = new_distance*abs(np.cos(np.pi/2-self._target[2]+b))
        return (prve_dist - road_dist)
    def _is_dist(self,theta=np.pi/180*5,threshold=10):
        new_dx = self._target[0] - self._state[0]
        new_dy = self._target[1] - self._state[1]
        new_distance = np.sqrt(new_dx ** 2 + new_dy ** 2)
        b= np.arctan2(new_dy, new_dx)
        road_dist = new_distance*np.cos(np.pi/2-self._target[2]+b)
        if abs(self._changetheta)< theta:
            return True
        return False
    
    def _angle_to_destination(self):
        x, y = self._target[0] - self.x, self._target[1] - self.y
        return self._wrap_angles(np.arctan2(y, x) - self.theta)

    def _wrap_angles(self, a):
        return (a + np.pi) % (2 * np.pi) - np.pi

    def _get_state(self):
        dx, dy = self._target[0] - self.x, self._target[1] - self.y
        curr_state = [dx / 1000, dy / 1000, self._angle_to_destination()]
        return np.hstack([curr_state, self._sensors[0].distances])

    def _control(self, state, t):
        """Returns the yaw given state.

        Args:
            state: State.
            t: Time.

        Returns:
            Yaw.
        """
        actions = [-np.pi/180*5, 0., np.pi/180*5]
        if self._sensors[0].is_laser() and self._is_dist():
            optimal_i=1
            optimal_a = actions[optimal_i]
            return optimal_a
        utilities = self._ctrl.evaluate(self._get_state())
        optimal_i = np.argmax(utilities)
        angle = self._changetheta+actions[optimal_i]
        if np.random.random() <= self._exploration and abs(angle)<np.pi/180.0*50.0:
            optimal_i = np.random.choice([0, 1, 2]) 
        elif angle >= np.pi/180.0*50.0:
            optimal_i = 0
        elif angle <= -np.pi/180.0*50.0:
            optimal_i = 2
       
        optimal_a = actions[optimal_i]
        self._changetheta = self._changetheta+optimal_a
        self._selected_i = optimal_i
        return optimal_a*30.0


class Obstacle(MovingObject):

    """Obstacle."""

    def __init__(self, velocity, radius, bounds, height=1.0):
        """Constructs a Robot.

        Args:
            velocity: Velocity of the robot in the forward direction.
            radius: Radius of the obstacle.
        """
        data = DebugData()
        self._bounds = bounds
        self._radius = radius
        self._height = height
        center = [0, 0, height / 2 - 0.5]
        axis = [0, 0, 1]  # Upright cylinder.
        data.addCylinder(center, axis, height, radius)
        data.addCube()
        polydata = data.getPolyData()
        super(Obstacle, self).__init__(velocity, polydata)

    def _control(self, state, t):
        """Returns the yaw given state.

        Args:
            state: State.
            t: Time.

        Returns:
            Yaw.
        """
        x_min, x_max, y_min, y_max = self._bounds
        x, y, theta = state
        if x - self._radius <= x_min:
            return np.pi
        elif x + self._radius >= x_max:
            return np.pi
        elif y - self._radius <= y_min:
            return np.pi
        elif y + self._radius >= y_max:
            return np.pi
        return 0.

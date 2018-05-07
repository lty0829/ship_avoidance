import argparse
import numpy as np
from world import World
from PythonQt import QtGui
from net import Controller
from sensor import RaySensor
from director import applogic
from moving_object import Robot
from moving_object import Commonship
from director import vtkAll as vtk
from director import objectmodel as om
from director.debugVis import DebugData
from director import visualization as vis
from director.consoleapp import ConsoleApp
from director.timercallback import TimerCallback


class Simulator(object):

    """Simulator."""

    def __init__(self, world,size):
        """Constructs the simulator.

        Args:
            world: World.
        """
        self._robots = []
        self._obstacles = []
        self._commonships = []
        self._world = world
        self._app = ConsoleApp()
        self._view = self._app.createView(useGrid=False)

        # performance tracker
        self._num_targets = 0
        self._num_crashes = 0
        self._run_ticks = 0
        self._worldsize=size
        self._play_count = 0

        self._initialize()

    def _initialize(self):
        """Initializes the world."""
        # Add world to view.
        om.removeFromObjectModel(om.findObjectByName("world"))
        vis.showPolyData(self._world.to_polydata(), "world")

    def _add_polydata(self, polydata, frame_name, color):
        """Adds polydata to the simulation.

        Args:
            polydata: Polydata.
            frame_name: Frame name.
            color: Color of object.

        Returns:
            Frame.
        """
        om.removeFromObjectModel(om.findObjectByName(frame_name))
        frame = vis.showPolyData(polydata, frame_name, color=color)

        vis.addChildFrame(frame)
        return frame
    def add_target(self, target):
        data = DebugData()
        center = [target[0], target[1], 1]
        axis = [0, 0, 1]  # Upright cylinder.
        data.addCylinder(center, axis, 2, 3)
        om.removeFromObjectModel(om.findObjectByName("target"))
        self._add_polydata(data.getPolyData(), "target", [0, 0.8, 0])

    def add_robot(self, robot):
        """Adds a robot to the simulation.

        Args:
            robot: Robot.
        """
        color = [0.4, 0.85098039, 0.9372549]
        frame_name = "robot{}".format(len(self._robots))
        frame = self._add_polydata(robot.to_polydata(), frame_name, color)
        self._robots.append((robot, frame))
        self._update_moving_object(robot, frame)

    def add_commonship(self,commonship):
        color = [1, 1, 1]
        frame_name = "commonship{}".format(len(self._commonships))
        frame = self._add_polydata(commonship.to_polydata(), frame_name, color)
        self._commonships.append((commonship, frame))
        self._update_moving_object(commonship, frame)
        
    def add_obstacle(self, obstacle):
        """Adds an obstacle to the simulation.

        Args:
            obstacle: Obstacle.
        """
        color = [1.0, 1.0, 1.0]
        frame_name = "obstacle{}".format(len(self._obstacles))
        frame = self._add_polydata(obstacle.to_polydata(), frame_name, color)
        self._obstacles.append((obstacle, frame))
        self._update_moving_object(obstacle, frame)

    def _update_moving_object(self, moving_object, frame):
        """Updates moving object's state.

        Args:
            moving_object: Moving object.
            frame: Corresponding frame.
        """
        t = vtk.vtkTransform()
        t.Translate(moving_object.x, moving_object.y, 0.0)
        t.RotateZ(np.degrees(moving_object.theta))
        frame.getChildFrame().copyFrame(t)

    def _update_sensor(self, sensor, frame_name):
        """Updates sensor's rays.

        Args:
            sensor: Sensor.
            frame_name: Frame name.
        """
        vis.updatePolyData(sensor.to_polydata(), frame_name,
                           colorByName="RGB255")
    def _get_line(self,x,y,num,color):
        data = DebugData()
        
        data.addLine(x,y,radius=0.7,color=[1, 1, 1])
        polydata = data.getPolyData()
        self._add_polydata(polydata, "line"+str(num), color)
       
        return polydata
    def clear_locator(self):
        d = DebugData()
        for ship, frame in self._commonships:
            d.addPolyData(ship.to_positioned_polydata())
        self.locator = vtk.vtkCellLocator()
        self.locator.SetDataSet(d.getPolyData())
        self.locator.BuildLocator()
    def update_locator(self):
        """Updates cell locator."""
        d = DebugData()
        size = self._worldsize
        #d.addPolyData(self._world.to_polydata())
        for robot, frame in self._robots:
            d.addPolyData(self._get_line([robot._initPos[0],robot._initPos[1]+51,0],[robot._target[0]+30,robot._target[1]+51,0],1,color=[1, 1, 1]))       
            d.addPolyData(self._get_line([robot._initPos[0],robot._initPos[1]-51,0],[robot._target[0]+30,robot._target[1]-51,0],2,color=[1, 1, 1]))
        for ship, frame in self._commonships:
            d.addPolyData(ship.to_positioned_polydata())

        self.locator = vtk.vtkCellLocator()
        self.locator.SetDataSet(d.getPolyData())
        self.locator.BuildLocator()

    def run(self, display):
        """Launches and displays the simulator.

        Args:
            display: Displays the simulator or not.
        """
        if display:
            widget = QtGui.QWidget()
            layout = QtGui.QVBoxLayout(widget)
            layout.addWidget(self._view)
            widget.showMaximized()

            # Set camera.
            applogic.resetCamera(viewDirection=[0.2, 0, -1])

        # Set timer.
        self._tick_count = 0
       
        self._timer = TimerCallback(targetFps=200)
        self._timer.callback = self.tick
        self._timer.start()

        self._app.start()

    def tick(self):
        """Update simulation clock."""
        self._tick_count += 1
        self._run_ticks += 1
        if self._tick_count >= 500:
            print("timeout")
            for robot, frame in self._robots:
                
                self.reset(robot, frame)

        need_update = False
        for obstacle, frame in self._obstacles:
            if obstacle.velocity != 0.:
                obstacle.move()
                self._update_moving_object(obstacle, frame)
                need_update = True
        for commonship,frame in self._commonships:
            self._update_moving_object(commonship, frame)
            commonship.move()
            need_update = True
            size = self._worldsize
            if commonship.x>size/2+40 or commonship.x<-size/2-40 or commonship.y>size/2+40 or commonship.y<-size/2-40:
                ship.velocity=0
        if need_update:
            self.update_locator()
            
      

        for i, (robot, frame) in enumerate(self._robots):
            self._update_moving_object(robot, frame)
            for sensor in robot.sensors:
                sensor.set_locator(self.locator)
            robot.move()
            size = self._worldsize
            for sensor in robot.sensors:
                frame_name = "rays{}".format(i)
                self._update_sensor(sensor, frame_name)
                if sensor.has_collided():
                    self._num_crashes += 1
                    print("collided", min(d for d in sensor._distances if d > 0))
                    print("targets hit", self._num_targets)
                    print("ticks lived", self._run_ticks)
                    print("deaths", self._num_crashes)
                    self._run_ticks = 0
                    self._num_targets = 0
                #    new_target = self.generate_position()
                 #   for robot, frame in self._robots:
                #        robot.set_target(new_target)
               #    self.add_target(new_target)
                    self.reset(robot, frame)
            if robot.x>size/2+2 or robot.x<-size/2:
                self._run_ticks = 0
                
                self.reset(robot, frame)

            if robot.at_target():
                self._run_ticks = 0
                
                self.reset(robot, frame)
          #      self._num_targets += 1
          #      self._tick_count = 0
          #      new_target = self.generate_position()
          #      for robot, frame in self._robots:
          #          robot.set_target(new_target)
          #      self.add_target(new_target)

    def generate_position(self):
        size = self._worldsize
        return tuple(np.random.uniform(-2*size/8, 2*size/8, 2))

    def set_safe_position(self, robot):
        while True:
            size = self._worldsize
      
            for ship,frame in self._commonships:
                #ship.x,ship.y = self.generate_position()
                randomCommonship(ship)
                vis.updatePolyData(ship.to_polydata(),"commonship0")
                randn=np.random.choice([0,1,2,3,4,5,6])
                if randn==0 or randn == 6:
                    self.set_meeting(robot,ship,size)
                elif randn==1 or randn==2:
                    self.set_catch(robot,ship,size)
                else:
                    self.set_side(robot,ship,size)
                
                    
                #ship.velocity=0
                theta = (robot.theta+2*np.pi)%(2*np.pi)
                if theta > 0 and theta <=np.pi/2:
                    x = size/2
                    y = robot.y+(size/2 - robot.x)*np.tan(robot.theta)
                    robot.set_target((x,y,robot.theta))
                    self.add_target((x,y))
              #      print("target",x,y,robot.theta)
                elif theta > np.pi/2 and theta <=np.pi:
                    x = -size/2
                    y = robot.y+(-size/2 - robot.x)*np.tan(robot.theta)
                    robot.set_target((x,y,robot.theta))
                    self.add_target((x,y))
               #     print("target",x,y,robot.theta)
                elif theta > np.pi and theta <=np.pi/2*3:
                    x = -size/2
                    y = robot.y+(-size/2 - robot.x)*np.tan(robot.theta)
                    robot.set_target((x,y,robot.theta))
                    self.add_target((x,y))
               #     print("target",x,y,robot.theta)
                elif theta > np.pi/2*3 and theta <=np.pi*2:
                    x = size/2
                    y = robot.y+(size/2 - robot.x)*np.tan(robot.theta)
                    robot.set_target((x,y,robot.theta))
                    self.add_target((x,y))
                 #   print("target",x,y,robot.theta)
                robot.set_initPos((robot.x,robot.y))   
                     
            robot.attach_sensor(RaySensor())
            if min(robot.sensors[0].distances) >= 0.30:
                return
    def set_meeting(self,robot,ship,size):
        robot.x, robot.y = self.generate_position()
        robot.x=-size/2+0.5+np.random.uniform(0,5)
        dx_1 = size/2-robot.x
        dy_1 = size/2-50-robot.y
        theta_1 = np.arctan2(dy_1,dx_1)
        dx_2 = size/2 -robot.x
        dy_2 = -size/2+50 - robot.y
        theta_2 = np.arctan2(dy_2,dx_2)
        robot.theta = np.random.uniform(theta_2, theta_1)
        ship.theta = (robot.theta+np.pi)%(2*np.pi)
        dist = size-np.random.uniform(0,5)-5
        ship.x =robot.x+dist
        ship.y = robot.y+dist*np.tan(robot.theta)
        ship.velocity = np.random.uniform(30,50)
            #ship.x = 
    def set_catch(self,robot,ship,size):
        randn=np.random.choice([0, 1])
        if randn == 0:
            robot.x, robot.y = self.generate_position()
            robot.x=-size/2+0.5+np.random.uniform(0,5)
            dx_1 = size/2-robot.x
            dy_1 = size/2-50-robot.y
            theta_1 =np.arctan2(dy_1,dx_1)
            dx_2 = size/2 -robot.x
            dy_2 = -size/2+50 - robot.y
            theta_2 =np.arctan2(dy_2,dx_2)
            robot.theta = np.random.uniform(theta_2, theta_1)
            ship.theta = robot.theta
            ship.velocity = np.random.uniform(5,20)
            dist = np.random.uniform(40,50)
            ship.x =robot.x+dist
            ship.y = robot.y+dist*np.tan(robot.theta)
        if randn == 1:
            ship.x, ship.y = self.generate_position()
            ship.x=-size/2+0.5+np.random.uniform(0,5)
            dx_1 = size/2-ship.x
            dy_1 = size/2-50-ship.y
            theta_1 = np.arctan2(dy_1,dx_1)
            dx_2 = size/2 -ship.x
            dy_2 = -size/2 +50- ship.y
            theta_2 = np.arctan2(dy_2,dx_2)
            robot.theta =  np.random.uniform(theta_2, theta_1)
            ship.theta = robot.theta
            ship.velocity = np.random.uniform(60,80)
            dist = np.random.uniform(40,50)
            robot.x =ship.x+dist
            robot.y = ship.y+dist*np.tan(robot.theta)
    def set_side(self,robot,ship,size):
        sx,sy = tuple(np.random.uniform(-size/16, size/16, 2))
        dx_1 = sx+size/2
        dy_1 = sy-size/2+50
        theta_1 = np.arctan2(dy_1,dx_1)
        
        dx_2 = size/2 -sx
        dy_2 = -size/2+50 - sy
        theta_2 = np.arctan2(dy_2,dx_2)
        if theta_2>theta_1:
            theta_1 = theta_2
        
        dx_3 = sx +size/2
        dy_3 = sy +size/2-50
        theta_3 = np.arctan2(dy_3,dx_3)
        
        dx_4 = size/2-sx
        dy_4 = size/2-50-sy
        theta_4 = np.arctan2(dy_4,dx_4)
        
        if theta_4<theta_3:
            theta_3 = theta_4

        
        
        robot.theta = np.random.uniform(theta_1, theta_3)
        theta_n1 = 68.0/180.0*np.pi
        ship.theta = np.random.uniform(robot.theta+theta_n1,2*np.pi+robot.theta-theta_n1)
        theta_1 = np.pi/2-robot.theta
        theta_2 = np.pi/2-ship.theta
        dist1 = np.random.uniform(3*size/8,7*size/16)
        y1 = dist1*np.cos(theta_1)
        x1 = dist1*np.sin(theta_1)
        robot.x = sx-x1
        robot.y = sy-y1
        t = dist1/40.0
        
        dist2 = np.random.uniform(3*size/8,7*size/16)
        y2 = dist2*np.cos(theta_2)
        x2 = dist2*np.sin(theta_2)
        ship.x = sx-x2
        ship.y = sy-y2
        ship.velocity = dist2/t
            
        

    def reset(self, robot, frame_name):
        self._tick_count = 0
        robot.init_angle()
        self.clear_locator()
        om.removeFromObjectModel(om.findObjectByName("line1"))
        om.removeFromObjectModel(om.findObjectByName("line2"))
        self._play_count+=1
        self.set_safe_position(robot)
        print("count:",self._play_count)
        self._update_moving_object(robot, frame_name)
        robot._ctrl.save()


def get_args():
    """Gets parsed command-line arguments.

    Returns:
        Parsed command-line arguments.
    """
    parser = argparse.ArgumentParser(description="avoids obstacles")
    parser.add_argument("--obstacle-density", default=0.01, type=float,
                       help="area density of obstacles")
    parser.add_argument("--moving-obstacle-ratio", default=0.0, type=float,
                        help="percentage of moving obstacles")
    parser.add_argument("--exploration", default=0.3, type=float,
                        help="exploration rate")
    parser.add_argument("--learning-rate", default=0.01, type=float,
                        help="learning rate")
    parser.add_argument("--no-display", action="store_false", default=True,
                        help="whether to display the simulator or not",
                        dest="display")

    return parser.parse_args()
def randomCommonship(ship):
    scale = np.random.uniform(6,14)
    velocity = np.random.uniform(30,50)
    ship.velocity = velocity
    ship.setScale(scale)

if __name__ == "__main__":
    args = get_args()
    size = 200
    world = World(size, size)
    sim = Simulator(world,size)
   # for obstacle in world.generate_obstacles(args.obstacle_density,
    #                                         args.moving_obstacle_ratio):
     #   sim.add_obstacle(obstacle)
        
    print(args.obstacle_density)
    print(args.moving_obstacle_ratio)

    sim.update_locator()

    target = sim.generate_position()
  #  sim.add_target(target)

    controller = Controller(args.learning_rate)
    controller.load()

    robot = Robot(exploration=args.exploration,size=size)
    
    robot.set_target(target)
    robot.attach_sensor(RaySensor())
    robot.set_controller(controller)
    
    ship = Commonship()
    sim.add_robot(robot)
    sim.add_commonship(ship)
     
    robot.set_ship(ship)
    sim.set_safe_position(robot)
  

    sim.run(args.display)
    controller.save()
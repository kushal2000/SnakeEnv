import numpy
from shapely.geometry import Polygon
from IPython import embed
from matplotlib import pyplot as plt
from matplotlib import patches
from matplotlib.collections import PatchCollection
import seaborn as sns

from PlanarArmEnvironment import PlanarArmEnvironment


def get_polygon(boundary_points):
    '''
    Returns a shapely polygon object given the boundary points.
    '''
    return 0

# =================================================================================================

'''
Class for the N-Link Planar Arm
'''
class PlanarArm(object):
    def __init__(self, env):
        # Name of the planar arm.
        self.name = 'planar_arm'

        # Environment the planar arm is working in.
        self.env = env

        # Location of the fixed base. 
        # Default at the center.
        self.base_location = numpy.array([self.env.get_scale()/2.0]*2)

        # Degrees of freedom of the arm. 
        # Default set to 3.
        self.dimension = 7

        # Flag for whether self collisions are allows (planar or offset links).
        # Default set to True.
        self.allow_self_collision = None

        # Length/Width of the links.
        self.link_length = 0.05
        self.link_width = 0.01

        # Configuration of the planar arm
        self.configuration = numpy.zeros(self.dimension)

    def get_configuration(self):
        '''
        @return Configuration of the planar arm.
        '''
        return self.configuration

    def set_configuration(self, configuration):
        '''
        Sets the configuration of the planar arm.
        @param configuration to set with angles specified in radians.
        '''
        if numpy.shape(configuration)[0] == self.dimension:
            self.configuration = configuration
        else:
            raise ValueError("Dimension mismatch!")

    def get_base_location(self):
        '''
        @return base location
        '''
        return self.base_location

    def set_base_location(self, base_location):
        '''
        Sets the fixed base location of the planar arm.
        @param base_location Fixed base location of the planar arm.
        '''
        if numpy.shape(base_location)[0] == 2:
            self.base_location = base_location
        else:
            raise ValueError("Dimension mismatch!")

    def set_dimension(self, dim):
        '''
        Sets the degrees of freedom of the planar arm.
        @param dof DOF of the arm.
        '''
        self.dimension = dim

    def get_dimension(self):
        '''
        @return DOF of the arm.
        '''
        return self.dimension

    def get_link_length(self):
        '''
        @return link length
        '''
        return self.link_length

    def set_link_length(self, length):
        '''
        Sets the length of the links.
        @param length Length of the links of the arm.
        '''
        if length > 0:
            self.link_length = length
        else:
            raise ValueError("Length has to be positive!")
    
    def get_link_width(self):
        '''
        @return link width
        '''
        return self.link_width

    def set_link_width(self, width):
        '''
        Sets the width of the links.
        @param width Width of the links of the arm.
        '''
        if width > 0:
            self.link_width = width
        else:
            raise ValueError("Length has to be positive!")

    def get_self_collision_flag(self):
        '''
        @return If self-collision is allowed (true) or not (false).
        '''
        return self.allow_self_collision

    def set_self_collision_flag(self, value):
        '''
        Sets if self-collision is allowed.
        @param value True if self-collision is allowed.
        '''
        self.allow_self_collision = value
    
    def is_reachable(self, point):
        '''
        Checks if the point is reachable with the current arm.
        @param point in 2D to check for reachability.
        '''
        translated_position = point - self.base_location
        radius = self.dimension*self.link_length
        distance_from_center = numpy.linalg.norm(point)
        if distance_from_center < radius:
            return True
        return False

    def in_collision(self):
        '''
        @return true if the arm is in collision
        '''
        obstacles = self.env.get_obstacles()

        if obstacles is None:
            return False

        for dimension in range(self.get_dimension()):
            bounding_points = self.get_link_bounding_points(dimension + 1)
            link_polygon = Polygon(bounding_points)
            for obstacle in obstacles:
                obstacle_points = []
                for point in obstacle:
                    obstacle_points.append(tuple(point)) 
                obstacle_polygon = Polygon(obstacle_points)
                if link_polygon.intersects(obstacle_polygon):
                    print("Link {} in collision".format(dimension + 1))
                    return True
        return False
    
    def get_end_effector_position(self):
        '''
        @return the [x, y] position of the end-effector.
        '''
        position = numpy.zeros(2)
        for dim in range(self.dimension):
            position[0] += self.link_length*numpy.cos(self.configuration[dim])
            position[1] += self.link_length*numpy.sin(self.configuration[dim])
        return self.base_location + position
    
    def get_link_base_position(self, link_index):
        '''
        @returns the base location of the link of interest.
        @param link_index Index of the link of interest. Indexed at 1.
        '''
        if link_index > self.dimension or link_index <= 0:
            raise ValueError("Link index out of bounds!")

        max_dim = link_index - 1

        position = numpy.zeros(2)
        for dim in range(max_dim):
            position[0] += self.link_length*numpy.cos(self.configuration[dim])
            position[1] += self.link_length*numpy.sin(self.configuration[dim])
        return self.base_location + position

    def get_link_bounding_points(self, link_index):
        '''
        @returns the boundary points of the link of interest.
        @param link_index Index of the link of interest. Indexed at 1.
        '''
        dim = link_index - 1
        angle = self.configuration[dim]
        base = self.get_link_base_position(link_index)
        rotation_matrix = numpy.array(
          [[numpy.cos(angle), -numpy.sin(angle)],[numpy.sin(angle), numpy.cos(angle)]])
        vectors = numpy.array(
          [[0, self.link_length, self.link_length, 0],[-self.link_width, -self.link_width, self.link_width, self.link_width]])
        converted_vectors = numpy.matmul(rotation_matrix, vectors)

        points = []
        for i in range(numpy.shape(converted_vectors)[1]):
            point = base + converted_vectors[:,i]
            points.append((point[0], point[1]))
        
        return points
    
    def visualize(self):
        '''
        Visualize the arm and the environment
        '''
        fig = plt.figure()
        ax = plt.gca()

        obstacles = self.env.get_obstacles()
        obs = []
        for obstacle in obstacles:
            rect = patches.Rectangle([obstacle[0, 0], obstacle[0, 1]], obstacle[2,0]-obstacle[0,0], obstacle[2,1]-obstacle[0,1])
            obs.append(rect)

        pc = PatchCollection(obs, facecolor=sns.light_palette("red")[1], edgecolor='None')
        ax.add_collection(pc)
        

        for i in range(self.dimension):
            bounding_points = self.get_link_bounding_points(i+1)
            coords = [numpy.asarray(point) for point in bounding_points]
            coords.append(coords[0]) # close the loop.
            xs, ys = zip(*coords)

            plt.plot(xs, ys, 'k') 

        plt.xlim([0,1])
        plt.ylim([0,1])
        ax.set_aspect('equal', 'box')
        plt.show()

# =================================================================================================

if __name__ == '__main__':

    env = PlanarArmEnvironment()
    env.set_obstacles([[[0, 0],[0.1, 0],[0.1, 0.1],[0, 0.1]]])
    env.add_obstacle([[[0.45, 0.45],[0.55, 0.45],[0.55, 0.55],[0.45, 0.55]]])
    arm = PlanarArm(env)
    embed()
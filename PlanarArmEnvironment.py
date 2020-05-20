import numpy
from IPython import embed
from matplotlib import pyplot as plt
from matplotlib import patches
from matplotlib.collections import PatchCollection
import seaborn as sns

'''
Class the describes the environment of the planar arm.
'''

class PlanarArmEnvironment(object):
    def __init__(self):
        # Name of the environment.
        self.name = 'env'

        # Size of the environment.
        # Default set to unit square.
        self.scale = 1

        # Obstacles in the environment
        self.obstacles = None

    def get_scale(self):
        '''
        @return size of the square environment.
        '''
        return self.scale

    def set_scale(self, scale):
        '''
        Sets the dimensions of the square environment.
        @param scale Dimension of the square environment.
        '''
        self.scale = scale

    def get_obstacles(self):
        '''
        @return obstacles defined in the environment.
        '''
        return self.obstacles

    def add_obstacle(self, obstacle):
        '''
        Appends the current set of obstacles to existing obstacles
        @param obstacles Obstacles to add to the environment.
        '''
        if self.obstacles is None:
            self.set_obstacles(obstacle)
        else:
            self.obstacles = numpy.append(self.obstacles, obstacle, 0)

    def set_obstacles(self, obstacles):
        '''
        Sets the obstacles after clearning the existing ones if any.
        @param obstacles Obstacles to set in the environment.
        '''
        self.obstacles = obstacles
    
    def visualize(self):
        '''
        Visualize the environment
        '''
        fig = plt.figure()
        ax = plt.gca()

        obs = []
        for obstacle in self.obstacles:
            rect = patches.Rectangle([obstacle[0, 0], obstacle[0, 1]], obstacle[2,0]-obstacle[0,0], obstacle[2,1]-obstacle[0,1])
            obs.append(rect)

        pc = PatchCollection(obs, facecolor=sns.light_palette("red")[1], edgecolor='None')
        ax.add_collection(pc)
        ax.set_aspect('equal', 'box')
        plt.show()

if __name__ == '__main__':
    env = PlanarArmEnvironment()
    embed()
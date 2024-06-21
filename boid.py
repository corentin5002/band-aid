import utils as ut
import variables as var
import numpy as np
import uuid


class Boid:
    def __init__(self, position, velocity, acceleration, name='', type='agent'):
        self.id = uuid.uuid4()
        self.name = 'b_' + name
        self.vision = var.BOT_VISION

        # Flocking variables
        self.position = position
        self.velocity = velocity
        self.acceleration = acceleration
        self.neighbors = []
        # Define the type of boid :
        # agent : moving boid (define later platelet and messenger)
        # obstacle : static boid
        self.type = type
        self.color = var.WHITE

class Platelet(Boid):
    def __init__(self, position, velocity, acceleration, name='', type='agent'):
        super().__init__(position, velocity, acceleration, name, type)
        self.color = var.RED
        self.neighbors = []
        self.obstacles = []

    def separate(self, neighbors, desired_separation):
        steer = np.zeros(2)

        separationDistance = var.SEP_MINIMAL
        if len(neighbors) > 0 and neighbors[0].type != 'agent':
            separationDistance = var.BOT_VISION

        for neighbor in neighbors:

            diff = self.position - neighbor.position
            if 0. < np.linalg.norm(diff) < separationDistance:
                steer += diff / np.linalg.norm(diff)
            elif np.linalg.norm(diff) == 0:
                steer += diff
        return steer

    def align(self, neighbors):
        steer = np.zeros(2)
        for neighbor in neighbors:
            steer += neighbor.velocity

        if len(neighbors) != 0:
            steer /= len(neighbors)
            steer -= self.velocity
        return steer

    def cohesion(self, neighbors):
        steer = np.zeros(2)
        for neighbor in neighbors:
            steer += neighbor.position

        # handle case neighbors = []
        if len(neighbors) != 0:
            steer /= len(neighbors)
            steer -= self.position
        return steer

    def avoid_obstacles(self, obstacles):
        pass

    def avoid_walls(self):
        nextPosition = self.position + self.velocity * var.OBS_AVOIDANCE

        while not ut.isInField(nextPosition):
            rotation = np.random.randint(0,60)
            rotation = np.deg2rad(rotation - 30)
            rotationMatrix = np.array([[np.cos(rotation), -np.sin(rotation)], [np.sin(rotation), np.cos(rotation)]])

            self.velocity  = np.dot(rotationMatrix, self.velocity)
            nextPosition = self.position + self.velocity * 3

        # Old version "bounce" against the wall
        # if nextPosition[0] < 0 or nextPosition[0] > var.WIDTH_FIELD:
        #     self.velocity[0] *= -1
        #
        # elif nextPosition[1] < 0 or nextPosition[1] > var.HEIGHT_FIELD :
        #     self.velocity[1] *= -1


    def neighbors(self, neighbors):
        self.neighbors = neighbors

    def updateColor(self):
        if len(self.neighbors) != 0:
            self.color = var.BLUE
        else:
            self.color = var.RED


class Obstacle(Boid):
    def __init__(self, position, velocity, acceleration, name='', type='obstacle'):
        super().__init__(position, velocity, acceleration, name, type)
        self.color = var.BLACK

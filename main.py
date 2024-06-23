# libraries
import pygame
import quads
import numpy as np

# Functions
import variables as var
import utils as ut

# region Quadtree
import quads as qd

quadsField = qd.QuadTree((0, 0), var.WIDTH_FIELD, var.HEIGHT_FIELD)

# force pygame to open on the right screen
desired_display = 0

# initialize pygame and create window
pygame.init()
pygame.mixer.init()
screen = pygame.display.set_mode((var.WIDTH_SCREEN, var.HEIGHT_SCREEN), display=desired_display)
pygame.display.set_caption("Flocking Simulation")
clock = pygame.time.Clock()
dt = 0

# region Objects initialization
Obstacles, quadsFieldObstacles = ut.spawnObstacle(0, 63)
Boids, quadsField = ut.spawnBoid(10, 8, 63)
ut.insertObjectsQuadTree(Obstacles, quadsField)

# quads.visualize(quadsField)
# endregion Objects initialization

# endregion

# region Game loop

Config = {
    "running" : True,
    "play" : False,
    "labels" : False,
    "vision" : False,
    "neighborsLines" : False,
    "obstacleLines" : True,
    "velocity" : False,
    "showSpecs" : False,
    "FPS" : 1
}

# region keys

# endregion keys


while Config["running"]:

    # keep loop running at the right speed
    clock.tick(var.FPS)
    dt += 1
    # region handling events
    ut.handleEvents(pygame, Config, Boids, Obstacles, quadsField, quadsFieldObstacles)

    screen.fill(var.WHITE)
    ut.drawWalls(pygame, screen, quadsField)

    # region Update

    update = False
    if Config['play'] and dt % Config["FPS"] == 0:
        play = True
        update = True

        for boid in Boids :
                sep = boid.separate(boid.neighbors, 2)
                coh = boid.cohesion(boid.neighbors)
                ali = boid.align(boid.neighbors)
                obs = boid.separate(boid.obstacles, 2)

                total_force = (
                    sep * var.SEP_WEIGHT
                    + coh * var.COH_WEIGHT
                    + ali * var.ALI_WEIGHT
                    + obs * var.OBS_AVOIDANCE
                   )

                # Limit speed of the boid
                boid.velocity += total_force
                boid.avoid_walls()

                boid.velocity = np.clip(boid.velocity, -var.MAX_SPEED_PLATELET, var.MAX_SPEED_PLATELET)

                boid.position += (boid.velocity).astype(int)
                boid.acceleration = np.zeros(2)

        quadsField = None
        quadsField = ut.updateQuadTree(Boids)

    # endregion Update

    # region Rendering
    for boid in Boids:
        if update:
            boid.neighbors = ut.getNeighbors(boid.position, boid.vision, Boids, quadsField)
            boid.obstacles = ut.getNeighbors(boid.position, boid.vision, Obstacles, quadsFieldObstacles)
            boid.updateColor()

        ut.drawBot(
            pygame,
            screen,
            boid,
            label=Config['labels'],
            vision=Config['vision'],
            neighborsLines=Config['neighborsLines'],
            obstacleLines=Config['obstacleLines'],
            velocity=Config['velocity']
        )

    for obstacle in Obstacles:
        ut.drawBot(
            pygame,
            screen,
            obstacle,
            label=Config['labels'],
            vision=Config['vision'],
            neighborsLines=Config['neighborsLines'],
            velocity=Config['velocity']
        )

    ut.drawSpecs(pygame, screen, showSpecs=Config['showSpecs'])
    # endregion rendering

    # region Draw / render
    pygame.display.flip()
    # endregion Draw / render


pygame.quit()
# endregion Game loop

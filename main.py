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

# region Boids initialization
Boids, quadsField = ut.spawnBoid(2, 10, 63)



# quads.visualize(quadsField)
# endregion Boids initialization

# endregion

# region Game loop
running = True

# region keys
play = False
labels = False
vision = False
neighborsLines = False
velocity = False
FPS = 1

# endregion keys


while running:

    # keep loop running at the right speed
    clock.tick(var.FPS)
    dt += 1
    # region Process input (events)
    for event in pygame.event.get():
        # check for closing window
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                play = not play
            if event.key == pygame.K_l:
                labels = not labels

            if event.key == pygame.K_v:
                vision = not vision

            if event.key == pygame.K_n:
                neighborsLines = not neighborsLines

            if event.key == pygame.K_s:
                velocity = not velocity

            if event.key == pygame.K_UP:
                FPS += 1

            if event.key == pygame.K_DOWN:
                FPS -= 1
                if FPS < 1:
                    FPS = 1




    # endregion Process input (events)

    screen.fill(var.WHITE)
    ut.drawWalls(pygame, screen, quadsField)

        # quads.visualize(quadsField)
    # region Update

    update = False
    if play and dt % FPS == 0:
        play = True
        update = True

        print(f'===============ROUND {dt}==============')
        for boid in Boids :
                sep = boid.separate(boid.neighbors, 2)
                coh = boid.cohesion(boid.neighbors)
                ali = boid.align(boid.neighbors)

                total_force = (
                    sep * var.SEP_WEIGHT
                    + coh * var.COH_WEIGHT
                    + ali * var.ALI_WEIGHT
                   )

                # Limit speed of the boid
                boid.velocity += total_force
                boid.find_obstacle()

                boid.velocity = np.clip(boid.velocity, -var.MAX_SPEED_PLATELET, var.MAX_SPEED_PLATELET)

                boid.position += (boid.velocity).astype(int)
                boid.acceleration = np.zeros(2)

        quadsField = None
        quadsField = ut.updateQuadTree(Boids)

    for boid in Boids:
        if update:
            boid.neighbors = ut.getNeighbors(boid.position, boid.vision, Boids, quadsField)

        ut.drawBot(
            pygame,
            screen,
            boid,
            label=labels,
            vision=vision,
            neighborsLines=neighborsLines,
            velocity=velocity
        )

    # endregion Update

    # region Draw / render
    pygame.display.flip()
    # endregion Draw / render


pygame.quit()
# endregion Game loop
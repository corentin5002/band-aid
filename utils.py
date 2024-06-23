import numpy as np
import variables as var
import quads as qd
from boid import Boid, Platelet, Obstacle
import utils as ut

# region Spawn boid
def spawnObstacle(numberOfObstacles=10, seed=var.RANDOM_SEED):
    np.random.seed(seed)

    Obstacles = []
    quadsFieldObstacles = qd.QuadTree((0, 0), var.WIDTH_FIELD, var.HEIGHT_FIELD)
    for i in range(numberOfObstacles):
        position = np.round(np.random.rand(2) * np.random.randint(0, 802, 2)).astype(int)

        Obstacles.append(Obstacle(
            position,
            np.zeros(2),
            np.zeros(2),
            name=str(i),
            type='obstacle'
        ))
        ut.insertQuadTree(Obstacles[i].position, quadsFieldObstacles)

    return Obstacles, quadsFieldObstacles

def spawnBoid(numberOfBoidsPerGroup=10, numberOfGroups=1, seed=var.RANDOM_SEED):
    np.random.seed(seed)

    Boids = []
    quadsField = qd.QuadTree((0, 0), var.WIDTH_FIELD, var.HEIGHT_FIELD)
    for group in range(numberOfGroups):
        groupPosition = np.round(np.random.rand(2) * np.random.randint(0, 802, 2)).astype(int)
        velocity = np.round(np.random.rand(2) * 20)
        direction = [np.random.choice([-1, 1]) for i in range(2)]
        velocity = velocity * direction
        for i in range(numberOfBoidsPerGroup):

            #Spread the boid around the group position
            position = (groupPosition + (np.random.rand(2) * 90)).astype(int)

            while not isInField(position):
                position = (groupPosition + (np.random.rand(2) * 90)).astype(int)

            # Random velocity global to the group
            velocity = velocity

            direction = [np.random.choice([-1, 1]) for i in range(2)]
            acceleration = np.random.rand(2) * direction

            Boids.append(Platelet(
                position,
                velocity,
                acceleration,
                name=str(i)
            ))
            # Add boid position to quadtree
        ut.insertQuadTree(Boids[(group + 1) * i].position, quadsField)

    for boid in Boids:
        boid.neighbors = ut.getNeighbors(boid.position, boid.vision, Boids, quadsField)

    return Boids, quadsField

# endregion

# Check if the position is within the field
def isInField(position):
    return 0 < position[0] < var.WIDTH_FIELD and 0 < position[1] < var.HEIGHT_FIELD

# region Change coordinate to quadTree referential
def pygameToQuadtree(position):
    # change the origin of the coordinate system (Top left -> Center)
    normPosition = np.array(position)
    normPosition[0] = normPosition[0] - var.WIDTH_FIELD // 2
    normPosition[1] = - (normPosition[1] - var.HEIGHT_FIELD // 2 )

    return tuple(normPosition)

def quadtreeToPygame(position):
    # change the origin of the coordinate system (Center -> Top left)
    normPosition = np.array([position.x, position.y])
    normPosition[0] = normPosition[0] + var.WIDTH_FIELD // 2
    normPosition[1] = - normPosition[1] + var.HEIGHT_FIELD // 2

    return normPosition

def insertQuadTree(position, quadTree):
    normPosition = pygameToQuadtree(position)
    quadTree.insert(normPosition)

def insertObjectsQuadTree(objects, quadTree):
    for object in objects:
        insertQuadTree(object.position, quadTree)
def getNeighbors(position, vision , Boids, quadTree):
    # Convert to quadtree coordinate
    normPosition = pygameToQuadtree(position)

    # Collect the max number of neighbors
    quadPositions = quadTree.nearest_neighbors(normPosition, var.NBR_NEIGHBORS)
    # Exclude the ones that are too far (euclidean distance)
    temp = []
    for pos in quadPositions:
        distance = np.linalg.norm(position - quadtreeToPygame(pos))
        if 0 < distance < vision:
            temp.append(pos)
    quadPositions = temp[:]

    # Convert back to pygame coordinate
    normPositions = [quadtreeToPygame(normPosition) for normPosition in quadPositions]

    # Get the neighbors
    neighbors = []
    for normPosition in normPositions:
        for boid in Boids:
            if np.array_equal(normPosition, boid.position):
                neighbors.append(boid)
                break

    return neighbors

def getPosition(position, quadTree):
    normPosition = np.array(position)
    normPosition[0] = normPosition[0] + var.WIDTH_FIELD // 2
    normPosition[1] = - normPosition[1] + var.HEIGHT_FIELD // 2

def updateQuadTree(boids):
    quadsField = qd.QuadTree((0, 0), var.WIDTH_FIELD, var.HEIGHT_FIELD)

    for boid in boids:
        insertQuadTree(boid.position, quadsField)

    return quadsField

# endregion Change coordinate to quadTree referential

# region Draw Functions
def drawBot(pygame, screen, bot, label=False, vision= False, neighborsLines=False, velocity=False, obstacleLines=False):

    if len(bot.neighbors) > 0:
        color = var.BLUE

    pygame.draw.rect(
        screen,
        bot.color,
        (
            bot.position[0] * var.PIXEL_SIZE - var.BOT_SIZE // 2,  # x
            bot.position[1] * var.PIXEL_SIZE - var.BOT_SIZE // 2,  # y
            var.BOT_SIZE,
            var.BOT_SIZE
        ),
        0
    )
    if vision:
        pygame.draw.circle(
            screen,
            var.BLUE_VISION,
            (bot.position[0] * var.PIXEL_SIZE, bot.position[1] * var.PIXEL_SIZE),
            bot.vision * var.PIXEL_SIZE,
            1
        )
    if label:
        botLabel = (bot.name
                    # + ' '
                    # + str(bot.position)
                    )
        font = pygame.font.Font(None, var.FONT_SIZE)
        text = font.render(botLabel,True, (0, 0, 0))
        screen.blit(text, (bot.position[0] * var.PIXEL_SIZE - var.BOT_SIZE // 2 , bot.position[1] * var.PIXEL_SIZE - var.BOT_SIZE * 1.5))

    if neighborsLines:
        for neighbor in bot.neighbors:
            pygame.draw.line(
                screen,
                var.BLUE_VISION,
                (bot.position[0] * var.PIXEL_SIZE, bot.position[1] * var.PIXEL_SIZE),
                (neighbor.position[0] * var.PIXEL_SIZE, neighbor.position[1] * var.PIXEL_SIZE),
                1
            )
    if obstacleLines and bot.type != 'obstacle':
        for neighbor in bot.obstacles:
            pygame.draw.line(
                screen,
                var.BLUE_VISION,
                (bot.position[0] * var.PIXEL_SIZE, bot.position[1] * var.PIXEL_SIZE),
                (neighbor.position[0] * var.PIXEL_SIZE, neighbor.position[1] * var.PIXEL_SIZE),
                1
            )

    if velocity:
        pygame.draw.line(
            screen,
            var.BLACK,
            (bot.position[0] * var.PIXEL_SIZE, bot.position[1] * var.PIXEL_SIZE),
            (bot.position[0] * var.PIXEL_SIZE + bot.velocity[0], bot.position[1] * var.PIXEL_SIZE + bot.velocity[1]),
            2
        )

# endregion draw bot

# region obstacles
def drawObstacle(pygame, screen, position, size, color):

    pygame.draw.rect(
        screen,
        color,
        (
            position[0] * var.PIXEL_SIZE,  # x
            position[1] * var.PIXEL_SIZE,  # y
            position[0] * var.PIXEL_SIZE + size,
            position[1] * var.PIXEL_SIZE + size
        ),
        0
    )


# endregion obstacles

def drawWalls(pygame, screen, quadField):
    size = 4
    for width in range(var.WIDTH_FIELD):
        drawObstacle(
            pygame,
            screen,
            (width, 0),
            size,
            var.TEST
        )
        drawObstacle(
            pygame,
            screen,
            (width, var.HEIGHT_FIELD - size),
            size,
            var.TEST
        )

    for height in range(var.HEIGHT_FIELD):
        drawObstacle(
            pygame,
            screen,
            (0, height),
            size,
            var.TEST
        )
        drawObstacle(
            pygame,
            screen,
            (var.WIDTH_SCREEN - size, height),
            size,
            var.TEST
        )

def drawSpecs(pygame, screen, showSpecs=False):

    font = pygame.font.Font(None, var.FONT_SIZE)

    if showSpecs:
        specs = [ "Bot Speed : " + str(var.MAX_SPEED_PLATELET) + " range [0, 200]",
                  "",
                  "Commands :",
                  "- Space : Pause/Play",
                  "- l : Show/Hide labels (laggy)",
                  "- v : Show/Hide vision range",
                  "- n : Show/Hide neighbors links (coolest view)",
                  "- o : Show/Hide obstacle links",
                  "- s : Show/Hide velocity vector",
                  "- UP/DOWN : Decrease/Increase Simulation speed",
                  "- LEFT/RIGHT : Decrease/Increase Bot speed",
                  "- ESC : Quit"
                  ]
        for spec in specs:
            text = font.render(spec,True, (0, 0, 0))
            screen.blit(text, (20, 10 + specs.index(spec) * var.FONT_SIZE))


    else:
        text = font.render("Press h to show specs",True, (130, 130, 130))
        screen.blit(text, (20, 10))



def createStringSpecs(key, value):
    return key + ': ' + str(value)

# endregion Draw Functions

# region Event

def handleEvents(pygame, config, Boids, Obstacles, quadField, quadsFieldObstacles):
    for event in pygame.event.get():
        # check for closing window
        if event.type == pygame.QUIT:
            config['running'] = False

        if event.type == pygame.MOUSEBUTTONDOWN:
            mouseLeft, mouseCenter, mouseRight = pygame.mouse.get_pressed()

            if mouseLeft:
                position = pygame.mouse.get_pos()
                position = np.array([
                    position[0] // var.PIXEL_SIZE ,
                    position[1] // var.PIXEL_SIZE
                ])
                Obstacles.append(Obstacle(position, np.zeros(2), np.zeros(2), name=str(len(Obstacles)), type='obstacle'))
                insertQuadTree(position, quadsFieldObstacles)

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                config['running'] = False

            if event.key == pygame.K_SPACE:
                config['play'] = not config['play']

            if event.key == pygame.K_l:
                config['labels'] = not ['labels']

            if event.key == pygame.K_v:
                config['vision'] = not config['vision']

            if event.key == pygame.K_n:
                config['neighborsLines'] = not config['neighborsLines']

            if event.key == pygame.K_o:
                config['obstacleLines'] = not config['obstacleLines']

            if event.key == pygame.K_s:
                config['velocity'] = not config['velocity']

            if event.key == pygame.K_UP:
                config['FPS'] += 1

            if event.key == pygame.K_DOWN:
                config['FPS'] -= 1
                if config['FPS'] < 1:
                    config['FPS'] = 1

            if event.key == pygame.K_LEFT:
                var.MAX_SPEED_PLATELET -= 1
                if var.MAX_SPEED_PLATELET < 0:
                    var.MAX_SPEED_PLATELET = 0

            if event.key == pygame.K_RIGHT:
                var.MAX_SPEED_PLATELET += 1
                if var.MAX_SPEED_PLATELET > 200:
                    var.MAX_SPEED_PLATELET = 200

            if event.key == pygame.K_h:
                config['showSpecs'] = not config['showSpecs']

# endregion
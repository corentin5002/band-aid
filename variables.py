PIXEL_SIZE = 1
BOT_SIZE = 5
BOT_VISION = 32.
FONT_SIZE = 16
WIDTH_SCREEN = 800
WIDTH_FIELD = WIDTH_SCREEN // PIXEL_SIZE
HEIGHT_SCREEN = 800
HEIGHT_FIELD = HEIGHT_SCREEN // PIXEL_SIZE


FPS = 30


WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
TEST = (150, 100, 35)
BLUE = (0, 0, 255)
BLUE_VISION = (0, 120, 120)

# Flocking variables
SEP_WEIGHT = 5
SEP_MINIMAL = BOT_VISION * 0.4
COH_WEIGHT = 0.3
ALI_WEIGHT = 0.5
OBS_AVOIDANCE = 3
# always add 1 to the number of neighbors because the quadtree return the point itself if it exist within the quadtree
NBR_NEIGHBORS = 7

MAX_SPEED_PLATELET = 8
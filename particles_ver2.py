import brickpi
import math
from random import gauss
import random
import numpy as np

#Particles represented as a np array [x, y, theta]
NUMBER_OF_PARTICLES = 50
SONAR_OFFSET = 0
PARTICLES = []
ALL_PARTICLES = []
WEIGHTS = []
#std deviations of each noise term
E = 0.3
F = 0.02
G = 0.03
#fraction that accounts for sensor garbage
K = 0.00001


# A Canvas class for drawing a map and particles:
# 	- it takes care of a proper scaling and coordinate transformation between
#	  the map frame of reference (in cm) and the display (in pixels)
class Canvas:
    def __init__(self,map_size=210):
        self.map_size    = map_size;    # in cm;
        self.canvas_size = 768;         # in pixels;
        self.margin      = 0.05*map_size;
        self.scale       = self.canvas_size/(map_size+2*self.margin);

    def drawLine(self,line):
        x1 = self.__screenX(line[0]);
        y1 = self.__screenY(line[1]);
        x2 = self.__screenX(line[2]);
        y2 = self.__screenY(line[3]);
        #print "drawLine:" + str((x1,y1,x2,y2))

    def drawParticles(self):
        toTuple = lambda v : (self.__screenX(v[0]), self.__screenY(v[1]), v[2])
        #display = [(self.__screenX(d[0]),self.__screenY(d[1])) + d[2:] for d in data];
        #print "drawParticles:" + str(map(toTuple, PARTICLES))

    def drawParticlesWithWeights(self):
        toTuple = lambda v : (self.__screenX(v[0][0]), self.__screenY(v[0][1]), v[0][2], v[1])
    #    print "drawParticles:" + str(map(toTuple, zip(PARTICLES,WEIGHTS)))

    def __screenX(self,x):
        return (x + self.margin)*self.scale

    def __screenY(self,y):
        return (self.map_size + self.margin - y)*self.scale

# A Map class containing walls
class Map:
    def __init__(self):
        self.walls = [];

    def add_wall(self,wall):
        self.walls.append(wall);

    def clear(self):
        self.walls = [];

    def draw(self):
        for wall in self.walls:
            canvas.drawLine(wall);

#canvas = Canvas();	# global canvas we are going to draw on

mymap = Map();
# Definitions of walls
# a: O to A
# b: A to B
# c: C to D
# d: D to E
# e: E to F
# f: F to G
# g: G to H
# h: H to O
mymap.add_wall((0,0,0,168));        # a
mymap.add_wall((0,168,84,168));     # b
mymap.add_wall((84,126,84,210));    # c
mymap.add_wall((84,210,168,210));   # d
mymap.add_wall((168,210,168,84));   # e
mymap.add_wall((168,84,210,84));    # f
mymap.add_wall((210,84,210,0));     # g
mymap.add_wall((210,0,0,0));        # h
#mymap.draw();

# def drawParticles():
#   #magically draws particles in the web interface...
#   toTuple = lambda v : (10*v[0], 10*v[1], v[2])
#   # #print "drawParticles:" + str(map(toTuple, ALL_PARTICLES))
#   #print "drawParticles:" + str(map(toTuple, ALL_PARTICLES))


def initialiseParticles(x,y,theta):
  global PARTICLES, ALL_PARTICLES, WEIGHTS
  PARTICLES = [np.array([x, y, theta + gauss(0, math.radians(2))]) for _ in range(NUMBER_OF_PARTICLES)]
  ALL_PARTICLES += PARTICLES
  WEIGHTS = [1./NUMBER_OF_PARTICLES for _ in range(NUMBER_OF_PARTICLES)]

def updateParticleStraight(particle,D,e,f):
  t = particle[2]
  return np.array([particle[0]+(D+e)*math.cos(t), particle[1]+(D+e)*math.sin(t), t+f])

def updateAllParticlesStraight(distance):
  global PARTICLES, ALL_PARTICLES
  PARTICLES = map(lambda p : updateParticleStraight(p, distance, gauss(0, E), gauss(0, F)), PARTICLES)
 # ALL_PARTICLES += PARTICLES

def updateAllParticlesRotate(alpha):
  global PARTICLES, ALL_PARTICLES
  PARTICLES = map(lambda p : p + np.array([0, 0, alpha + gauss(0, G)]), PARTICLES)
 # ALL_PARTICLES += PARTICLES

def getPosition():
  return sum([p*w for (p, w) in zip(PARTICLES, WEIGHTS)])

#calculates the angle between the sonar and the normal of the closest wall
#used to determine whether the current sonar reading is valid
def normalAngleToWall(particle, wall):
    [x,y,theta] = particle
    (A_x, A_y, B_x, B_y) = wall
    top = math.cos(theta) * (A_y-B_y) + math.sin(theta) * (B_x - A_x)
    bottom = math.sqrt(pow((A_y-B_y), 2) + pow((B_x - A_x),2))
    angle = math.acos(top/bottom)
    return angle

#this gets the intersection point of where the line of sight of the robot would hit the wall, and finds out whether it is wihtin bounds
def willHitWall(m, particle, wall):
    [x,y,theta] = particle
    eps = 3
    x_intersect = x + m * math.cos(theta)
    y_intersect = y + m * math.sin(theta)
    #print "x intersect: ",x_intersect, "y intersect: " , y_intersect
    (x1,y1,x2,y2) = wall
    #print x1,y1,x2,y2
    within_x = (min(x1,x2)-eps) < x_intersect  and x_intersect < (max(x1,x2) + eps)
    within_y = (min(y1,y2) - eps) < y_intersect and y_intersect < (max(y1,y2) + eps)
    return within_x and within_y

#returns the distance to the nearest wall, as well as whether the reading is likely to be accurate or not
def distanceToNearestWall(particle, walls):
    nearest_distance = 255
    nearest_wall = walls[0]
    for wall in walls:
        m = calculateDistanceToWall(particle, wall)
        #closest distance to a wall
        if m > 0 and nearest_distance > m:
            if willHitWall(m,particle,wall):
                nearest_distance = m
                nearest_wall = wall
    #calculates the angle to the normal of the closest wall
    # normal_angle = normalAngleToWall(particle, nearest_wall)
    # #whether the angle to the normal is too great for the reading to be accurate
    # inaccurate_angle = (normal_angle > math.pi/5)
#    print "nearest wall :" , nearest_wall, "angle to normal of wall: ", normal_angle
    return nearest_distance, False

def calculateDistanceToWall(particle, wall):
    [x,y,theta] = particle
    (A_x, A_y, B_x, B_y) = wall
    top = (B_y - A_y)*(B_x - x) - (B_x- A_x)*(A_y-y)
    bottom = (B_y - A_y)*math.cos(theta) - (B_x - A_x)*math.sin(theta)
    if bottom == 0:
        return float("inf")
    distance = top / bottom
    return distance

def calculateLikelihood(particle, sonar_reading):
    actual_distance,inaccurate = distanceToNearestWall(particle, mymap.walls)
    #print "estimated distance: ", actual_distance, " inaccurate: ", inaccurate, " sonar reading: ", sonar_reading
    #likelihood function
    sonar_value = sonar_reading + SONAR_OFFSET
    #print "sonar: ", sonar_reading+SONAR_OFFSET, "actual distance: ", actual_distance
    new_weight = np.exp(-np.power((sonar_value - actual_distance),2) / (2 * np.power(math.sqrt(sonar_value),2))) + K
    #print new_weight
    return new_weight, inaccurate


#normalise the current WEIGHT so that the total probability add up to 1
def normaliseWeights():
    global WEIGHTS
    prob_sum = sum(WEIGHTS)
    WEIGHTS = map(lambda x: x/prob_sum, WEIGHTS)

#calculates the new weights for particles based on the likelihood function
#update the WEIGHTS vector to un-normalised probabilities
def updateWeights(sonar_reading):
  global WEIGHTS
  new_weights = []
  inaccurate_count = 0

  for i in range(0, len(PARTICLES)):
    #for each particle, calculate its likelihood, and whether the reading is likely to be accurate
    weight, inaccurate = calculateLikelihood(PARTICLES[i], sonar_reading)

    new_weights.append(weight)
    if inaccurate:
        inaccurate_count += 1
  #check if the number of particles that will give inaccurate readings exceed threshhold
  #only updates global weights if particles agree
  #if inaccurate_count < 50:
    WEIGHTS = new_weights
    #  canvas.drawParticlesWithWeights()
    normaliseWeights()
 # canvas.drawParticles()

#searches the cumulative array to find the particle to copy
def binsearchCumuArray(value, array):
    upper = len(array)-1
    lower = 0
    while(upper > lower + 1):
        mid = int ((upper + lower) /2)
        if array[mid] < value:
          lower = mid
        else:
          upper = mid
    return upper


#optionally replace PARTICLES by resampling based on current weights
def resampleParticles():
    global NUMBER_OF_PARTICLES, WEIGHTS, PARTICLES
    cumulative_array = []
    cumulative_array.append(WEIGHTS[0])
    #generating the cumulative array using probabilities
    for i in range(1, NUMBER_OF_PARTICLES):
        cumulative_array.append(WEIGHTS[i] + cumulative_array[i-1])
    new_particles = []
    for i in range(0, NUMBER_OF_PARTICLES):
        ran = random.uniform(0.0,1.0)
        #retrieves an index from the cumulative array to go to the particle list to make a copy
        new_particles.append(np.copy(PARTICLES[binsearchCumuArray(ran, cumulative_array)]))
    PARTICLES = new_particles
    #reset all weights to be equal
    WEIGHTS = [1./NUMBER_OF_PARTICLES for _ in range(NUMBER_OF_PARTICLES)]
    #canvas.drawParticles()
# initialiseParticles(10, 10, 0)
# for i in range(4):
#   for j in range(4):
#       updateAllParticlesStraight(10)
#       drawParticles()
#   updateAllParticlesRotate(math.pi/2)

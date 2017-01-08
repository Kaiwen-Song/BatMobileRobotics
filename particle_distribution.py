import brickpi
import math
from random import gauss

NUMBER_OF_PARTICLES = 100
PARTICLES = []
ALL_PARTICLES = []
#std deviations of each noise term
E = 0.1
F = 0.010
G = 0.005

class Particle:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
	self.weight = 10
    def __update(self, new_x,new_y,new_theta):
        self.x = new_x
        self.y = new_y
        self.theta = new_theta
    def toTriple(self):
	return (self.weight*self.x, self.weight*self.y, self.theta)
    def updateParticleStraight(self,D,e,f):
        t = self.theta
        self.__update(self.x+(D+e)*math.cos(t), self.y+(D+e)*math.sin(t), t+f)
    def updateParticleRotate(self, a, g):
        self.__update(self.x, self.y, self.theta+a+g)


def particleWeight(particle):
    return 1/NUMBER_OF_PARTICLES
def drawParticles():
    #magically draws particles in the web interface...
    print "drawParticles:" + str(ALL_PARTICLES)
def initialiseParticles(x,y,theta):
    global PARTICLES, ALL_PARTICLES
    for i in range(0, NUMBER_OF_PARTICLES):
        PARTICLES.append(Particle(x,y,theta))
    ALL_PARTICLES += map(lambda p: p.toTriple(), PARTICLES)
def updateAllParticlesStraight(distance):
    global PARTICLES, ALL_PARTICLES
    for particle in PARTICLES:
        particle.updateParticleStraight(distance, gauss(0, E), gauss(0, F))
    ALL_PARTICLES += map(lambda p: p.toTriple(), PARTICLES)
def updateAllParticlesRotate(alpha):
    global PARTICLES, ALL_PARTICLES
    for particle in PARTICLES:
        particle.updateParticleRotate(alpha, gauss(0, G))
    ALL_PARTICLES += map(lambda p: p.toTriple(), PARTICLES)


import numpy as np
import turtle

# Simulation parameters
rM0 = 4.60
vM0 = 5.10e-1
baseAccel = 9.90e-1
TM = 8.80e+1
rS = 2.95e-7
rL2 = 8.19e-7

# Definition of the timestep
dt=2*vM0/baseAccel/20

# Function for updating position and velocity of planet
def getAccel(posMerc_old, velMerc_old, alpha = 0, beta = 0):

	# Compute the strength of the acceleration
	correction = 1 + alpha*rS/np.linalg.norm(posMerc_old) + beta*rL2/np.linalg.norm(posMerc_old)**2
	accelMag = baseAccel * correction/np.linalg.norm(posMerc_old)**2
	# Multiply by the direction
	accelMerc = -accelMag * (posMerc_old/np.linalg.norm(posMerc_old))
	return accelMerc


# Set initial conditions
posInit = np.array([0,rM0])
velInit = np.array([vM0, 0])
t = 0.0

screen = turtle.getscreen()
screen.bgcolor("black")
#screen.mode('world')
screen.setworldcoordinates(-10,-10,10,10)

Sun = turtle.Turtle(shape="circle")
Sun.color("yellow")

Mercury = turtle.Turtle(shape="circle")

Mercury.penup()
Mercury.color("red")
Mercury.goto(posInit[0], posInit[1])
Mercury.pendown()
turtle.tracer(n=1, delay=0)

posMerc = posInit
velMerc = velInit

while t < 2*TM:

	# Compute Accel
	accelMerc = getAccel(posMerc, velMerc, 0, 0)

	# Update velocity vector
	velMerc = velMerc+accelMerc*dt
	# Update position vector
	posMerc = posMerc+velMerc*dt

	Mercury.setpos(posMerc[0], posMerc[1])

	# Move to new time
	t = t + dt

screen.exitonclick()	

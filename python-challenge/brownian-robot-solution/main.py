#! /usr/bin/env python3

import turtle
import random
import time

class BrownianRobot:
	"""
	Class for simulation of Brownian motion on Robot in 2D arena.

	Attributes:
		None
	Methods:
		drawBoundary(): Draws the Boundary of arena
		init_robot(): Initializes the Turtle Robot
		rotate_to_left(angle): Rotates the robot to left side
		rotate_to_right(angle): Rotates the robot to right side
		closeWindow(): Closes the window
		do_simulation(): Starts the simulation

	"""

	def __init__(self):
		"""
		The constructor for BrownianRobot class.

		Parameters:
			window (class): Screen or Window of GUI
			boundWidth (int): Width of Arena
			boundHeight (int): Height of Arena
			default_vel (float): Default velocity of the robot
			dt (int): Time
		"""
		self.window = turtle.Screen()
		self.window.title("Brownian Motion On Turtle Robot")
		self.window.bgcolor("lightgreen")
		self.window.setup(600, 400)

		self.boundWidth = 500;
		self.boundHeight = 300;

		self.default_vel = 1
		self.dt = 1

		self.init_robot()


	def drawBoundary(self):
		"""
		Draws the 600x600 Boundary to create arena. Where we will see the simulation of robot Using Brownian Motion

		Parameters:
			None
		Returns:
			None
		"""
		boundTurtle = turtle.Turtle()
		boundTurtle.penup()
		boundTurtle.setposition(-self.boundWidth / 2, -self.boundHeight / 2)
		boundTurtle.pendown()
		boundTurtle.pensize(3)

		boundTurtle.forward(self.boundWidth)
		boundTurtle.left(90)
		boundTurtle.forward(self.boundHeight)
		boundTurtle.left(90)
		boundTurtle.forward(self.boundWidth)
		boundTurtle.left(90)
		boundTurtle.forward(self.boundHeight)

		boundTurtle.hideturtle()

	def init_robot(self):
		"""
		Initializes the Turtle Robot.

		Parameters:
			None
		Returns:
			None
		"""
		self.robot = turtle.Turtle()

		self.robot.color("green")
		self.robot.shape("turtle")
		self.robot.pendown()
		self.robot.pensize(2)

		self.robot.setheading(random.randint(0, 360))

	def rotate_to_left(self, angle):
		"""
		Rotates the robot to its left with given angle.

		Parameters:
			angle (int): Angle to ratate to left side of robot.
		Returns:
			None
		"""
		for _ in range(angle):
			self.robot.left(1)
			time.sleep(0.001) # waits for 1 milli second.

	def rotate_to_right(self, angle):
		"""
		Rotates the robot to its right with given angle.

		Parameters:
			angle (int): Angle to ratate to right side of robot.
		Returns:
			None
		"""
		for _ in range(angle):
			self.robot.right(1)
			time.sleep(0.001) # waits for 1 milli second.

	def closeWindow(self):
		"""
		Closes the GUI Window.

		Parameters:
			None
		Returns:
			None
		"""
		self.window.bye()

	def do_simulation(self):
		"""
		Robot starts moving in forward direction, On collision with the boundary, the robot
		will rotate for a random duration and then keep moving forward in the set direction.

		Parameters:
			None.
		Returns:
			None
		"""
		self.robot.forward(self.dt * self.default_vel)

		# When robot hits the upper wall
		if self.robot.ycor() > self.boundHeight / 2:

			curr_angle = self.robot.heading()
			rand_dt = random.randint(0, 180) # Random duration

			if(curr_angle > 0  and curr_angle < 90):
				self.rotate_to_right(round(curr_angle - 0))
				for _ in range(rand_dt): # Rotates for random duration.
					self.robot.right(1)
					time.sleep(0.001) # waits for 1 milli second.
			else:
				self.rotate_to_left(round(180 - curr_angle))
				for _ in range(rand_dt):
					self.robot.left(1)
					time.sleep(0.001) # waits for 1 milli second.

			self.robot.forward(3)

		# When robot hits the left wall
		if self.robot.xcor() < - self.boundWidth / 2:

			curr_angle = self.robot.heading()
			rand_dt = random.randint(0, 180)

			if(curr_angle > 90 and curr_angle < 180):
				self.rotate_to_right(round(curr_angle - 90))
				for _ in range(rand_dt):
					self.robot.right(1)
					time.sleep(0.001) # waits for 1 milli second.
			else:
				self.rotate_to_left(round(270 - curr_angle))
				for _ in range(rand_dt):
					self.robot.left(1)
					time.sleep(0.001) # waits for 1 milli second.

			self.robot.forward(3)

		# When robot hits the lower wall
		if self.robot.ycor() < - self.boundHeight / 2:

			curr_angle = self.robot.heading()
			rand_dt = random.randint(0, 180)

			if(curr_angle > 180 and curr_angle < 270):
				self.rotate_to_right(round(curr_angle - 180))
				for _ in range(rand_dt):
					self.robot.right(1)
					time.sleep(0.001) # waits for 1 milli second.
			else:
				self.rotate_to_left(round(360 - curr_angle))
				for _ in range(rand_dt):
					self.robot.left(1)
					time.sleep(0.001) # waits for 1 milli second.

			self.robot.forward(3)

		# When robot hits the right wall
		if self.robot.xcor() > self.boundWidth / 2:

			curr_angle = self.robot.heading()
			rand_dt = random.randint(0, 180)

			if(curr_angle > 270 and curr_angle < 360):
				self.rotate_to_right(round(curr_angle - 270))
				for _ in range(rand_dt):
					self.robot.right(1)
					time.sleep(0.001) # waits for 1 milli second.
			else:
				self.rotate_to_left(round(90 - curr_angle))
				for _ in range(rand_dt):
					self.robot.left(1)
					time.sleep(0.001) # waits for 1 milli second.

			self.robot.forward(3)

		# Press the space or exit button to close the GUI window
		self.window.onkey(self.closeWindow, "space")
		self.window.listen()


def main():
	browRobot = BrownianRobot()
	browRobot.drawBoundary()

	while True:
		browRobot.do_simulation()

if __name__ == '__main__':
	main()
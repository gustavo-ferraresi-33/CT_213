# Simulation Parameters
SCREEN_WIDTH = 640
SCREEN_HEIGHT = 480
PIX2M = 0.01  # factor to convert from pixels to meters
M2PIX = 100.0  # factor to convert from meters to pixels

# Sample Time Parameters
FREQUENCY = 60.0  # simulation frequency
SAMPLE_TIME = 1.0 / FREQUENCY  # simulation sample time

# Behavior Parameters

# Times relevant to DFSM state transitions
# "t2"
t2 = MOVE_FORWARD_TIME = 3.0  # time moving forward before switching to the spiral behavior
n2 = t2 * FREQUENCY # number of sampling cycles elapsed in the time "t2"
# "t1"
t1 = MOVE_IN_SPIRAL_TIME = 20.0  # time moving in spiral before switching back to moving forward
n1 = t1 * FREQUENCY # number of sampling cycles elapsed in the time "t1"
# "t3"
t3 = GO_BACK_TIME = 0.5  # time going back after hitting a wall
n3 = t3 * FREQUENCY # number of sampling cycles elapsed in the time "t3"

# Default speeds
FORWARD_SPEED = 0.5  # default linear speed when going forward
BACKWARD_SPEED = -0.1 # default backward speed when going back after hitting a wall
ANGULAR_SPEED = 0.5  # default angular speed

# Spiral geometric parameters
r0 = INITIAL_RADIUS_SPIRAL = 0.2  # initial spiral radius
b = SPIRAL_FACTOR = 0.05  # factor used to make the spiral grow while the time passes

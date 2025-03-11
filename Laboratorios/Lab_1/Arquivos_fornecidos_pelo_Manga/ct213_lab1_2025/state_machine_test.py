from utils import Pose
from constants import FREQUENCY
from roomba import Roomba
from simulation import *
from state_machine import FiniteStateMachine, MoveForwardState

pygame.init()

window = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

pygame.display.set_caption("Lab 1 - Roomba Finite State Machine")

clock = pygame.time.Clock()

behavior = FiniteStateMachine(StateName.MOVE_FORWARD)
# DUVIDA: why "behavior" should receive a behavior tree, which the implementation we are focused at (DSFM) does not depend on ?
# behavior = RoombaBehaviorTree()
pose = Pose(PIX2M * SCREEN_WIDTH / 2.0, PIX2M * SCREEN_HEIGHT / 2.0, 0.0)
roomba = Roomba(pose, 1.0, 2.0, 0.34 / 2.0, behavior)
simulation = Simulation(roomba, clock)

run = True

while run:
    clock.tick(FREQUENCY)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False

    simulation.update(clock)
    draw(simulation, window)

pygame.quit()

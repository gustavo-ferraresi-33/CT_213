import random
import math

from pygame.time import Clock
from constants import *
from roomba import *
from utils import *
from enum import Enum

class StateName(Enum):
    """
    Represents the state of the DFSM (Deterministic Finite State Machine)
    """
    MOVE_FORWARD = 0
    MOVE_IN_SPIRAL = 1
    GO_BACK = 2
    ROTATE = 3

class FiniteStateMachine(object):
    """
    A finite state machine.
    """
    def __init__(self, state_name: StateName):
        new_state = State(state_name)
        self.state = new_state

    def change_state(self, new_state_name: StateName):
        new_state = State(new_state_name)
        self.state = new_state

    def update(self, agent: Roomba, clock: Clock):
        self.state.check_transition(agent, self, clock)
        self.state.execute(agent, clock)


class State(object):
    """
    Abstract state class.
    """
    def __init__(self, state_name: StateName):
        """
        Creates a state.

        :param state_name: the name of the state.
        :type state_name: str
        """
        self.state_name = state_name

    def check_transition(self, agent: Roomba, fsm: FiniteStateMachine, clock: Clock):
        """
        Checks conditions and execute a state transition if needed.

        :param agent: the agent where this state is being executed on.
        :param fsm: finite state machine associated to this state.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")

    def execute(self, agent: Roomba, clock: Clock):
        """
        Executes the state logic.

        :param agent: the agent where this state is being executed on.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")


class MoveForwardState(State):
    def __init__(self):
        # TODO: add initialization code
        super().__init__(StateName.MOVE_FORWARD)
        # DUVIDA: did I forget to initialize something?

    def check_transition(self, agent: Roomba, state_machine: FiniteStateMachine, clock: Clock):
        # TODO: add logic to check and execute state transition
        # If a collision is detected
        if agent.get_bumper_state():
            # Go to the state GoBackState
            state_machine.change_state(StateName.GO_BACK)
        # If maximum time t2 is surpassed
        elif clock.get_rawtime() >= t2:
            # Go to the state MoveInSpiral
            state_machine.change_state(StateName.MOVE_IN_SPIRAL)

    def execute(self, agent: Roomba, clock: Clock):
        # TODO: add execution logic
        # Change velocity to move forwards
        agent.set_velocity(FORWARD_SPEED, 0)

class MoveInSpiralState(State):
    def __init__(self):
        # TODO: add initialization code
        super().__init__(StateName.MOVE_IN_SPIRAL)
        # DUVIDA: did I forget to initialize something?

    def check_transition(self, agent: Roomba, state_machine: FiniteStateMachine, clock: Clock):
        # TODO: add logic to check and execute state transition
        # If a collision is detected
        if agent.get_bumper_state():
            # Go to the state GoBackState
            state_machine.change_state(StateName.GO_BACK)
        # If maximum time t1 is surpassed
        elif clock.get_rawtime() >= t1:
            # Go to the state MoveInSpiral
            state_machine.change_state(StateName.MOVE_IN_SPIRAL)            

    def execute(self, agent: Roomba, clock: Clock):
        # TODO: add execution logic
        # Instantaneous curvature radius
        r = r0 + b * clock.get_rawtime()
        # Change velocity to move in spiral
        agent.set_velocity(FORWARD_SPEED, FORWARD_SPEED/r)



class GoBackState(State):
    def __init__(self):
        # TODO: add initialization code
        super().__init__(StateName.GO_BACK)
        # DUVIDA: did I forget to initialize something?

    def check_transition(self, agent: Roomba, state_machine: FiniteStateMachine, clock: Clock):
        # TODO: add logic to check and execute state transition
        # If maximum time t3 is surpassed
        if clock.get_rawtime() >= t3:
            # Go to the state MoveInSpiral
            state_machine.change_state(StateName.ROTATE)            
        
    def execute(self, agent: Roomba, clock: Clock):
        # TODO: add execution logic
        # Change velocity to move backwards
        agent.set_velocity(-BACKWARD_SPEED, 0)
        # DUVIDA: call "check_transition()" ?
        

class RotateState(State):
    def __init__(self):
        # TODO: add initialization code
        super().__init__(StateName.ROTATE)
        # DUVIDA: did I forget to initialize something?

    def check_transition(self, agent: Roomba, state_machine: FiniteStateMachine, clock: Clock):
        # TODO: add logic to check and execute state transition
        # Go to the state MoveInSpiral
        state_machine.change_state(StateName.MOVE_FORWARD)
    
    def execute(self, agent: Roomba, clock: Clock):
        # TODO: add execution logic
        # Spin of a random angle in [-pi; pi) with uniform PDF
        agent.pose.rotation += math.pi * random.uniform(-1, 1)


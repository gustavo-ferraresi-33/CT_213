import random
import math
from constants import *

from enum import Enum
# from AUX_roomba import Roomba

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
    def __init__(self, state_name):
        new_state = State(state_name)
        self.state = new_state

    def change_state(self, new_state_name):
        new_state = State(new_state_name)
        self.state = new_state

    def update(self, agent):
        self.state.check_transition(agent, self)
        self.state.execute(agent)


class State(object):
    """
    Abstract state class.
    """
    def __init__(self, n: int = 0, *, state_name):
        """
        Creates a state.

        :param state_name: the name of the state.
        :type state_name: StateName
        """
        # Initialize sampling cycles counter "n"
        self.n = n
        self.state_name = state_name

    def check_transition(self, agent, fsm):
        """
        Checks conditions and execute a state transition if needed.

        :param agent: the agent where this state is being executed on.
        :param fsm: finite state machine associated to this state.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")

    def execute(self, agent):
        """
        Executes the state logic.

        :param agent: the agent where this state is being executed on.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")


class MoveForwardState(State):
    def __init__(self):
        super().__init__(state_name = StateName.MOVE_FORWARD)
        # TODO: add initialization code
        # Set sampling cycles counter "n" to "0"
        self.n = 0

    def check_transition(self, agent, state_machine):
        # TODO: add logic to check and execute state transition
        # If a collision is detected
        if agent.get_bumper_state():
            # Go to the state GoBackState
            state_machine.change_state(StateName.GO_BACK)
        # If the maximum number of sampling cycles n2 is surpassed
        elif self.n >= n2:
            # Go to the state MoveInSpiral
            state_machine.change_state(StateName.MOVE_IN_SPIRAL)

    def execute(self, agent):
        # TODO: add execution logic
        # Increment sampling cycles counter "n"
        self.n += 1
        # Change velocity to move forwards
        agent.set_velocity(FORWARD_SPEED, 0)

class MoveInSpiralState(State):
    def __init__(self):
        # TODO: add initialization code
        super().__init__(state_name = StateName.MOVE_IN_SPIRAL)
        # Set sampling cycles counter "n" to "0"
        self.n = 0

    def check_transition(self, agent, state_machine):
        # TODO: add logic to check and execute state transition
        # If a collision is detected
        if agent.get_bumper_state():
            # Go to the state GoBackState
            state_machine.change_state(StateName.GO_BACK)
        # If the maximum number of sampling cycles n1 is surpassed
        elif self.n >= n1:
            # Go to the state MoveInSpiral
            state_machine.change_state(StateName.MOVE_IN_SPIRAL)            

    def execute(self, agent):
        # TODO: add execution logic
        # Increment sampling cycles counter "n"
        self.n += 1
        # Instantaneous curvature radius
        r = r0 + b * self.n * SAMPLE_TIME
        # Change velocity to move in spiral
        agent.set_velocity(FORWARD_SPEED, FORWARD_SPEED/r)



class GoBackState(State):
    def __init__(self):
        # TODO: add initialization code
        super().__init__(state_name = StateName.GO_BACK)
        # Set sampling cycles counter "n" to "0"
        self.n = 0

    def check_transition(self, agent, state_machine):
        # TODO: add logic to check and execute state transition
        # If the maximum number of sampling cycles n3 is surpassed
        if self.n >= n3:
            # Go to the state MoveInSpiral
            state_machine.change_state(StateName.ROTATE)            
        
    def execute(self, agent):
        # TODO: add execution logic
        # Increment sampling cycles counter "n"
        self.n += 1
        # Change velocity to move backwards
        agent.set_velocity(-BACKWARD_SPEED, 0)
        # DUVIDA: call "check_transition()" ?


class RotateState(State):
    def __init__(self):
        # TODO: add initialization code
        super().__init__(state_name = StateName.ROTATE)
        # Set sampling cycles counter "n" to "0"
        self.n = 0

    def check_transition(self, agent, state_machine):
        # TODO: add logic to check and execute state transition
        # Go to the state MoveForward
        state_machine.change_state(StateName.MOVE_FORWARD)
    
    def execute(self, agent):
        # TODO: add execution logic
        # Increment sampling cycles counter "n"
        self.n += 1
        # Spin of a random angle in [-pi; pi) with uniform PDF
        agent.pose.rotation += math.pi * random.uniform(-1, 1)


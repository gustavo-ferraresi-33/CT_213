import random
import math
from constants import *

from enum import Enum
from Arquivos_fornecidos_pelo_Manga.ct213_lab1_2025.roomba import main_phase

class StateName(Enum):
    MOVE_FORWARD = 0
    MOVE_IN_SPIRAL = 1
    GO_BACK = 2
    ROTATE = 3



class FiniteStateMachine(object):
    """
    A finite state machine.
    ::
    """
    def __init__(self, state_name):
        state = None

        match state_name:
            case StateName.MOVE_FORWARD:
                state = MoveForwardState()
            case StateName.MOVE_IN_SPIRAL:
                state = MoveInSpiralState()
            case StateName.GO_BACK:
                state = GoBackState()
            case StateName.ROTATE:
                state = RotateState()
            case _:
                raise ValueError("Error! Invalid state name.")

        self.state = state


    def change_state(self, new_state_name):
        self.__init__(new_state_name)

    def update(self, agent):
        self.state.check_transition(agent, self)
        self.state.execute(agent, self)


class State(object):
    """
    Abstract state class.
    """
    def __init__(self, state_name = None, n = 0):
        """
        Creates a state.

        :param state_name: the name of the state.
        :type state_name: StateName
        :param n: sampling cycles counter
        :type n: int
        """

        # Initializing sampling cycles counter
        self.n = n
        # Initializing the state itself
        self.state_name = state_name

    def check_transition(self, agent, fsm):
        """
        Checks conditions and execute a state transition if needed.

        :param agent: the agent where this state is being executed on.
        :param fsm: finite state machine associated to this state.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")

    def execute(self, agent, fsm):
        """
        Executes the state logic.

        :param agent: the agent where this state is being executed on.
        :param fsm: finite state machine associated to this state.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")


class MoveForwardState(State):
    def __init__(self):
        super().__init__(StateName.MOVE_FORWARD)
        # Todo: add initialization code

    def check_transition(self, agent, fsm):
        # Todo: add logic to check and execute state transition
        # If a collision happened, go to state "GO_BACK"
        if agent.get_bumper_state():
            agent.set_bumper_state(False)
            fsm.change_state(StateName.GO_BACK)
        # If the time for executing the state "MOVE_FORWARD" is up, go to state "MOVE_IN_SPIRAL"
        elif self.n >= n2:
            fsm.change_state(StateName.MOVE_IN_SPIRAL)



    def execute(self, agent, fsm):
        # Todo: add execution logic
        # Setting roomba's velocity
        agent.set_velocity(FORWARD_SPEED, 0)
        # Incrementing sampling cycles counter
        self.n += 1

class MoveInSpiralState(State):
    def __init__(self):
        super().__init__(StateName.MOVE_IN_SPIRAL)
        # Todo: add initialization code

    def check_transition(self, agent, fsm):
        # Todo: add logic to check and execute state transition
        # If a collision happened, go to state "GO_BACK"
        if agent.get_bumper_state():
            agent.set_bumper_state(False)
            fsm.change_state(StateName.GO_BACK)
        # If the time for executing the state "MOVE_IN_SPIRAL" is up, go to state "MOVE_FORWARD"
        elif self.n >= n1:
            fsm.change_state(StateName.MOVE_FORWARD)

    def execute(self, agent, fsm):
        # Todo: add execution logic
        # Instantaneous curvature radius
        r = r0 + b * self.n * SAMPLE_TIME
        # Change velocity to move in spiral
        agent.set_velocity(FORWARD_SPEED, FORWARD_SPEED / r)
        # Incrementing sampling cycles counter
        self.n += 1


class GoBackState(State):
    def __init__(self):
        super().__init__(StateName.GO_BACK)
        # Todo: add initialization code

    def check_transition(self, agent, fsm):
        # Todo: add logic to check and execute state transition
        # If a collision happened or the time for executing the state "GO_BACK" is up,
        # go to state "ROTATE"
        if agent.get_bumper_state() or self.n >= n3:
            agent.set_bumper_state(False)
            fsm.change_state(StateName.ROTATE)

    def execute(self, agent, fsm):
        # Todo: add execution logic
        # Change velocity to go back
        agent.set_velocity(BACKWARD_SPEED, 0)
        # Incrementing sampling cycles counter
        self.n += 1


class RotateState(State):
    def __init__(self, delta_rotation = math.pi * random.uniform(-1, 1)):
        super().__init__(StateName.ROTATE)
        # Todo: add initialization code
        self.delta_rotation = main_phase(delta_rotation)

    def check_transition(self, agent, fsm):
        # Todo: add logic to check and execute state transition
        # If a collision occurred, raise an error
        if agent.get_bumper_state():
            raise RuntimeError("Error! Roomba collided while rotating.")
        # If the time for executing the state "ROTATE" is up, go to state "MOVE_FORWARD"
        elif self.n >= math.ceil(math.fabs(self.delta_rotation / (ANGULAR_SPEED * SAMPLE_TIME))):
            fsm.change_state(StateName.MOVE_FORWARD)

    def execute(self, agent, fsm):
        # Todo: add execution logic

        # Changing the roomba's velocity to make it rotate, in the shortest amount of time, to the
        # desired value of "agent.pose.inclination"
        if self.delta_rotation >= 0:
            # Spin in clockwise sense (positive w)
            agent.set_velocity(0, ANGULAR_SPEED)
        else:
            # Spin in counterclockwise sense (negative w)
            agent.set_velocity(0, -ANGULAR_SPEED)

        # Incrementing sampling cycles counter
        self.n += 1

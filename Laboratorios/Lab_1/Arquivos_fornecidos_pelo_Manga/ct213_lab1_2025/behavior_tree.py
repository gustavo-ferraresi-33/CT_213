from enum import Enum

from constants import *
import random
import math

from Arquivos_fornecidos_pelo_Manga.ct213_lab1_2025.roomba import main_phase

class NodeStatus(Enum):
    """
    Represents the visitation status of a behavior tree node.
    """
    NOT_VISITED = 0
    VISITING = 1
    VISITED = 2

class ExecutionStatus(Enum):
    """
    Represents the execution status of a behavior tree node.
    """
    SUCCESS = 0
    FAILURE = 1
    RUNNING = 2


class BehaviorTree(object):
    """
    Represents a behavior tree.
    """
    def __init__(self, root=None):
        """
        Creates a behavior tree.

        :param root: the behavior tree's root node.
        :type root: TreeNode
        """
        self.root = root
        self.root.enter()

    def update(self, agent):
        """
        Updates the behavior tree.

        :param agent: the agent this behavior tree is being executed on.
        """
        if self.root is not None:
            self.root.execute(agent)


class TreeNode(object):
    """
    Represents a node of a behavior tree.
    """
    def __init__(self, status = None, node_name = "No name attributed", parent = None, n = 0):
        """
        Creates a node of a behavior tree.

        :param node_name: the name of the node.
        """
        self.status = status
        self.node_name = node_name
        self.parent = parent
        self.n = n

    def enter(self, agent):
        """
        This method is executed when this node is entered.

        :param agent: the agent this node is being executed on.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")

    def execute(self, agent):
        """
        Executes the behavior tree node logic.

        :param agent: the agent this node is being executed on.
        :return: node status (success, failure or running)
        :rtype: ExecutionStatus
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")


class LeafNode(TreeNode):
    """
    Represents a leaf node of a behavior tree.
    """
    def __init__(self, status = None, node_name = "No name attributed", parent = None, n = 0):
        super().__init__(status = status, node_name = node_name, parent = parent, n = n)


class CompositeNode(TreeNode):
    """
    Represents a composite node of a behavior tree.
    """
    def __init__(self, status = None, node_name = "No name attributed", parent = None, n = 0):
        super().__init__(status = status, node_name = node_name, parent = parent, n = n)
        self.children = []


    def add_child(self, child):
        """
        Adds a child to this composite node.

        :param child: child to be added to this node.
        :type child: TreeNode
        """

        child.parent = self
        self.children.append(child)


class SequenceNode(CompositeNode):
    """
    Represents a sequence node of a behavior tree.
    """
    def __init__(self, status=NodeStatus.NOT_VISITED, node_name="No name attributed", parent=None, n=0):
        super().__init__(status=status, node_name=node_name, parent=parent, n=n)
        # We need to keep track of the last running child when resuming the tree execution
        self.running_child = None


    def enter(self, agent = None):
        # When this node is entered, no child should be running
        self.running_child = None
        # Change node status to VISITING
        self.status = NodeStatus.VISITING


    def execute(self, agent):
        if self.running_child is None:
            # If a child was not running, then the node puts its first child to run
            self.running_child = self.children[0]
            self.running_child.enter(agent)

        loop = True
        while loop:
            # Execute the running child
            status = self.running_child.execute(agent)
            if status == ExecutionStatus.FAILURE:
                self.n = 0
                # This is a sequence node, so any failure results in the node failing
                self.running_child = None
                return ExecutionStatus.FAILURE
            elif status == ExecutionStatus.RUNNING:
                # If the child is still running, then this node is also running
                return ExecutionStatus.RUNNING
            elif status == ExecutionStatus.SUCCESS:
                # If the child returned success, then we need to run the next child or declare success
                # if this was the last child
                index = self.children.index(self.running_child)
                if index + 1 < len(self.children):
                    self.running_child = self.children[index + 1]
                    self.running_child.enter(agent)
                else:
                    self.n = 0
                    self.running_child = None
                    return ExecutionStatus.SUCCESS


class SelectorNode(CompositeNode):
    """
    Represents a selector node of a behavior tree.
    """

    def __init__(self, status=NodeStatus.NOT_VISITED, node_name="No name attributed", parent=None, n=0):
        super().__init__(status=status, node_name=node_name, parent=parent, n=n)
        # We need to keep track of the last running child when resuming the tree execution
        self.running_child = None


    def enter(self, agent = None):
        # When this node is entered, no child should be running
        self.running_child = None
        # Change node status to VISITING
        self.status = NodeStatus.VISITING


    def execute(self, agent):
        if self.running_child is None:
            # If a child was not running, then the node puts its first child to run
            self.running_child = self.children[0]
            self.running_child.enter(agent)

        loop = True
        while loop:
            # Execute the running child
            status = self.running_child.execute(agent)
            if status == ExecutionStatus.FAILURE:
                # This is a selector node, so if the current node failed, we have to try the next one.
                # If there is no child left, then all children failed and the node must declare failure.
                index = self.children.index(self.running_child)
                if index + 1 < len(self.children):
                    self.running_child = self.children[index + 1]
                    self.running_child.enter(agent)
                else:
                    self.n = 0
                    self.running_child = None
                    return ExecutionStatus.FAILURE
            elif status == ExecutionStatus.RUNNING:
                # If the child is still running, then this node is also running
                return ExecutionStatus.RUNNING
            elif status == ExecutionStatus.SUCCESS:
                self.n = 0
                # If any child returns success, then this node must also declare success
                self.running_child = None
                return ExecutionStatus.SUCCESS


class RoombaBehaviorTree(BehaviorTree):
    """
    Represents a behavior tree of a roomba cleaning robot.
    """
    def __init__(self):
        # Todo: construct the tree here

        # Creating root node (Selector)
        selector_root = SelectorNode(node_name = "selector_root", parent = None, n = 0)

        # Creating left subtree's root node (Sequence)
        sequence_left = SequenceNode(node_name = "sequence_left", parent = selector_root, n = 0)
        # Creating left subtree's leaf nodes
        move_forward = MoveForwardNode(node_name = "move_forward", parent = sequence_left, n = 0)
        move_in_spiral = MoveInSpiralNode(node_name = "move_in_spiral", parent = sequence_left, n = 0)
        # Building left subtree structure
        sequence_left.add_child(move_forward)
        sequence_left.add_child(move_in_spiral)


        # Creating right subtree's root node (Sequence)
        sequence_right = SequenceNode(node_name = "sequence_right", parent = selector_root, n = 0)
        # Creating right subtree's leaf nodes
        go_back = GoBackNode(node_name = "go_back", parent = sequence_right, n = 0)
        rotate = RotateNode(node_name = "rotate", parent = sequence_right, n = 0)
        # Building right subtree structure
        sequence_right.add_child(go_back)
        sequence_right.add_child(rotate)

        # Building the entire tree's structure
        selector_root.add_child(sequence_left)
        selector_root.add_child(sequence_right)
        super().__init__(root = selector_root)


class MoveForwardNode(LeafNode):
    def __init__(self, status = NodeStatus.NOT_VISITED,  node_name = "move_forward", parent = None, n = 0):
        super().__init__(status = status, node_name = node_name, parent = parent, n = n)
        # Todo: add initialization code
        # Set sampling cycles counter "n" to "0"
        self.n = 0


    def enter(self, agent):
        # Todo: add enter logic
        # Set node status to VISITING
        self.status = NodeStatus.VISITING



    def execute(self, agent):
        # Todo: add execution logic

        # If leaf node is not being visited, raise error
        if self.status != NodeStatus.VISITING:
            raise ValueError("Error! Node status is not VISITING.")

        # Change velocity to move forwards
        agent.set_velocity(FORWARD_SPEED, 0)

        # Increment sampling cycles counter "n"
        self.n += 1
        # If a collision occurred
        if agent.get_bumper_state():
            agent.set_velocity(0, 0)
            self.n = 0
            return ExecutionStatus.FAILURE
        # If no collision occurred but the movement is not finished
        elif 0 <= self.n < n2:
            return ExecutionStatus.RUNNING
        # If the roomba was able to execute the entire movement without colliding, return SUCCESS
        elif self.n >= n2:
            self.n = 0
            self.status = NodeStatus.VISITED
            return ExecutionStatus.SUCCESS
        # If the counter "n" has an invalid value
        else:
            raise ValueError("Error! Sampling cycles counter \"n\" is invalid.")


class MoveInSpiralNode(LeafNode):
    def __init__(self, status = NodeStatus.NOT_VISITED, node_name = "move_in_spiral", parent = None, n = 0):
        super().__init__(status = status, node_name = node_name, parent = parent, n = n)
        # Todo: add initialization code
        # Set sampling cycles counter "n" to "0"
        self.n = 0


    def enter(self, agent):
        # Todo: add enter logic
        # Set node status to VISITING
        self.status = NodeStatus.VISITING


    def execute(self, agent):
        # Todo: add execution logic

        # If leaf node is not being visited, raise error
        if self.status != NodeStatus.VISITING:
            raise ValueError("Error! Node status is not VISITING.")

        # Instantaneous curvature radius
        r = r0 + b * self.n * SAMPLE_TIME
        # Change velocity to move in spiral
        agent.set_velocity(FORWARD_SPEED, FORWARD_SPEED / r)

        # Increment sampling cycles counter "n"
        self.n += 1
        # If a collision occurred
        if agent.get_bumper_state():
            self.n = 0
            agent.set_velocity(0, 0)
            return ExecutionStatus.FAILURE
        # If no collision occurred but the movement is not finished
        elif 0 <= self.n < n1:
            return ExecutionStatus.RUNNING
        # If the roomba was able to execute the entire movement without colliding, return SUCCESS
        elif self.n >= n1:
            self.n = 0
            self.status = NodeStatus.VISITED
            return ExecutionStatus.SUCCESS
        # If the counter "n" has an invalid value
        else:
            raise ValueError("Error! Sampling cycles counter \"n\" is invalid.")


class GoBackNode(LeafNode):
    def __init__(self, status = NodeStatus.NOT_VISITED, node_name = "go_back", parent = None, n = 0):
        super().__init__(status = status, node_name = node_name, parent = parent, n = n)
        # Todo: add initialization code
        # Set sampling cycles counter "n" to "0"
        self.n = 0


    def enter(self, agent):
        # Todo: add enter logic
        # Set node status to VISITING
        self.status = NodeStatus.VISITING


    def execute(self, agent):
        # Todo: add execution logic

        # If leaf node is not being visited, raise error
        if self.status != NodeStatus.VISITING:
            raise ValueError("Error! Node status is not VISITING.")

        # Change velocity to move backwards
        if int(self.n) == 0:
            agent.set_velocity(BACKWARD_SPEED, 0)
            agent.set_bumper_state(False)

        # Increment sampling cycles counter "n"
        self.n += 1
        # If the roomba was able to execute the entire movement or if a collision happened, return SUCCESS
        #
        # *************************************************************************************************
        # * OBS: the reason for we to return SUCCESS even if a collision happened is that, if the roomba  *
        # * collides while going back, the next leaf node executed is "RotateNode", which saves the       *
        # * robot from becoming trapped in an endless loop of executing only the leaf nodes               *
        # * "MoveForwardNode" and "GoBackNode"                                                            *
        # *************************************************************************************************
        if agent.get_bumper_state() or self.n >= n3:
            self.n = 0
            self.status = NodeStatus.VISITED
            return ExecutionStatus.SUCCESS
        # If no collision occurred but the movement is not finished
        elif 0 <= self.n < n3:
            return ExecutionStatus.RUNNING
        # If none of the above options is entered
        else:
            raise ValueError("Error! Did not enter in any of the options.")


class RotateNode(LeafNode):
    def __init__(self, status = NodeStatus.NOT_VISITED, node_name = "rotate", parent = None, n = 0, delta_rotation = math.pi * random.uniform(-1, 1)):
        super().__init__(status = status, node_name = node_name, parent = parent, n = n)
        # Todo: add initialization code
        self.delta_rotation = main_phase(delta_rotation)
        # Set sampling cycles counter "n" to "0"
        self.n = 0


    def enter(self, agent):
        # Todo: add enter logic
        # Set node status to VISITING
        self.status = NodeStatus.VISITING
        # Choosing a new random value for "delta_rotation"
        self.delta_rotation = math.pi * random.uniform(-1, 1)


    def execute(self, agent):
        # Todo: add execution logic

        # If leaf node is not being visited, raise error
        if self.status != NodeStatus.VISITING:
            raise ValueError("Error! Node status is not VISITING.")

        agent.set_bumper_state(False)

        # Changing the roomba's velocity to make it rotate, ***IN THE SHORTEST AMOUNT OF TIME***, to the
        # desired value of "agent.pose.inclination"
        if self.delta_rotation >= 0:
            # Spin clockwise (positive w)
            agent.set_velocity(0, ANGULAR_SPEED)
        else:
            # Spin counterclockwise (negative w)
            agent.set_velocity(0, -ANGULAR_SPEED)

        # Increment sampling cycles counter "n"
        self.n += 1
        # If a collision occurred
        if agent.get_bumper_state():
            raise RuntimeError("Error! Roomba collided while rotating.")
        # If no collision occurred but the movement is not finished
        elif self.n < math.ceil(math.fabs(self.delta_rotation / (ANGULAR_SPEED * SAMPLE_TIME))):
            return ExecutionStatus.RUNNING
        # If the roomba was able to execute the entire movement without colliding, return SUCCESS
        elif self.n >= math.ceil(math.fabs(self.delta_rotation / (ANGULAR_SPEED * SAMPLE_TIME))):
            self.n = 0
            self.status = NodeStatus.VISITED
            return ExecutionStatus.SUCCESS
        else:
            raise RuntimeError("Error! Did not enter in any of the conditions.")

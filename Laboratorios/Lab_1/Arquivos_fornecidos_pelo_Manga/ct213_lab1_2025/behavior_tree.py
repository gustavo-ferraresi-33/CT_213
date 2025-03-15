from enum import Enum
from constants import *
import random
import math

import inspect

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

        # # DEBUGGING: printing leaf node being executed
        # print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)


class CompositeNode(TreeNode):
    """
    Represents a composite node of a behavior tree.
    """
    def __init__(self, status = None, node_name = "No name attributed", parent = None, n = 0):
        super().__init__(status = status, node_name = node_name, parent = parent, n = n)
        self.children = []

        # # DEBUGGING: printing leaf node being executed
        # print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)


    def add_child(self, child):
        """
        Adds a child to this composite node.

        :param child: child to be added to this node.
        :type child: TreeNode
        """

        # DEBUGGING: printing leaf node being executed
        print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

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

        # DEBUGGING: printing leaf node being executed
        print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)


    def enter(self, agent = None):
        # When this node is entered, no child should be running
        self.running_child = None

        self.status = NodeStatus.VISITING

        # DEBUGGING: printing leaf node being executed
        print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)


    def execute(self, agent):

        # DEBUGGING: printing leaf node being executed
        print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

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

        # DEBUGGING: printing leaf node being executed
        print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)


    def enter(self, agent = None):
        # DEBUGGING: printing leaf node being executed
        print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)
        # When this node is entered, no child should be running
        self.running_child = None
        self.status = NodeStatus.VISITING


    def execute(self, agent):

        # DEBUGGING: printing leaf node being executed
        print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

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

        # DEBUGGING: printing leaf node being executed
        print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        # Creating root node (Selector)
        selector_root = SelectorNode(node_name = "selector_root", parent = None, n = 0)

        # Creating left subtree's root node (Sequence)
        sequence_left = SequenceNode(node_name = "sequence_left", parent = selector_root, n = 0)
        # Creating left subtree's leaf nodes
        move_forward = MoveForwardNode(node_name = "move_forward", parent = sequence_left, n = 0)
        move_in_spiral = MoveInSpiralNode(node_name = "move_in_spiral", parent = sequence_left, n = 0)
        # Building left subtree
        sequence_left.add_child(move_forward)
        sequence_left.add_child(move_in_spiral)


        # Creating right subtree's root node (Sequence)
        sequence_right = SequenceNode(node_name = "sequence_right", parent = selector_root, n = 0)
        # Creating right subtree's leaf nodes
        go_back = GoBackNode(node_name = "go_back", parent = sequence_right, n = 0)
        rotate = RotateNode(node_name = "rotate", parent = sequence_right, n = 0)
        # Building right subtree
        sequence_right.add_child(go_back)
        sequence_right.add_child(rotate)

        # Building the entire tree
        selector_root.add_child(sequence_left)
        selector_root.add_child(sequence_right)

        # DEBUGGING: checking if the behavior tree was constructed correctly
        def print_node_info(debug_node):
            print(2 * "\n" + 40 * "-" + "\n", "Node under analysis", debug_node.node_name)

            if debug_node.parent is not None:
                print("Parent of node", debug_node.node_name, "is", debug_node.parent.node_name)

            if hasattr(debug_node, "children"):
                print("Children of node", debug_node.node_name, "are", [child.node_name for child in debug_node.children])

            print("Node status of node", debug_node.node_name, "is", debug_node.status)
            print("Sampling cycles counter of node", debug_node.node_name, "is n ==", debug_node.n)

        print_node_info(selector_root)
        print_node_info(sequence_left)
        print_node_info(sequence_right)
        print_node_info(move_forward)
        print_node_info(move_in_spiral)
        print_node_info(go_back)
        print_node_info(rotate)
        print(4 * "\n")

        super().__init__(root = selector_root)


class MoveForwardNode(LeafNode):
    def __init__(self, status = NodeStatus.NOT_VISITED,  node_name = "move_forward", parent = None, n = 0):
        super().__init__(status = status, node_name = node_name, parent = parent, n = n)
        # Todo: add initialization code

        # DEBUGGING: printing leaf node being executed
        print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        # Set sampling cycles counter "n" to "0"
        self.n = 0




    def enter(self, agent):
        # Todo: add enter logic

        # DEBUGGING: printing leaf node being executed
        print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        # Set node status to VISITING
        self.status = NodeStatus.VISITING



    def execute(self, agent):
        # Todo: add execution logic

        # DEBUGGING: printing leaf node being executed
        print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        # If leaf node is not being visited, raise error
        if self.status != NodeStatus.VISITING:
            raise ValueError("Error! Node status is not VISITING.")

        # Change velocity to move forwards
        agent.set_velocity(FORWARD_SPEED, 0)
        # DUVIDA: call "check_transition()" ?

        # Increment sampling cycles counter "n"
        self.n += 1
        # If a collision occurred
        if agent.get_bumper_state():
            print(type(self).__name__, "returned", ExecutionStatus.FAILURE)
            agent.set_velocity(0, 0)
            self.n = 0
            return ExecutionStatus.FAILURE
        # If no collision occurred but the movement is not finished
        elif 0 <= self.n < n2:
            print(type(self).__name__, "returned", ExecutionStatus.RUNNING)
            return ExecutionStatus.RUNNING
        # If the roomba was able to execute the entire movement without colliding, return SUCCESS
        elif self.n >= n2:
            print(type(self).__name__, "returned", ExecutionStatus.SUCCESS)
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

        # DEBUGGING: printing leaf node being executed
        print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        # Set sampling cycles counter "n" to "0"
        self.n = 0


    def enter(self, agent):
        # Todo: add enter logic

        # DEBUGGING: printing leaf node being executed
        print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        # Set node status to VISITING
        self.status = NodeStatus.VISITING


    def execute(self, agent):
        # Todo: add execution logic

        # DEBUGGING: printing leaf node being executed
        print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        # If leaf node is not being visited, raise error
        if self.status != NodeStatus.VISITING:
            raise ValueError("Error! Node status is not VISITING.")

        # Instantaneous curvature radius
        r = r0 + b * self.n * SAMPLE_TIME
        # Change velocity to move in spiral
        agent.set_velocity(FORWARD_SPEED, FORWARD_SPEED / r)
        # DUVIDA: call "check_transition()" ?

        # Increment sampling cycles counter "n"
        self.n += 1
        # If a collision occurred
        if agent.get_bumper_state():
            print(type(self).__name__, "returned", ExecutionStatus.FAILURE)
            self.n = 0
            agent.set_velocity(0, 0)
            return ExecutionStatus.FAILURE
        # If no collision occurred but the movement is not finished
        elif 0 <= self.n < n1:
            print(type(self).__name__, "returned", ExecutionStatus.RUNNING)
            return ExecutionStatus.RUNNING
        # If the roomba was able to execute the entire movement without colliding, return SUCCESS
        elif self.n >= n1:
            print(type(self).__name__, "returned", ExecutionStatus.SUCCESS)
            self.n = 0
            self.status = NodeStatus.VISITED
            return ExecutionStatus.SUCCESS
        # If the counter "n" has an invalid value
        else:
            raise ValueError("Error! Sampling cycles counter \"n\" is invalid.")


class GoBackNode(LeafNode):
    def __init__(self, status = NodeStatus.NOT_VISITED, node_name = "go_back", parent = None, n = 0):
        super().__init__(status = status, node_name = node_name, parent = parent, n = n)

        # DEBUGGING: printing leaf node being executed
        print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        # Todo: add initialization code
        # Set sampling cycles counter "n" to "0"
        self.n = 0


    def enter(self, agent):
        # Todo: add enter logic

        # DEBUGGING: printing leaf node being executed
        print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        # Set node status to VISITING
        self.status = NodeStatus.VISITING


    def execute(self, agent):
        # Todo: add execution logic

        # DEBUGGING: printing leaf node being executed
        print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        # If leaf node is not being visited, raise error
        if self.status != NodeStatus.VISITING:
            raise ValueError("Error! Node status is not VISITING.")



        # Change velocity to move backwards
        if int(self.n) == 0:
            agent.set_velocity(BACKWARD_SPEED, 0)
            agent.set_bumper_state(False)
        # DUVIDA: call "check_transition()" ?

        # Increment sampling cycles counter "n"
        self.n += 1
        # If a collision occurred
        if agent.get_bumper_state():
            print(type(self).__name__, "returned", ExecutionStatus.FAILURE)
            self.n = 0
            return ExecutionStatus.FAILURE
        # If no collision occurred but the movement is not finished
        elif 0 <= self.n < n3:
            print(type(self).__name__, "returned", ExecutionStatus.RUNNING)
            return ExecutionStatus.RUNNING
        # If the roomba was able to execute the entire movement without colliding, return SUCCESS
        elif self.n >= n3:
            print(type(self).__name__, "returned", ExecutionStatus.SUCCESS)
            self.n = 0
            self.status = NodeStatus.VISITED
            return ExecutionStatus.SUCCESS
        # If the counter "n" has an invalid value
        else:
            raise ValueError("Error! Sampling cycles counter \"n\" is invalid.")




class RotateNode(LeafNode):
    def __init__(self, status = NodeStatus.NOT_VISITED, node_name = "rotate", parent = None, n = 0, delta_rotation = math.pi * random.uniform(-1, 1)):
        super().__init__(status = status, node_name = node_name, parent = parent, n = n)
        self.delta_rotation = delta_rotation
        # Todo: add initialization code

        # DEBUGGING: printing leaf node being executed
        print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        # Set sampling cycles counter "n" to "0"
        self.n = 0


    def enter(self, agent):
        # Todo: add enter logic

        # DEBUGGING: printing leaf node being executed
        print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        # Set node status to VISITING
        self.status = NodeStatus.VISITING
        # Choosing a new random value for "delta_rotation"
        self.delta_rotation = math.pi * random.uniform(-1, 1)
        # Make both roomba's velocities equal to zero
        agent.set_velocity(0, 0)



    def execute(self, agent):
        # Todo: add execution logic

        # DEBUGGING: printing leaf node being executed
        print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        # If leaf node is not being visited, raise error
        if self.status != NodeStatus.VISITING:
            raise ValueError("Error! Node status is not VISITING.")

        agent.set_bumper_state(False)

        # DEBUGGING
        print(type(self).__name__ + ": self.delta_rotation ==", self.delta_rotation)
        # DEBUGGING
        print(type(self).__name__ + ": agent.pose.rotation ==", agent.pose.rotation)
        # DEBUGGING
        print(type(self).__name__ + ": \"n\" ==", self.n)
        # DEBUGGING
        print(type(self).__name__ + ": number of sampling cycles to run to complete rotation ==",
              math.ceil(math.fabs(self.delta_rotation / (ANGULAR_SPEED * SAMPLE_TIME))),
        )

        if self.delta_rotation >= 0:
            # Spin in horary sense (positive w)
            agent.set_velocity(0, ANGULAR_SPEED)
        else:
            # Spin in anti-horary sense (negative w)
            agent.set_velocity(0, -ANGULAR_SPEED)



        # DUVIDA: call "check_transition()" ?


        # Increment sampling cycles counter "n"
        self.n += 1
        # If a collision occurred
        if agent.get_bumper_state():
            raise RuntimeError("Error! Roomba collided while rotating.")
        # If no collision occurred but the movement is not finished
        elif self.n < math.ceil(math.fabs(self.delta_rotation / (ANGULAR_SPEED * SAMPLE_TIME))):
            # DEBUGGING
            print(type(self).__name__, "returned", ExecutionStatus.RUNNING)
            return ExecutionStatus.RUNNING
        # If the roomba was able to execute the entire movement without colliding, return SUCCESS
        elif self.n >= math.ceil(math.fabs(self.delta_rotation / (ANGULAR_SPEED * SAMPLE_TIME))):
            print(type(self).__name__, "returned", ExecutionStatus.SUCCESS)
            self.n = 0
            self.status = NodeStatus.VISITED
            return ExecutionStatus.SUCCESS
        else:
            raise RuntimeError("Error! Did not enter in any of the conditions.")



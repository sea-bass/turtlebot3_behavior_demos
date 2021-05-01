"""
Custom py_trees decorators
Note that some of these are taken from later versions (ROS 2 compatible) of the source code
"""

from py_trees import behaviour
from py_trees import common
from py_trees.decorators import Decorator


class Loop(Decorator):
    """
    A py_trees decorator for looping a behavior until successful or a 
    maximum number of iterations have been reached
    """
    def __init__(self, child, max_iters=3,
                 name=common.Name.AUTO_GENERATED):
        """
        Init with the decorated child.
                
        Args:
            child (:class:`~py_trees.behaviour.Behaviour`): behaviour to time
            max_iters (:obj:`int`): the maximum number of iterations
            name (:obj:`str`): the decorator name
        """
        super().__init__(name=name, child=child)
        self.max_iters = max_iters

    def initialise(self):
        self.iters = 0
    
    def update(self):
        """
        If the child behavior has failed, move on to the next iteration until 
        the maximum number has been reached. Else, keep ticking the child behavior.
        """
        if self.decorated.status == common.Status.FAILURE:
            self.iters += 1
            self.logger.info(f"Loop Failure Count: {self.iters}")
            if self.iters >= self.max_iters:
                self.logger.info(f"Exceeded {self.max_iters} failures of {self.decorated.name}")
                return common.Status.FAILURE
            return common.Status.RUNNING
        else:
            return self.decorated.status


class OneShot(Decorator):
    """
    Modified version of a decorator that implements the oneshot pattern.

    This decorator ensures that the underlying child is ticked through
    to completion just once and while doing so, will return
    with the same status as it's child. Thereafter it will return
    with the final status of the underlying child.

    Completion status is determined by the policy given on construction.

    * With policy :data:`~py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION`, the oneshot will activate only when the underlying child returns :data:`~py_trees.common.Status.SUCCESS` (i.e. it permits retries).
    * With policy :data:`~py_trees.common.OneShotPolicy.ON_COMPLETION`, the oneshot will activate when the child returns :data:`~py_trees.common.Status.SUCCESS` || :data:`~py_trees.common.Status.FAILURE`.

    .. seealso:: :meth:`py_trees.idioms.oneshot`
    """
    def __init__(self, child,
                 name=common.Name.AUTO_GENERATED,
                 on_successful_completion=False):
        """
        Init with the decorated child.

        Args:
            name (:obj:`str`): the decorator name
            child (:class:`~py_trees.behaviour.Behaviour`): behaviour to time
            on_successful_completion (:obj:`bool`) determining when the oneshot should activate
        """
        super(OneShot, self).__init__(name=name, child=child)
        self.final_status = None
        if on_successful_completion:
            self.policy_values = [common.Status.SUCCESS]
        else:
            self.policy_values = [common.Status.SUCCESS, common.Status.FAILURE]


    def update(self):
        """
        Bounce if the child has already successfully completed.
        """
        if self.final_status:
            self.logger.debug("{}.update()[bouncing]".format(self.__class__.__name__))
            return self.final_status
        return self.decorated.status


    def tick(self):
        """
        Select between decorator (single child) and behaviour (no children) style
        ticks depending on whether or not the underlying child has been ticked
        successfully to completion previously.
        """
        if self.final_status:
            # ignore the child
            for node in behaviour.Behaviour.tick(self):
                yield node
        else:
            # tick the child
            for node in Decorator.tick(self):
                yield node


    def terminate(self, new_status):
        """
        If returning :data:`~py_trees.common.Status.SUCCESS` for the first time,
        flag it so future ticks will block entry to the child.
        """
        if not self.final_status and new_status in self.policy_values:
            self.logger.debug("{}.terminate({})[oneshot completed]".format(self.__class__.__name__, new_status))
            self.feedback_message = "oneshot completed"
            self.final_status = new_status
        else:
            self.logger.debug("{}.terminate({})".format(self.__class__.__name__, new_status))

from pr2_pbd_interaction.msg import ExecuteAction
from pr2_pbd_interaction.msg import ExecuteFeedback
from pr2_pbd_interaction.msg import ExecuteResult
from pr2_pbd_interaction.msg import ExecutionStatus
from response import Response
import actionlib
from robot_speech import RobotSpeech
import rospy


class ExecuteActionServer(object):
    def __init__(self, interaction, action_db):
        """Initialize this server with the interaction class.

        Args:
            interaction: Interaction class to use.
            action_db: The ActionDatabase to use.
        """
        self._server = actionlib.SimpleActionServer('execute_pbd_action',
                                                    ExecuteAction,
                                                    execute_cb=self._execute,
                                                    auto_start=False)
        self._server.register_preempt_callback(self._preempt)
        self._interaction = interaction
        self._action_db = action_db

    def start(self):
        self._server.start()

    def _execute(self, goal):
        """Callback for serving Execute action requests.
        """
        action_id = None
        if goal.action_id != '':
            action_id = goal.action_id
        else:
            action_id = self._action_db.id_for_name(goal.name)
        self._interaction.switch_to_action_by_id(action_id)
        response_params = self._interaction._execute_action(
            preregistered_landmarks=goal.landmarks)
        if RobotSpeech.START_EXECUTION not in response_params[0]:
            result = ExecuteResult()
            result.error = response_params[0]
            self._server.set_aborted(result=result, text=result.error)
            response = Response(self._interaction._empty_response,
                                response_params)
            response.respond()
            return

        # Wait for it to start executing for at most 10 seconds
        rate = rospy.Rate(10)
        start = rospy.Time.now()
        timeout = rospy.Duration(10)
        while not self._interaction.arms.is_executing():
            elapsed_time = rospy.Time.now() - start
            if elapsed_time > timeout:
                rospy.logwarn('PbD action did not start after 10 seconds')
                result = ExecuteResult()
                result.error = 'PbD action did not start after 10 seconds'
                self._server.set_aborted(result=result, text=result.error)
                break
            rate.sleep()

        response = Response(self._interaction._empty_response, response_params)
        response.respond()
        rate = rospy.Rate(10)
        start = rospy.Time.now()
        timeout = rospy.Duration(60 * 5)
        while self._interaction.arms.is_executing():
            elapsed_time = rospy.Time.now() - start
            self._server.publish_feedback(ExecuteFeedback())
            if elapsed_time > timeout:
                rospy.logwarn('PbD action did not finish after 5 minutes')
                result = ExecuteResult()
                result.error = 'PbD action did not finish after 5 minutes'
                self._server.set_aborted(result=result, text=result.error)
                break
            rate.sleep()

        if self._interaction.arms.status == ExecutionStatus.SUCCEEDED:
            self._server.set_succeeded()
        elif self._interaction.arms.status == ExecutionStatus.NOT_EXECUTING:
            # Race condition? Assume this means success.
            self._server.set_succeeded()
        elif self._interaction.arms.status == ExecutionStatus.PREEMPTED:
            error = 'The PbD action was preempted.'
            result = ExecuteResult()
            result.error = error
            self._server.set_aborted(result=result, text=error)
        elif self._interaction.arms.status == ExecutionStatus.NO_IK:
            error = 'The robot\'s arms couldn\'t reach some poses.'
            result = ExecuteResult()
            result.error = error
            self._server.set_aborted(result=result, text=error)
        else:
            error = 'Unknown error {} running the PbD action'.format(
                self._interaction.arms.status)
            result = ExecuteResult()
            result.error = error
            self._server.set_aborted(result=result, text=error)

    def _preempt(self):
        if self._interaction.arms.is_executing():
            self._interaction.arms.stop_execution()

from pr2_pbd_interaction.msg import ExecuteAction
from pr2_pbd_interaction.msg import ExecuteFeedback
from pr2_pbd_interaction.msg import ExecuteResult
from response import Response
import actionlib
from robot_speech import RobotSpeech
import rospy


class ExecuteActionServer(object):
    def __init__(self, interaction):
        """Initialize this server with the interaction class.

        Args:
            interaction: Interaction class to use.
        """
        self._server = actionlib.SimpleActionServer('execute_pbd_action',
                                                    ExecuteAction,
                                                    execute_cb=self._execute,
                                                    auto_start=False)
        self._server.register_preempt_callback(self._preempt)
        self._interaction = interaction

    def start(self):
        self._server.start()

    def _execute(self, goal):
        """Callback for serving Execute action requests.
        """
        self._interaction.switch_to_action_by_id(goal.action_id)
        response_params = self._interaction._execute_action()
        if RobotSpeech.START_EXECUTION not in response_params[0]:
            result = ExecuteResult()
            result.error = response_params[0]
            self._server.set_aborted(result=result, text=result.error)

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

        self._server.set_succeeded()

    def _preempt(self):
        if self._interaction.arms.is_executing():
            self._interaction.arms.stop_execution();

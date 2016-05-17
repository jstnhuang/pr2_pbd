from pr2_pbd_interaction.srv import ExecuteActionById, ExecuteActionByIdResponse


class ExecuteActionServer(object):
    def __init__(self, interaction):
        """Initialize this server with the interaction class.

        Args:
            interaction: Interaction class to use.
        """
        self._interaction = interaction

    def serve(self, request):
        """Callback for serving ExecuteActionById requests.
        """
        self._interaction.switch_to_action_by_id(request.action_id)
        self._interaction._execute_action()
        response = ExecuteActionByIdResponse()
        return response

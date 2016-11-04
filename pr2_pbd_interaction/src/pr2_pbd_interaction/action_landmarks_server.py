"""A server to get a list of custom landmarks for a given PbD action.
"""
from pr2_pbd_interaction.srv import GetLandmarksForActionResponse
import programmed_action


class ActionLandmarksServer(object):
    def __init__(self, action_db):
        """Construct this server.

        Args:
            action_db: an ActionDatabase, used to retrieve actions by MongoDB ID.
        """
        self._action_db = action_db

    def serve(self, req):
        action_id = req.action_id
        if req.action_id is None or req.action_id == '':
            action_id = self._action_db.id_for_name(self, req.name)

        action = self._action_db.find(action_id)
        if action is None:
            response = GetLandmarksForActionResponse()
            return response
        custom_landmarks = programmed_action.custom_landmarks_from_sequence(
            action.sequence)
        response = GetLandmarksForActionResponse()
        response.landmarks = custom_landmarks
        return response

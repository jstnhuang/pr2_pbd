#!/usr/bin/env python
'''This runs the PbD system (i.e. the backend).'''

from pymongo import MongoClient
from mongo_msg_db import MessageDb
from static_cloud_db import StaticCloudDb
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from object_search_msgs.srv import RecordObject
from object_search_msgs.srv import Search
from pr2_pbd_interaction import ActionDatabase
from pr2_pbd_interaction import ActionLandmarksServer
from pr2_pbd_interaction import Arms
from pr2_pbd_interaction import CustomLandmarkFinder
from pr2_pbd_interaction import ExecuteActionServer
from pr2_pbd_interaction import Interaction
from pr2_pbd_interaction import Session
from pr2_pbd_interaction import World
from pr2_pbd_interaction.srv import GetLandmarksForAction
from pr2_pbd_interaction.srv import ExecuteActionById
from tabletop_object_detector.srv import TabletopSegmentation
import pr2_pbd_interaction
import rospy
import signal
import tf


def signal_handler(signal, frame):
    # The following makes sure the state of a user study is saved, so that it can be recovered
    global interaction
    interaction.session.save_current_action()
    rospy.loginfo("Saved experiment state. Terminating.")
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGQUIT, signal_handler)

UPDATE_WAIT_SECONDS = 0.1

if __name__ == '__main__':
    global interaction
    # Register as a ROS node.
    rospy.init_node('pr2_pbd_interaction', anonymous=True)

    # Build world object
    tf_listener = tf.TransformListener()
    im_server = InteractiveMarkerServer('world_objects')
    rospy.wait_for_service('tabletop_segmentation', timeout=5)
    segment_tabletop = rospy.ServiceProxy('tabletop_segmentation',
                                          TabletopSegmentation)
    world = World(tf_listener, im_server, segment_tabletop)

    # Build session
    action_db = ActionDatabase.build_real()
    session = Session(world, world.get_frame_list(), action_db)

    # Build arms
    arms = Arms(tf_listener, world)

    # Build interaction
    # Build the static cloud DB for retrieving custom landmarks.
    mongo_client = MongoClient()
    mongo_db = MessageDb(mongo_client)
    static_cloud_db = StaticCloudDb(mongo_db)
    rospy.wait_for_service('find_object', timeout=5)
    find_landmark = rospy.ServiceProxy('find_object', Search)
    landmark_finder = CustomLandmarkFinder(static_cloud_db, "object_search",
                                           "objects", find_landmark,
                                           tf_listener)

    rospy.wait_for_service('record_object', timeout=5)
    capture_landmark = rospy.ServiceProxy('record_object', RecordObject)
    interaction = Interaction(arms, session, world, capture_landmark,
                              landmark_finder)

    # Server that outputs custom landmarks used in an action.
    action_landmarks_server = ActionLandmarksServer(action_db)
    get_landmarks_for_action = rospy.Service('pr2_pbd/landmarks_for_action',
                                             GetLandmarksForAction,
                                             action_landmarks_server.serve)

    # Actionlib server to execute PbD actions.
    execute_server = ExecuteActionServer(interaction, action_db)
    execute_server.start()

    while (not rospy.is_shutdown()):
        interaction.update()
        # This is the pause between update runs. Note that this doesn't
        # guarantee an update rate, only that there is this amount of
        # pause between udpates.
        rospy.sleep(UPDATE_WAIT_SECONDS)

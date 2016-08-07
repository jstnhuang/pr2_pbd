#!/usr/bin/env python
'''This runs the PbD system (i.e. the backend).'''

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from object_search_msgs.srv import RecordObject
from object_search_msgs.srv import Search
from pr2_pbd_interaction import ActionDatabase
from pr2_pbd_interaction import Arms
from pr2_pbd_interaction import ExecuteActionServer
from pr2_pbd_interaction import Interaction
from pr2_pbd_interaction import Session
from pr2_pbd_interaction import World
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
    tf_broadcaster = tf.TransformBroadcaster()
    tf_broadcaster = tf.TransformBroadcaster()
    im_server = InteractiveMarkerServer('world_objects')
    rospy.wait_for_service('tabletop_segmentation', timeout=5)
    segment_tabletop = rospy.ServiceProxy('tabletop_segmentation',
                                          TabletopSegmentation)
    world = World(tf_listener, tf_broadcaster, im_server, segment_tabletop)

    # Build session
    db = ActionDatabase.build_real()
    session = Session(world, world.get_frame_list(), db)

    # Build arms
    arms = Arms(tf_listener, world)

    # Build interaction
    rospy.wait_for_service('record_object', timeout=5)
    capture_landmark = rospy.ServiceProxy('record_object', RecordObject)
    rospy.wait_for_service('find_object', timeout=5)
    find_landmark = rospy.ServiceProxy('find_object', Search)
    interaction = Interaction(arms, session, world, capture_landmark, find_landmark)

    execute_server = ExecuteActionServer(interaction)
    rospy.Service('execute_action', ExecuteActionById, execute_server.serve)

    while (not rospy.is_shutdown()):
        interaction.update()
        # This is the pause between update runs. Note that this doesn't
        # guarantee an update rate, only that there is this amount of
        # pause between udpates.
        rospy.sleep(UPDATE_WAIT_SECONDS)

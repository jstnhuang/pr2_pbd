"""Contains the World class as well as geometry-related helper functions.
"""

import threading
import numpy as np
import rospy
import tf
from geometry_msgs.msg import Quaternion, Vector3, Point, Pose, PoseStamped
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from interactive_markers.menu_handler import MenuHandler
from actionlib_msgs.msg import GoalStatus
from pr2_pbd_interaction.msg import Landmark, ArmState
from pr2_pbd_interaction.response import Response
from pr2_social_gaze.msg import GazeGoal
from world_landmark import WorldLandmark

# Two objects must be closer than this to be considered 'the same'.
OBJ_SIMILAR_DIST_THRESHOLD = 0.075

# When adding objects, if they are closer than this they'll replace one
# another.
OBJ_ADD_DIST_THRESHOLD = 0.02

# How close to 'nearest' object something must be to be counted as
# 'near' it.
OBJ_NEAREST_DIST_THRESHOLD = 0.4

# Landmark distances below this will be clamped to zero.
OBJ_DIST_ZERO_CLAMP = 0.0001

# Scales
SCALE_TEXT = Vector3(0.0, 0.0, 0.03)
SURFACE_HEIGHT = 0.01  # 0.01 == 1cm (I think)
OFFSET_OBJ_TEXT_Z = 0.06  # How high objects' labels are above them.

# Colors
COLOR_OBJ = ColorRGBA(0.2, 0.8, 0.0, 0.6)
COLOR_SURFACE = ColorRGBA(0.8, 0.0, 0.4, 0.4)
COLOR_TEXT = ColorRGBA(0.0, 0.0, 0.0, 0.5)

# Frames
BASE_LINK = 'base_link'  # The robot's base frame

# Time
MARKER_DURATION = rospy.Duration(2)
# How long to pause when waiting for external code, like gaze actions or
# object segmentation, to finish before checking again.
PAUSE_SECONDS = rospy.Duration(0.1)


def get_pose_from_transform(transform):
    """Returns pose for transformation matrix.

    Args:
        transform (Matrix3x3): (I think this is the correct type.
            See ActionStepMarker as a reference for how to use.)

    Returns:
        Pose
    """
    pos = transform[:3, 3].copy()
    rot = tf.transformations.quaternion_from_matrix(transform)
    return Pose(Point(pos[0], pos[1], pos[2]),
                Quaternion(rot[0], rot[1], rot[2], rot[3]))


def get_matrix_from_pose(pose):
    """Returns the transformation matrix for given pose.

    Args:
        pose (Pose)

    Returns:
        Matrix3x3: (I think this is the correct type. See
            ActionStepMarker as a reference for how to use.)
    """
    pp, po = pose.position, pose.orientation
    rotation = [po.x, po.y, po.z, po.w]
    transformation = tf.transformations.quaternion_matrix(rotation)
    position = [pp.x, pp.y, pp.z]
    transformation[:3, 3] = position
    return transformation


def pose_distance(pose1, pose2, is_on_table=True):
    """Returns distance between two world poses.

    Args:
        pose1 (Pose)
        pose2 (Pose)
        is_on_table (bool, optional): Whether the objects are on the
            table (if so, disregards z-values in computations).

    Returns:
        float
    """
    if pose1 == [] or pose2 == []:
        return 0.0
    else:
        p1p = pose1.position
        p2p = pose2.position
        if is_on_table:
            arr1 = np.array([p1p.x, p1p.y])
            arr2 = np.array([p2p.x, p2p.y])
        else:
            arr1 = np.array([p1p.x, p1p.y, p1p.z])
            arr2 = np.array([p2p.x, p2p.y, p2p.z])
        dist = np.linalg.norm(arr1 - arr2)
        if dist < OBJ_DIST_ZERO_CLAMP:
            dist = 0
        return dist


def object_dissimilarity(obj1, obj2):
    """Returns distance between two objects.

    Returns:
        float
    """
    d1 = obj1.dimensions
    d2 = obj2.dimensions
    return np.linalg.norm(
        np.array([d1.x, d1.y, d1.z]) - np.array([d2.x, d2.y, d2.z]))


def get_ref_from_name(ref_name):
    """Returns the reference frame type from the reference frame
    name specified by ref_name.

    Args:
        ref_name (str): Name of a referene frame.

    Returns:
        int: One of ArmState.*, the number code of the reference
            frame specified by ref_name.
    """
    if ref_name == BASE_LINK:
        return ArmState.ROBOT_BASE
    else:
        return ArmState.OBJECT


def get_most_similar_obj(ref_object, ref_frame_list):
    """Finds the most similar object in the world.

    Args:
        ref_object (?)
        ref_frame_list ([Landmark]): List of objects (as defined by
            Landmark.msg).

    Returns:
        Landmark|None: As in one of Landmark.msg, or None if no object
            was found close enough.
    """
    best_dist = 10000  # Not a constant; an absurdly high number.
    chosen_obj = None
    for ref_frame in ref_frame_list:
        dist = object_dissimilarity(ref_frame, ref_object)
        if dist < best_dist:
            best_dist = dist
            chosen_obj = ref_frame
    if chosen_obj is None:
        rospy.loginfo('Did not find a similar object.')
    else:
        rospy.loginfo('Landmark dissimilarity is --- ' + str(best_dist))
        if best_dist > OBJ_SIMILAR_DIST_THRESHOLD:
            rospy.loginfo('Found some objects, but not similar enough.')
            chosen_obj = None
        else:
            rospy.loginfo(
                'Most similar to new object: ' + str(chosen_obj.name))

    # Regardless, return the "closest object," which may be None.
    return chosen_obj


def build_landmark_marker(landmark):
    """Generate and return a marker for world landmarks.

    Args:
        landmark (WorldLandmark): The landmark to generate a marker for.

    Returns:
        InteractiveMarker
    """
    int_marker = InteractiveMarker()
    int_marker.name = landmark.name()
    int_marker.header.frame_id = BASE_LINK
    int_marker.pose = landmark.object.pose
    int_marker.scale = 1

    button_control = InteractiveMarkerControl()
    button_control.interaction_mode = InteractiveMarkerControl.BUTTON
    button_control.always_visible = True

    object_marker = Marker(type=Marker.CUBE,
                           id=index,
                           lifetime=MARKER_DURATION,
                           scale=landmark.object.dimensions,
                           header=Header(frame_id=BASE_LINK),
                           color=COLOR_OBJ,
                           pose=landmark.object.pose)

    button_control.markers.append(object_marker)

    text_pos = Point()
    text_pos.x = landmark.object.pose.position.x
    text_pos.y = landmark.object.pose.position.y
    text_pos.z = (landmark.object.pose.position.z +
                  landmark.object.dimensions.z / 2 + OFFSET_OBJ_TEXT_Z)
    button_control.markers.append(
        Marker(type=Marker.TEXT_VIEW_FACING,
               id=index,
               scale=SCALE_TEXT,
               text=int_marker.name,
               color=COLOR_TEXT,
               header=Header(frame_id=BASE_LINK),
               pose=Pose(text_pos, Quaternion(0, 0, 0, 1))))
    int_marker.controls.append(button_control)
    return int_marker


def _get_surface_marker(pose, dimensions):
    """Returns a surface marker with provided pose and dimensions.

    Args:
        pose (Pose)
        dimensions  (Vector3)

    Returns:
        InteractiveMarker
    """
    int_marker = InteractiveMarker()
    int_marker.name = 'surface'
    int_marker.header.frame_id = BASE_LINK
    int_marker.pose = pose
    int_marker.scale = 1
    button_control = InteractiveMarkerControl()
    button_control.interaction_mode = InteractiveMarkerControl.BUTTON
    button_control.always_visible = True
    object_marker = Marker(type=Marker.CUBE,
                           id=2000,
                           lifetime=MARKER_DURATION,
                           scale=dimensions,
                           header=Header(frame_id=BASE_LINK),
                           color=COLOR_SURFACE,
                           pose=pose)
    button_control.markers.append(object_marker)
    text_pos = Point()
    position = pose.position
    dimensions = dimensions
    text_pos.x = position.x + dimensions.x / 2 - 0.06
    text_pos.y = position.y - dimensions.y / 2 + 0.06
    text_pos.z = position.z + dimensions.z / 2 + 0.06
    text_marker = Marker(type=Marker.TEXT_VIEW_FACING,
                         id=2001,
                         scale=SCALE_TEXT,
                         text=int_marker.name,
                         color=COLOR_TEXT,
                         header=Header(frame_id=BASE_LINK),
                         pose=Pose(text_pos, Quaternion(0, 0, 0, 1)))
    button_control.markers.append(text_marker)
    int_marker.controls.append(button_control)
    return int_marker


class World:
    """World maintains the list of landmarks (WorldLandmark) in the scene.

    It also provides helper functions for transforming poses relative to the
    scene. A third responsibility is that it updates the scene visualization.
    Note that landmarks are sometimes called objects or frames.

    FIRST RESPONSIBILITY
    To populate the object list, call:
        world.update_object_pose()
    This causes the robot to look down and segment the tabletop scene.
    The detected objects will be named "thing 0," "thing 1," and so on.
    This also causes the robot to broadcast the TF frame of each object.

    To get a list of WorldLandmarks:
        world_landmarks = world.get_frame_list()

    To check if there even are any landmarks currently:
        world.has_objects()

    To check if there is an object with a specific name:
        world.has_object('thing 1')

    'base_link' is a special frame that is assumed to always exist.
    To check if a frame ID is valid (either it is an object or it is the base
    link):
        world.is_frame_valid('thing 1')
        world.is_frame_valid('base_link')

    To delete all objects:
        world.clear_all_objects()

    To find the nearest object within 40 cm to an arm pose (provided in
    the base frame):
        obj = world.get_nearest_object(arm_pose)

    SECOND RESPONSIBILITY
    As mentioned, this class also has the responsibilities of computing
    transforms between arm poses and objects.

    To get the end-effector pose of an arm in the base link:
        base_pose = world.get_absolute_pose(arm_state)

    To transform an arm frame in general, use convert_ref_frame.
    To transform the arm state to be relative to the base:
        world.convert_ref_frame(arm_state, ArmState.ROBOT_BASE)

    To transform the arm state to be relative to a landmark:
        world.convert_ref_frame(arm_state, ArmState.OBJECT, landmark)

    You can also use a transform method directly, which only works for valid
    frames (according to is_frame_valid):
        from = 'base_link'
        to = 'thing 1'
        world.transform(pose, from, to)
    """

    def __init__(self, tf_listener, tf_broadcaster, im_server,
                 segment_tabletop):
        """Construct a World instance.

        Args:
            tf_listener: A tf.TransformListener
            tf_broadcaster: A tf.TransformBroadcaster
            im_server: An InteractiveMarkerServer for visualizing objects.
            segment_tabletop: A rospy.ServiceProxy for the tabletop
                segmentation service.
        """
        self._objects = []  # Type: [WorldLandmark]
        self._surface = None
        self._lock = threading.Lock()
        self._tf_listener = tf_listener
        self._tf_broadcaster = tf_broadcaster
        self._im_server = im_server
        self._segment_tabletop = segment_tabletop
        self.clear_all_objects()

    def get_absolute_pose(self, arm_state):
        """Returns absolute pose of an end effector state (trasnforming
        if relative).

        Args:
            arm_state (ArmState)

        Returns:
            Pose
        """
        if arm_state.refFrame == ArmState.OBJECT:
            arm_state_copy = ArmState(arm_state.refFrame,
                                      Pose(arm_state.ee_pose.position,
                                           arm_state.ee_pose.orientation),
                                      arm_state.joint_pose[:],
                                      arm_state.refFrameLandmark)
            self.convert_ref_frame(arm_state_copy, ArmState.ROBOT_BASE)
            return arm_state_copy.ee_pose
        else:
            return arm_state.ee_pose

    def get_frame_list(self):
        """Function that returns the list of reference frames (Landmarks).

        Returns:
            [Landmark]: List of Landmark (as defined by Landmark.msg), the
                current reference frames.
        """
        return [w_obj.object for w_obj in self._objects]

    def has_objects(self):
        """Returns whetehr there are any objects (reference frames).

        Returns:
            bool
        """
        return len(self._objects) > 0

    def convert_ref_frame(self, arm_frame, ref_frame,
                          ref_frame_obj=Landmark()):
        """Transforms an arm frame to a new ref. frame.

        Args:
            arm_frame (ArmState)
            ref_frame (int): One of ArmState.*
            ref_frame_obj (Landmark): As in Landmark.msg

        Returns:
            ArmState: arm_frame (passed in), but modified.
        """
        if ref_frame == ArmState.ROBOT_BASE:
            if arm_frame.refFrame == ArmState.ROBOT_BASE:
                # Transform from robot base to itself (nothing to do).
                rospy.logdebug(
                    'No reference frame transformations needed (both ' +
                    'absolute).')
            elif arm_frame.refFrame == ArmState.OBJECT:
                # Transform from object to robot base.
                abs_ee_pose = self.transform(arm_frame.ee_pose,
                                             arm_frame.refFrameLandmark.name,
                                             BASE_LINK)
                arm_frame.ee_pose = abs_ee_pose
                arm_frame.refFrame = ArmState.ROBOT_BASE
                arm_frame.refFrameLandmark = Landmark()
            else:
                rospy.logerr('Unhandled reference frame conversion: ' +
                             str(arm_frame.refFrame) + ' to ' + str(ref_frame))
        elif ref_frame == ArmState.OBJECT:
            if arm_frame.refFrame == ArmState.ROBOT_BASE:
                # Transform from robot base to object.
                rel_ee_pose = self.transform(arm_frame.ee_pose, BASE_LINK,
                                             ref_frame_obj.name)
                arm_frame.ee_pose = rel_ee_pose
                arm_frame.refFrame = ArmState.OBJECT
                arm_frame.refFrameLandmark = ref_frame_obj
            elif arm_frame.refFrame == ArmState.OBJECT:
                # Transform between the same object (nothign to do).
                if arm_frame.refFrameLandmark.name == ref_frame_obj.name:
                    rospy.logdebug(
                        'No reference frame transformations needed (same ' +
                        'object).')
                else:
                    # Transform between two different objects.
                    rel_ee_pose = self.transform(
                        arm_frame.ee_pose, arm_frame.refFrameLandmark.name,
                        ref_frame_obj.name)
                    arm_frame.ee_pose = rel_ee_pose
                    arm_frame.refFrame = ArmState.OBJECT
                    arm_frame.refFrameLandmark = ref_frame_obj
            else:
                rospy.logerr('Unhandled reference frame conversion: ' +
                             str(arm_frame.refFrame) + ' to ' + str(ref_frame))
        return arm_frame

    def has_object(self, object_name):
        """Returns whether the world contains an Landmark with object_name.

        Args:
            object_name (str)

        Returns:
            bool
        """
        return object_name in [wobj.object.name for wobj in self._objects]

    def is_frame_valid(self, object_name):
        """Returns whether the frame (object) name is valid for
        transforms.

        Args:
            object_name (str)

        Returns:
            bool
        """
        return object_name == BASE_LINK or self.has_object(object_name)

    def transform(self, pose, from_frame, to_frame):
        """Transforms a pose between two reference frames. If there is a
        TF exception or object does not exist, it will return the pose
        back without any transforms.

        Args:
            pose (Pose)
            from_frame (str)
            to_frame (str)

        Returns:
            Pose
        """
        if self.is_frame_valid(from_frame) and self.is_frame_valid(to_frame):
            pose_stamped = PoseStamped()
            try:
                common_time = self._tf_listener.getLatestCommonTime(from_frame,
                                                                    to_frame)
                pose_stamped.header.stamp = common_time
                pose_stamped.header.frame_id = from_frame
                pose_stamped.pose = pose
                rel_ee_pose = self._tf_listener.transformPose(to_frame,
                                                              pose_stamped)
                return rel_ee_pose.pose
            except tf.Exception:
                rospy.logerr('TF exception during transform.')
                return pose
            except rospy.ServiceException:
                rospy.logerr('ServiceException during transform.')
                return pose
        else:
            rospy.logdebug('One of the frame objects might not exist: ' +
                           from_frame + ' or ' + to_frame)
            return pose

    def update_object_pose(self):
        """ Function to externally update an object pose."""
        # Look down at the table.
        rospy.loginfo('Head attempting to look at table.')
        Response.force_gaze_action(GazeGoal.LOOK_DOWN)
        while (Response.gaze_client.get_state() == GoalStatus.PENDING or
               Response.gaze_client.get_state() == GoalStatus.ACTIVE):
            rospy.sleep(PAUSE_SECONDS)
        if Response.gaze_client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr('Could not look down to take table snapshot')
            return False
        rospy.loginfo('Head is now (successfully) staring at table.')

        try:
            resp = self._segment_tabletop()
            rospy.loginfo("Adding landmarks")

            self._reset_objects()

            # add the table
            xmin = resp.table.x_min
            ymin = resp.table.y_min
            xmax = resp.table.x_max
            ymax = resp.table.y_max
            depth = xmax - xmin
            width = ymax - ymin

            pose = resp.table.pose.pose
            pose.position.x = pose.position.x + xmin + depth / 2
            pose.position.y = pose.position.y + ymin + width / 2
            dimensions = Vector3(depth, width, 0.01)
            self._surface = _get_surface_marker(pose, dimensions)
            self._im_server.insert(self._surface, self.marker_feedback_cb)
            self._im_server.applyChanges()

            for cluster in resp.clusters:
                points = cluster.points
                if (len(points) == 0):
                    return Point(0, 0, 0)
                [minX, maxX, minY, maxY, minZ,
                 maxZ] = [points[0].x, points[0].x, points[0].y, points[0].y,
                          points[0].z, points[0].z]
                for pt in points:
                    minX = min(minX, pt.x)
                    minY = min(minY, pt.y)
                    minZ = min(minZ, pt.z)
                    maxX = max(maxX, pt.x)
                    maxY = max(maxY, pt.y)
                    maxZ = max(maxZ, pt.z)
                self._add_bounding_box_landmark(
                    Pose(Point((minX + maxX) / 2, (minY + maxY) / 2,
                               (minZ + maxZ) / 2), Quaternion(0, 0, 0, 1)),
                    Point(maxX - minX, maxY - minY, maxZ - minZ))
            return True

        except rospy.ServiceException, e:
            print "Call to segmentation service failed: %s" % e
            return False

    def add_landmark(self, landmark):
        self._objects.append(landmark)
        landmark.int_marker = self.build_landmark_marker(landmark)
        self._im_server.insert(landmark.int_marker, self.marker_feedback_cb)
        landmark.menu_handler.apply(self._im_server, landmark.int_marker.name)
        self._im_server.applyChanges()

    def clear_all_objects(self):
        """Removes all objects from the world."""
        self._reset_objects()
        self._remove_surface()

    def get_nearest_object(self, arm_pose):
        """Returns the nearest object, if one exists.

        Args:
            arm_pose (Pose): End-effector pose.

        Returns:
            Landmark|None: As in Landmark.msg, the nearest object (if it
                is close enough), or None if there were none close
                enough.
        """
        # First, find which object is the closest.
        distances = []
        for wobj in self._objects:
            dist = pose_distance(wobj.object.pose, arm_pose)
            distances.append(dist)

        # Then, see if the closest is actually below our threshold for
        # a 'closest object.'
        if len(distances) > 0:
            if min(distances) < OBJ_NEAREST_DIST_THRESHOLD:
                chosen = distances.index(min(distances))
                return self._objects[chosen].object

        # We didn't have any objects or none were close enough.
        return None

    def marker_feedback_cb(self, feedback):
        """Callback for when feedback from a marker is received.

        Args:
            feedback (InteractiveMarkerFeedback)
        """
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.loginfo('Clicked on object ' + str(feedback.marker_name))
            rospy.loginfo('Number of objects ' + str(len(self._objects)))
        else:
            # This happens a ton, and doesn't need to be logged like
            # normal events (e.g. clicking on most marker controls
            # fires here).
            rospy.logdebug('Unknown event: ' + str(feedback.event_type))

    def update(self):
        """Update function called in a loop.

        Returns:
            bool: Whether any tracked objects were removed, AKA "is
                world changed."
        """
        # Visualize the detected object
        is_world_changed = False
        self._lock.acquire()
        if self.has_objects():
            to_remove = None
            for i in range(len(self._objects)):
                self._publish_tf_pose(self._objects[i].object.pose,
                                      self._objects[i].name(), BASE_LINK)
                if self._objects[i].is_removed:
                    to_remove = i
            if to_remove is not None:
                self._remove_object(to_remove)
                is_world_changed = True

        self._lock.release()
        return is_world_changed

    # ##################################################################
    # Instance methods: Internal ("private")
    # ##################################################################

    def _reset_objects(self):
        """Removes all objects."""
        self._lock.acquire()
        for wobj in self._objects:
            self._im_server.erase(wobj.int_marker.name)
            self._im_server.applyChanges()
        if self._surface is not None:
            self._remove_surface()
        self._im_server.clear()
        self._im_server.applyChanges()
        self._objects = []
        self._lock.release()

    def _add_bounding_box_landmark(self, pose, dimensions):
        """Maybe add a new object with the specified properties to our
        object list.

        It might not be added if too similar of an object already
        exists (and has been added).

        Args:
            pose (Pose)
            dimensions (Vector3)
            is_recognized (bool)
            mesh (Mesh, optional): A mesh, if it exists. Default is
                None.

        Returns:
            bool: Whether the object was actually added.
        """
        # Whether whether we already have an object at ~ the same
        # location (and if so, don't add).
        for wobj in self._objects:
            if (pose_distance(wobj.object.pose, pose) <
                OBJ_ADD_DIST_THRESHOLD):
                rospy.loginfo(
                    'Previously detected object at the same location, ' +
                    'will not add this object.')
                return False

        # Actually add the object.
        name = 'thing {}'.format(len(self._objects))
        landmark = WorldLandmark.bounding_box(name, pose, dimensions)
        self._objects.append(landmark)
        landmark.int_marker = self.build_landmark_marker(landmark)
        self._im_server.insert(landmark.int_marker, self.marker_feedback_cb)
        landmark.menu_handler.apply(self._im_server, landmark.int_marker.name)
        self._im_server.applyChanges()
        return True

    def _remove_object(self, to_remove):
        """Remove an object by index.

        Args:
            to_remove (int): Index of the object to remove in
                self._objects.
        """
        obj = self._objects.pop(to_remove)
        rospy.loginfo('Removing object ' + obj.int_marker.name)
        self._im_server.erase(obj.int_marker.name)
        self._im_server.applyChanges()

    def _remove_surface(self):
        """Function to request removing surface (from IM)."""
        rospy.loginfo('Removing surface')
        self._im_server.erase('surface')
        self._im_server.applyChanges()
        self._surface = None

    def _publish_tf_pose(self, pose, name, parent):
        """ Publishes a TF for object named name with pose pose and
        parent reference frame parent.

        Args:
            pose (Pose): The object's pose.
            name (str): The object's name.
            parent (str): The parent reference frame.
        """
        if pose is not None:
            pp = pose.position
            po = pose.orientation
            pos = (pp.x, pp.y, pp.z)
            rot = (po.x, po.y, po.z, po.w)
            # TODO(mbforbes): Is it necessary to change the position
            # and orientation into tuples to send to TF?
            self._tf_broadcaster.sendTransform(pos, rot, rospy.Time.now(),
                                               name, parent)

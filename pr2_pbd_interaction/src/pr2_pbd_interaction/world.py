"""Contains the World class as well as geometry-related helper functions.

Helper functions:
To get the end-effector pose of an arm in the base link:
    base_pose = get_absolute_pose(arm_state)

To transform an arm frame in general, use convert_ref_frame.
To transform the arm state to be relative to the base:
    transformed_arm_state = convert_ref_frame(arm_state, ArmState.ROBOT_BASE)

To transform the arm state to be relative to a landmark:
    transformed_state = convert_ref_frame(arm_state, ArmState.OBJECT, landmark)

More helper functions are available, see source for details.
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
OBJ_NEAREST_DIST_THRESHOLD = 0.3

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
BASE_LINK = 'base_link'

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


def convert_ref_frame(arm_state, ref_frame, ref_frame_obj=Landmark()):
    """Transforms an arm frame to a new ref. frame.

    Args:
        arm_state (ArmState)
        ref_frame (int): One of ArmState.*
        ref_frame_obj (Landmark): As in Landmark.msg

    Returns:
        ArmState: A copy of arm_state, but transformed.
    """
    output_state = ArmState(arm_state.refFrame,
                            Pose(arm_state.ee_pose.position,
                                 arm_state.ee_pose.orientation),
                            arm_state.joint_pose[:],
                            arm_state.refFrameLandmark)
    if ref_frame == ArmState.ROBOT_BASE:
        if arm_state.refFrame == ArmState.ROBOT_BASE:
            pass  # Nothing to do
        elif arm_state.refFrame == ArmState.OBJECT:
            # Transform from object to robot base.
            ee_in_obj = get_matrix_from_pose(arm_state.ee_pose)
            obj_pose = arm_state.refFrameLandmark.pose  # In base frame T^B_O
            obj_to_base = get_matrix_from_pose(obj_pose)
            abs_ee_pose = get_pose_from_transform(
                np.dot(obj_to_base, ee_in_obj))
            output_state.ee_pose = abs_ee_pose
            output_state.refFrame = ArmState.ROBOT_BASE
            output_state.refFrameLandmark = Landmark()
        else:
            rospy.logerr(
                'Unhandled reference frame conversion: {} to {}'.format(
                    arm_state.refFrame, ref_frame))
    elif ref_frame == ArmState.OBJECT:
        if arm_state.refFrame == ArmState.ROBOT_BASE:
            # Transform from robot base to provided object.
            arm_in_base = get_matrix_from_pose(arm_state.ee_pose)
            base_to_obj = np.linalg.inv(
                get_matrix_from_pose(ref_frame_obj.pose))
            rel_ee_pose = get_pose_from_transform(
                np.dot(base_to_obj, arm_in_base))
            output_state.ee_pose = rel_ee_pose
            output_state.refFrame = ArmState.OBJECT
            output_state.refFrameLandmark = ref_frame_obj
        elif arm_state.refFrame == ArmState.OBJECT:
            if arm_state.refFrameLandmark.name == ref_frame_obj.name:
                pass  # Nothing to do
            else:
                # Transform from arm state's object to provided object.
                ee_in_source_obj = get_matrix_from_pose(arm_state.ee_pose)
                source_obj_to_base = get_matrix_from_pose(
                    arm_state.refFrameLandmark.pose)
                base_to_target_obj = np.linalg.inv(
                    get_matrix_from_pose(ref_frame_obj.pose))
                rel_ee_pose = get_pose_from_transform(
                    np.dot(np.dot(base_to_target_obj, source_obj_to_base),
                           ee_in_source_obj))
                output_state.ee_pose = rel_ee_pose
                output_state.refFrame = ArmState.OBJECT
                output_state.refFrameLandmark = ref_frame_obj
        else:
            rospy.logerr(
                'Unhandled reference frame conversion: {} to {}'.format(
                    arm_state.refFrame, ref_frame))
    return output_state


def get_absolute_pose(arm_state):
    """Returns absolute pose of an end effector state (transforming
    if relative).

    Args:
        arm_state (ArmState)

    Returns:
        Pose
    """
    if arm_state.refFrame == ArmState.OBJECT:
        transformed_arm_state = convert_ref_frame(arm_state, ArmState.ROBOT_BASE)
        return transformed_arm_state.ee_pose
    else:
        return arm_state.ee_pose


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
    if ref_name == 'base_link':
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

# ##################################################################
# Private helper functions
# ##################################################################


def _get_mesh_marker(marker, mesh):
    """Generates and returns a marker from a mesh.

    Args:
        marker (Marker)
        mesh (Mesh)

    Returns:
        Marker
    """
    marker.type = Marker.TRIANGLE_LIST
    index = 0
    marker.scale = Vector3(1.0, 1.0, 1.0)
    while index + 2 < len(mesh.triangles):
        if (mesh.triangles[index] < len(mesh.vertices) and
            mesh.triangles[index + 1] < len(mesh.vertices) and
            mesh.triangles[index + 2] < len(mesh.vertices)):
            marker.points.append(mesh.vertices[mesh.triangles[index]])
            marker.points.append(mesh.vertices[mesh.triangles[index + 1]])
            marker.points.append(mesh.vertices[mesh.triangles[index + 2]])
            index += 3
        else:
            rospy.logerr('Mesh contains invalid triangle!')
            break
    return marker


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

    It also updates the scene visualization.
    Note that landmarks are sometimes called objects or frames.

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
    """

    def __init__(self, tf_listener, im_server, segment_tabletop):
        """Construct a World instance.

        Args:
            tf_listener: A tf.TransformListener
            im_server: An InteractiveMarkerServer for visualizing objects.
            segment_tabletop: A rospy.ServiceProxy for the tabletop
                segmentation service.
        """
        self._objects = []  # Type: [WorldLandmark]
        self._surface = None
        self._lock = threading.Lock()
        self._tf_listener = tf_listener
        self._im_server = im_server
        self._segment_tabletop = segment_tabletop
        self.clear_all_objects()

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
        return object_name == 'base_link' or self.has_object(object_name)

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
                self._add_new_object(
                    Pose(Point((minX + maxX) / 2, (minY + maxY) / 2,
                               (minZ + maxZ) / 2), Quaternion(0, 0, 0, 1)),
                    Point(maxX - minX, maxY - minY, maxZ - minZ), False)
            return True

        except rospy.ServiceException, e:
            print "Call to segmentation service failed: %s" % e
            return False

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

        rospy.loginfo(distances)

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
                #self._publish_tf_pose(self._objects[i].object.pose,
                #                      self._objects[i].name(), BASE_LINK)
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

    def _add_new_object(self, pose, dimensions, is_recognized, mesh=None):
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
        to_remove = None
        if is_recognized:
            # TODO(mbforbes): Re-implement object recognition or remove
            # this dead code.
            return False
        else:
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
            self._add_new_object_internal(pose, dimensions, is_recognized,
                                          mesh)
            return True

    def _add_new_object_internal(self, pose, dimensions, is_recognized, mesh):
        """Does the 'internal' adding of an object with the passed
        properties. Call _add_new_object to do all pre-requisite checks
        first (it then calls this function).

        Args:
            pose (Pose)
            dimensions (Vector3)
            is_recognized (bool)
            mesh (Mesh|None): A mesh, if it exists (can be None).
        """
        n_objects = len(self._objects)
        self._objects.append(WorldLandmark(pose, n_objects, dimensions,
                                           is_recognized))
        int_marker = self._get_object_marker(len(self._objects) - 1)
        self._objects[-1].int_marker = int_marker
        self._im_server.insert(int_marker, self.marker_feedback_cb)
        self._im_server.applyChanges()
        self._objects[-1].menu_handler.apply(self._im_server, int_marker.name)
        self._im_server.applyChanges()

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

    def _get_object_marker(self, index, mesh=None):
        """Generate and return a marker for world objects.

        Args:
            index (int): ID for the new marker.
            mesh (Mesh, optional):  Mesh to use for the marker. Only
                utilized if not None. Defaults to None.

        Returns:
            InteractiveMarker
        """
        int_marker = InteractiveMarker()
        int_marker.name = self._objects[index].get_name()
        int_marker.header.frame_id = 'base_link'
        int_marker.pose = self._objects[index].object.pose
        int_marker.scale = 1

        button_control = InteractiveMarkerControl()
        button_control.interaction_mode = InteractiveMarkerControl.BUTTON
        button_control.always_visible = True

        object_marker = Marker(type=Marker.CUBE,
                               id=index,
                               lifetime=MARKER_DURATION,
                               scale=self._objects[index].object.dimensions,
                               header=Header(frame_id=BASE_LINK),
                               color=COLOR_OBJ,
                               pose=self._objects[index].object.pose)

        if mesh is not None:
            object_marker = _get_mesh_marker(object_marker, mesh)
        button_control.markers.append(object_marker)

        text_pos = Point()
        text_pos.x = self._objects[index].object.pose.position.x
        text_pos.y = self._objects[index].object.pose.position.y
        text_pos.z = (
            self._objects[index].object.pose.position.z +
            self._objects[index].object.dimensions.z / 2 + OFFSET_OBJ_TEXT_Z)
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

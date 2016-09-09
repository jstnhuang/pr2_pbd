from interactive_markers.menu_handler import MenuHandler
from pr2_pbd_interaction.msg import Landmark
import rospy


class WorldLandmark:
    """WorldLandmark represents a task-relevant part of the scene.

    This could include an object, part of an object, or stationary parts of the
    scene.

    There are two kinds of landmarks: bounding boxes and cloud boxes.
    A bounding box is found by segmenting whole objects from a tabletop scene.
    The only shape information kept by a bounding box are its dimensions.

    A cloud box is a point cloud surrounded by a bounding box. The point cloud
    does not necessarily have to be on a tabletop. The bounding box specifies a
    margin of empty space that is expected to be around the point cloud. The
    point cloud is localized in the scene using ICP.

    To create these two kinds of landmarks:
    landmark = WorldLandmark.bounding_box(name, pose, dimensions)
    landmark = WorldLandmark.cloud_box(name, pose, dimensions, db_id)

    All poses are assumed to be relative to the robot's base frame.
    The point cloud for cloud boxes are stored in MongoDB.
    """

    def __init__(self, name, pose, dimensions, db_id):
        """
        Constructs a WorldLandmark object.
        Use bounding_box() or cloud_box() instead.

        Args:
            name (str): Name of this landmark
            pose (Pose): Pose of the landmark, in base frame
            dimensions (Vector3): Size of bounding box
            db_id (str): The MongoDB id if this is a cloud box
        """
        self._name = name
        self._pose = pose
        self._dimensions = dimensions
        self._db_id = db_id

        if self.is_bounding_box():
            self.object = Landmark(type=Landmark.TABLE_TOP,
                                   name=name,
                                   pose=pose,
                                   dimensions=dimensions,
                                   db_id='')
        elif self.is_cloud_box():
            self.object = Landmark(type=Landmark.CLOUD_BOX,
                                   name=name,
                                   pose=pose,
                                   dimensions=dimensions,
                                   db_id=db_id)
        self.int_marker = None
        self.is_removed = False

        # TODO(jstn): Move this outside.
        self.menu_handler = MenuHandler()
        self.menu_handler.insert('Remove from scene', callback=self.remove)

    @staticmethod
    def bounding_box(name, pose, dimensions):
        """Construct a bounding box landmark.

        Args:
            name (str): The name of this landmark.
            pose (Pose): The pose of the bounding box center, in the base frame
            dimensions (Vector3): The x, y, and z lengths of the bounding box
        """
        return WorldLandmark(name, pose, dimensions, None)

    @staticmethod
    def cloud_box(name, pose, dimensions, db_id):
        """Construct a bounding box landmark.

        Args:
            name (str): The name of this landmark.
            pose (Pose): The pose of the bounding box center, in the base frame
            dimensions (Vector3): The x, y, and z lengths of the bounding box
                around this landmark.
            db_id (str): MongoDB ID of this landmark.
        """
        return WorldLandmark(name, pose, dimensions, db_id)

    @staticmethod
    def from_msg(msg):
        """Construct from a Landmark msg.
        """
        db_id = None
        if msg.db_id is not None and msg.db_id != '':
            db_id = msg.db_id
        return WorldLandmark(msg.name, msg.pose, msg.dimensions, db_id)

    def is_bounding_box(self):
        """Returns true if this is a bounding box type landmark."""
        return self._db_id is None

    def is_cloud_box(self):
        """Returns true if this is a cloud box type landmark."""
        return self._db_id is not None

    def name(self):
        """Returns the name of this landmark."""
        return self._name

    def remove(self, __=None):
        """Sets the is_removed flag on this landmark.

        Args:
            __: Unused
        """
        rospy.loginfo('Will remove object: ' + self._name)
        self.is_removed = True

from geometry_msgs.msg import Transform
from mongo_msg_db_msgs.msg import Collection
from object_search_msgs.srv import SearchRequest
from rapid_msgs.msg import StaticCloud
from rapid_msgs.srv import GetStaticCloudRequest
from sensor_msgs.msg import PointCloud2
import rospy
import tf

BASE_FRAME = 'base_link'


class CustomLandmarkFinder(object):
    def __init__(self, static_cloud_db, db_name, collection_name,
                 find_landmark, tf_listener):
        """Constructs the CustomLandmarkFinder

        Args:
            static_cloud_db: A StaticCloudDb
            db_name: The name of the MongoDB database to use
            collection_name: The name of the MongoDB collection to use
            find_landmark: A rospy.ServiceProxy for the
                object_search_msgs/Search service
            tf_listener: A tf.TransformListener
        """
        self._static_cloud_db = static_cloud_db
        self._collection = Collection()
        self._collection.db = db_name
        self._collection.collection = collection_name
        self._find_landmark = find_landmark
        self._tf_listener = tf_listener

    def find(self, db_id):
        """Finds a landmark given a database ID.

        It will read in the scene from the cloud_in topic (this should be
        remapped appropriately).

        Args:
            db_id: The MongoDB id of the landmark to search for in the scene.
        
        Returns:
            None if there was an error, an empty array if there were no matches
            found, and an array of matches (object_search_msgs/Match) if there
            were matches found.
        """
        get_req = GetStaticCloudRequest()
        get_req.collection = self._collection
        get_req.id = db_id
        get_resp = self._static_cloud_db.serve_get_cloud(get_req)
        if get_resp.error != '':
            rospy.logerr(get_resp.error)
            return None

        scene = StaticCloud()
        try:
            scene.cloud = rospy.wait_for_message('cloud_in', PointCloud2, 5)
        except:
            rospy.logerr('Failed to get point cloud on cloud_in '
                         '(has the topic been remapped?)')
            return None
        scene.parent_frame_id = BASE_FRAME
        try:
            trans, rot = self._tf_listener.lookupTransform(
                scene.cloud.header.frame_id, BASE_FRAME, rospy.Time(0))
            scene.base_to_camera.translation = trans
            scene.base_to_camera.rotation = rot
        except:
            rospy.logerr('Failed to get transform from {} to {}'.format(
                BASE_FRAME, scene.cloud.header.frame_id))
            return None

        max_error = rospy.get_param('fitness_threshold', 0.0045)
        search_resp = self._find_landmark(scene=scene,
                                          object=get_resp.cloud,
                                          is_tabletop=False,
                                          max_error=max_error,
                                          min_results=0)
        return search_resp.matches

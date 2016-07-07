#!/usr/bin/env python

import rospy
from pymongo import MongoClient
from mongo_msg_db import MessageDb
from mongo_msg_db_msgs.msg import Collection
from pbd_landmarks.srv import SaveLandmark, SaveLandmarkResponse
from pbd_landmarks.srv import GetLandmark, GetLandmarkResponse
from sensor_msgs.msg import PointCloud2


class LandmarkDb(object):
    def __init__(self, db):
        self._db = db

    def serve_save_landmark(self, req):
        collection = Collection()
        collection.db = 'pr2_pbd'
        collection.collection = 'landmarks'
        response = SaveLandmarkResponse()
        response.id = self._db.insert_msg(collection, req.landmark)
        return response

    def serve_get_landmark(self, req):
        collection = Collection()
        collection.db = 'pr2_pbd'
        collection.collection = 'landmarks'
        matched_count, landmark = self._db.find_msg(collection, req.id)
        response = GetLandmarkResponse();
        if matched_count == 0:
            response.error = 'Landmark was not found.'
        else:
            response.landmark = landmark
        return response


def main():
    rospy.init_node('landmark_db')
    mongo_client = MongoClient()
    mongo_db = MessageDb(mongo_client)
    landmark_db = LandmarkDb(mongo_db)
    save = rospy.Service('save_landmark', SaveLandmark,
                         landmark_db.serve_save_landmark)
    get = rospy.Service('get_landmark', GetLandmark,
                         landmark_db.serve_get_landmark)
    rospy.spin()


if __name__ == '__main__':
    main()

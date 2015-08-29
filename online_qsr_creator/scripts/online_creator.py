#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Pose
from bayes_people_tracker.msg import PeopleTracker
from qsrlib_io.world_trace import Object_State, World_Trace
from qsrlib_ros.qsrlib_ros_client import QSRlib_ROS_Client
from qsrlib.qsrlib import QSRlib_Request_Message
try:
    import cPickle as pickle
except:
    import pickle


class OnlineQtcCreator(object):
    __buffer = {
        "human": [],
        "robot": []
    }

    # Magic qsr_lib parameters for qtc
    __parameters = {"qtcs": {
        "validate": True,
        "no_collapse": False,
        "quantisation_factor": 0.01
    }, "for_all_qsrs": {
            "qsrs_for": [("human","robot")]
    }}

    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self.robot_pose = Pose()
        # QTC needs two agents, subscribing to robot and human pose
        rospy.Subscriber("/robot_pose", Pose, self.robot_cb, queue_size=1)
        rospy.Subscriber("/people_tracker/positions", PeopleTracker, self.ppl_cb, queue_size=1)
        rospy.loginfo("... all done")

    # Robot pose is set in a separate callback because it is not stamped and cannot be synchronised
    def robot_cb(self, msg):
        self.robot_pose = msg

    def ppl_cb(self, msg):
        self.__buffer["human"].append(msg.poses[0])
        self.__buffer["robot"].append(self.robot_pose)

        # QTC needs at least two instances in time to work
        ob = []
        if len(self.__buffer["human"]) > 1:
            # Creating the world trace for both agents over all timestamps
            world = World_Trace()
            for idx, (human, robot) in enumerate(zip(self.__buffer["human"], self.__buffer["robot"])):
                ob.append(Object_State(
                    name="human",
                    timestamp=idx,
                    x=human.position.x,
                    y=human.position.y
                ))
                ob.append(Object_State(
                    name="robot",
                    timestamp=idx,
                    x=robot.position.x,
                    y=robot.position.y
                ))

            world.add_object_state_series(ob)

            # Creating the qsr_lib request message
            qrmsg = QSRlib_Request_Message(
                which_qsr="qtcbs",
                input_data=world,
                dynamic_args=self.__parameters
            )
            cln = QSRlib_ROS_Client()
            req = cln.make_ros_request_message(qrmsg)
            res = cln.request_qsrs(req)
            out = pickle.loads(res.data)

            # Printing the result, publishing might be more useful though ;)
            qsr_array = []
            for t in out.qsrs.get_sorted_timestamps():
                for k, v in out.qsrs.trace[t].qsrs.items():
                    qsr_array.append(v.qsr.values()[0])

            print qsr_array

        rospy.sleep(0.3) # Worst smoothing ever, I'm sure you can do better


if __name__ == "__main__":
    rospy.init_node("online_qtc_creator")
    o = OnlineQtcCreator(rospy.get_name())
    rospy.spin()

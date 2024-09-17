#!/usr/bin/env python3


import rospy
from std_srvs.srv import Trigger, TriggerRequest, Empty, EmptyResponse


def handle_save_image_request(req):
    rospy.loginfo("Received request to save image.")
    # Call the Trigger service to save the data
    try:
        rospy.wait_for_service('/data_collection_server/save_request')
        save_request_service = rospy.ServiceProxy(
            '/data_collection_server/save_request', Trigger)
        trigger_req = TriggerRequest()
        response = save_request_service(trigger_req)
        if response.success:
            rospy.loginfo("Successfully triggered data save.")
        else:
            rospy.logwarn(
                "Failed to trigger data save: %s", response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
    return EmptyResponse()


def main():
    rospy.init_node('save_request_node')
    service = rospy.Service(
        '/save_image_request', Empty,
        handle_save_image_request)
    rospy.loginfo(
        "Ready to receive save image requests on /save_image_request.")
    rospy.spin()


if __name__ == '__main__':
    main()

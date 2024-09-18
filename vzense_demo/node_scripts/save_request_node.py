#!/usr/bin/env python3


import rospy
from std_srvs.srv import Trigger, TriggerRequest, Empty, EmptyResponse
from pybsc import run_command
from pybsc import make_fancy_output_dir
import rospkg
from pathlib import Path


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


def handle_calibration(req):
    rospy.loginfo("Received request to calibration.")
    run_command('rosrun vzense_demo aggregate_calib_images.py', shell=True)
    run_command('docker run -ti --rm --volume="$HOME/MC-Calib:/home/MC-Calib" --volume="$HOME/MC-Calib/data:/home/MC-Calib/data" --volume="$(rospack find vzense_demo)/config/calib_param_two_vzense.yaml:/home/MC-Calib/configs/calib_param_two_vzense.yaml" --volume="$(rospack find vzense_demo)/calib_results:/home/MC-Calib/data/vzense_data" bailool/mc-calib-prod bash -c "cd /home/MC-Calib/build && ./apps/calibrate/calibrate ../configs/calib_param_two_vzense.yaml"', shell=True)
    run_command('rosrun vzense_demo set_calib_tf.py', shell=True)
    return EmptyResponse()


def handle_clear_data(req):
    rospy.loginfo("Received request to clear data.")
    rospack = rospkg.RosPack()
    package_path = Path(rospack.get_path('vzense_demo'))
    source_path = package_path / 'data'
    target_path = package_path / 'data_backup'
    backup_path = Path(make_fancy_output_dir(target_path))
    run_command(f'mv {source_path}/* {backup_path}', shell=True)
    return EmptyResponse()


def main():
    rospy.init_node('save_request_node')
    rospy.Service(
        '/save_image_request', Empty,
        handle_save_image_request)
    rospy.Service(
        '/calib', Empty,
        handle_calibration)
    rospy.Service(
        '/clear_data', Empty,
        handle_clear_data)
    rospy.loginfo(
        "Ready to receive save image requests on /save_image_request.")
    rospy.spin()


if __name__ == '__main__':
    main()

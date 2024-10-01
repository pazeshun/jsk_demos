#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerRequest, Empty, EmptyResponse
import std_msgs.msg
from pybsc import run_command
from pybsc import make_fancy_output_dir
import rospkg
from pathlib import Path
from jsk_rviz_plugins.msg import OverlayText  # jsk_rviz_plugins for OverlayText


n_imgs = 0
data_path = 'data'
results_path = 'calib_results'


# Function to publish text to the OverlayText topic
def publish_overlay_text(text_pub, message, color="white"):
    overlay_text_msg = OverlayText()
    overlay_text_msg.text = message
    overlay_text_msg.width = 1000
    overlay_text_msg.height = 100
    overlay_text_msg.left = 10
    overlay_text_msg.top = 700
    overlay_text_msg.text_size = 20
    overlay_text_msg.line_width = 2
    overlay_text_msg.font = "DejaVu Sans Mono"
    overlay_text_msg.bg_color.r = 0
    overlay_text_msg.bg_color.g = 0
    overlay_text_msg.bg_color.b = 0
    overlay_text_msg.bg_color.a = 0.8  # Transparency
    overlay_text_msg.fg_color.r = 1 if color == "white" else 1
    overlay_text_msg.fg_color.g = 1 if color == "white" else 0
    overlay_text_msg.fg_color.b = 1 if color == "white" else 0
    overlay_text_msg.fg_color.a = 1.0  # Full opacity
    text_pub.publish(overlay_text_msg)


def handle_save_image_request(req, text_pub):
    global n_imgs
    rospy.loginfo("Received request to save image.")
    publish_overlay_text(text_pub, "Received request to save image.")
    # Call the Trigger service to save the data
    try:
        rospy.wait_for_service('/data_collection_server/save_request')
        save_request_service = rospy.ServiceProxy(
            '/data_collection_server/save_request', Trigger)
        trigger_req = TriggerRequest()
        response = save_request_service(trigger_req)
        if response.success:
            rospy.loginfo("Successfully triggered data save.")
            publish_overlay_text(text_pub, "Successfully triggered data save.")
            n_imgs += 1
        else:
            rospy.logwarn(
                "Failed to trigger data save: %s", response.message)
            publish_overlay_text(text_pub, f"Failed to save data: {response.message}")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        publish_overlay_text(text_pub, f"Service call failed: {e}")
    return EmptyResponse()


def handle_calibration(req, text_pub):
    global results_path
    global data_path
    rospy.loginfo("Received request to calibration.")
    publish_overlay_text(text_pub, "Received request for calibration.")
    run_command(f'rosrun vzense_demo aggregate_calib_images.py --data-path {data_path} --results-path {results_path}', shell=True)
    run_command(f'docker run -ti --rm --volume="$HOME/MC-Calib:/home/MC-Calib" --volume="$HOME/MC-Calib/data:/home/MC-Calib/data" --volume="$(rospack find vzense_demo)/config/calib_param_two_vzense.yaml:/home/MC-Calib/configs/calib_param_two_vzense.yaml" --volume="$(rospack find vzense_demo)/{results_path}:/home/MC-Calib/data/vzense_data" bailool/mc-calib-prod bash -c "cd /home/MC-Calib/build && ./apps/calibrate/calibrate ../configs/calib_param_two_vzense.yaml"', shell=True)
    run_command('rosrun vzense_demo set_calib_tf.py', shell=True)
    publish_overlay_text(text_pub, "Calibration completed.")
    return EmptyResponse()


def handle_clear_data(req, text_pub):
    global n_imgs
    global data_path
    rospy.loginfo("Received request to clear data.")
    publish_overlay_text(text_pub, "Received request to clear data.")
    rospack = rospkg.RosPack()
    package_path = Path(rospack.get_path('vzense_demo'))
    source_path = package_path / data_path
    target_path = package_path / '{}_backup'.format(data_path)
    backup_path = Path(make_fancy_output_dir(target_path))
    run_command(f'mv {source_path}/* {backup_path}', shell=True)
    publish_overlay_text(text_pub, "Data cleared and backed up.")
    n_imgs = 0
    return EmptyResponse()


def main():
    global n_imgs
    global data_path
    global results_path
    rospack = rospkg.RosPack()
    package_path = Path(rospack.get_path('vzense_demo'))

    rospy.init_node('save_request_node')
    results_path = rospy.get_param('~results_path', 'calib_results')
    data_path = rospy.get_param('~data_path', 'data')

    for i, dir_path in enumerate((package_path / data_path).glob('*')):
        if not dir_path.is_dir():
            continue
        left_image_path = dir_path / 'left.jpg'
        right_image_path = dir_path / 'right.jpg'
        if left_image_path.exists() and right_image_path.exists():
            n_imgs += 1

    # Create a publisher for OverlayText
    text_pub = rospy.Publisher('/rviz_overlay_text', OverlayText, queue_size=1)
    num_img_pub = rospy.Publisher('/num_calib_images',
                                  std_msgs.msg.Float32, queue_size=1)

    rospy.Service(
        '/save_image_request', Empty,
        lambda req: handle_save_image_request(req, text_pub))
    rospy.Service(
        '/calib', Empty,
        lambda req: handle_calibration(req, text_pub))
    rospy.Service(
        '/clear_data', Empty,
        lambda req: handle_clear_data(req, text_pub))

    rospy.loginfo("Ready to receive save image requests on /save_image_request.")
    publish_overlay_text(text_pub, "Ready to receive save image requests.")

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        num_img_pub.publish(n_imgs)
        rate.sleep()


if __name__ == '__main__':
    main()

###gmapping with abot###
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/ws/devel/setup.bash; roslaunch abot_bringup robot_with_laser.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/ws/devel/setup.bash; roslaunch robot_slam navigation.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/ws/devel/setup.bash; roslaunch track_tag usb_cam_with_calibration.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/ws/devel/setup.bash; roslaunch track_tag ar_track_camera.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/ws/devel/setup.bash; roslaunch robot_slam multi_goal.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; source ~/ws/devel/setup.bash; rosrun robot_voice tts_subscribe; exec bash"' \


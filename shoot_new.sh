#!/bin/bash




gnome-terminal --window --title="roscore" -- bash -c "roscore; exec bash" &
sleep 5


gnome-terminal --tab --title="tts" -- bash -c "source ~/ws/devel/setup.bash; rosrun robot_voice tts_subscribe; exec bash" &
sleep 3


gnome-terminal --tab --title="robot_imu" -- bash -c "source ~/ws/devel/setup.bash; roslaunch abot_bringup robot_with_imu.launch; exec bash" &
gnome-terminal --tab --title="navigation" -- bash -c "source ~/ws/devel/setup.bash; roslaunch robot_slam navigation_shoot.launch; exec bash" &
gnome-terminal --tab --title="camera" -- bash -c "source ~/ws/devel/setup.bash; roslaunch track_tag usb_cam_with_calibration.launch; exec bash" &
sleep 4


gnome-terminal --tab --title="ar_track" -- bash -c "source ~/ws/devel/setup.bash; roslaunch track_tag ar_track_camera.launch; exec bash" &
gnome-terminal --tab --title="find_object" -- bash -c "source ~/ws/devel/setup.bash; roslaunch abot_find find_object_2d.launch; exec bash" &
sleep 3


gnome-terminal --tab --title="shoot_main" -- bash -c "source ~/ws/devel/setup.bash; echo '=== 射击程序启动 ==='; echo '等待3秒确保所有服务就绪...'; sleep 3; roslaunch abot_bringup shoot.launch; exec bash" &
sleep 2


echo ""
echo "请等待约15秒让所有服务完全启动，然后："
echo "1. 在'shoot_main'终端按回车键开始"
echo "2. 说'比赛开始'启动程序"
echo "3. 说出两个点位号码进行射击任务"
echo ""
echo "如果TTS有问题，可以手动重启："
echo "rosnode kill /TextToSpeech && rosrun robot_voice tts_subscribe"

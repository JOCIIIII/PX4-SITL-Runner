1. PC

$ git clone https://github.com/JOCIIIII/PX4-SITL-Runner.git -b PILS-ROBOT
$ cd /dev/gazebo-classic-11-airsim
$ docker compose -f docker-compose.yml --env-file run.env up

2. jetson
$ docker pull kestr3l/ros2:uxrce-humble

$ docker run \
-it \
--privileged \
-e TERM=xterm-256color \
-e DISPLAY=:0 \
-v /dev:/dev \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
--network host \
kestr3l/ros2:uxrce-humble bash \

VScode 실행 후 도커안으로 진입 

$ sudo chmod 666 /dev/ttyUSB0

/home/user/scripts폴더 entrypoint.sh 수정
MicroXRCEAgent UDP -> serial --dev /dev/USB0 -b 921600

$ cd /home/user/scripts
$ ./entrypoint.sh





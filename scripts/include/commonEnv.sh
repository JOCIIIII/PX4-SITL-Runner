# COMMON ENVIRONMENT VARIABLES
## THIS VARIABLES WILL BE USED FOR SCRIPT CONTROL
# >>>----------------------------------------------------
SITL_DEPLOY_DIR=${HOME}/Documents/A4VAI-SITL
# FOR PF-CA INTEGRATION TEST
# -z 0.50 (was 0.00): the x8's skids are 0.44 m below its base, so at 0 they started 0.42 m underground.
# A ground plane pushes them back up, but windy/empty.world's asphalt_plane (a 0.1 m box) caught them
# below its mid-plane - the box pushed down, the plane up: no lift even at full thrust (iris: drops ~0.35 m).
GAZEBO_POSE="-x -27.50 -y 27.50 -z 0.50 -Y -0.785398"
# FOR PP-PF INTEGRATION TEST
#GAZEBO_POSE="-x -180.6640 -y 170.8984 -z 10.0 -Y -0.785398"

SITL_NETWORK_NAME=sitl_network
SITL_NETWORK_SUBNET=172.70.0.0/16

SITL_ROS_DOMAIN_ID="" # IF NOT SET, THIS WILL BE RANDOMLY GENERATED
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


# DEPENDENT ENVIRONMENT VARIABLES
## DO NOT MODIFY THESE VARIABLES UNLESS YOU KNOW WHAT YOU ARE DOING
# >>>----------------------------------------------------
SITL_ENV_DIR=${SITL_DEPLOY_DIR}/envs

PX4_WORKSPACE=${SITL_DEPLOY_DIR}/PX4-Autopilot
GAZEBO_CLASSIC_WORKSPACE=${SITL_DEPLOY_DIR}/Gazebo-Classic
AIRSIM_WORKSPACE=${SITL_DEPLOY_DIR}/AirSim
ROS2_WORKSPACE=${SITL_DEPLOY_DIR}/ROS2
QGC_WORKSPACE=${SITL_DEPLOY_DIR}/QGroundControl
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

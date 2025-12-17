source /home/yjin/miniconda3/etc/profile.d/conda.sh
conda activate polymetis-local
pkill -9 gripper
#chmod a+rw /dev/ttyUSB0
#launch_gripper.py gripper=robotiq_2f gripper.comport=/dev/ttyUSB0
launch_gripper.py gripper=franka_hand  gripper.executable_cfg.robot_ip=172.17.0.2

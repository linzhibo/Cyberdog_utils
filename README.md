# Cyberdog Utils

## Realsense depth image centroid follower
When the height is set to 28cm and the gait set to trot, the dog will bark and enter following mode

### **How to build**

on your dog:
```
mkdir -p ~/ros2_ws/src && cd ~/ros_ws
git clone https://github.com/linzhibo/Cyberdog_utils.git src/cyberdog_utils
colcon build --merge-install --install-base /opt/ros2/cyberdog
sudo cp src/cyberdog_utils/config/cyberdog_follower.service /etc/systemd/system
sudo systemctl enable cyberdog_follower.service 
```
Restart your dog.

**Recommanded tools**

vscode + ssh extension 

## Generate wav file to replace the ugly one
* Dependencies
> pip install baidu-aip
``` bash
cd scripts
python3 csv_2_wav.py
```
the wav files will be generated by data/wav folder

```
scp ../data/wav/* mi@192.168.3.86:/opt/ros2/cyberdog/data/wav/

```
replace ip, reboot the robot.

## Video demo

## Buy me a naicha
<img src="https://github.com/linzhibo/Cyberdog_utils/blob/main/readme_pics/naicha.png">
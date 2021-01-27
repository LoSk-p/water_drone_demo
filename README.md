# Water Drone Demo
Control [Heron](https://github.com/heron) water drone with [Robonomics](https://wiki.robonomics.network/docs/) and publish telemetry with [Ocean](https://docs.oceanprotocol.com/).

## Requirements
* ROS melodic, Gazebo (installation instruction [here](http://wiki.ros.org/melodic/Installation/Ubuntu))
* Some extra packages 
```bash
sudo apt-get install ros-melodic-gazebo-ros-control ros-melodic-effort-controllers ros-melodic-joint-state-controller ros-melodic-imu-tools ros-melodic-uuv-simulator ros-melodic-lms1xx
```
* IPFS version 0.4.22 
```bash
wget https://dist.ipfs.io/go-ipfs/v0.4.22/go-ipfs_v0.4.22_linux-amd64.tar.gz
tar -xvzf go-ipfs_v0.4.22_linux-amd64.tar.gz
cd go-ipfs
sudo bash install.sh
ipfs init
```
* Robonomics node (binary file) (download latest release [here](https://github.com/airalab/robonomics/releases))

## Install package for simulation

Clone Heron repositopies:
```bash
cd catkin_ws/src
git clone https://github.com/heron/heron_simulator
git clone https://github.com/heron/heron.git
git clone https://github.com/heron/heron_controller.git
git clone https://github.com/heron/heron_desktop.git
git clone https://github.com/heron/heron_worlds.git
git clone https://github.com/ros-visualization/interactive_marker_twist_server.git
cd ..
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=melodic -yr
```

## Install control package

Clone control package to your src folder in workspace:
```bash
https://github.com/LoSk-p/water_drone_demo
cd ..
catkin_make
```

## Set Ethereum network & node (Rinkeby & Infura)


1. Infura runs hosted Ethereum nodes. Go to https://infura.io and sign up.
2. At Infura site, create a new project.
3. Within the project settings page, note your Infura `project id` value. We will use it in the next step.
4. Write your Infura `project id` to `ocean/infura` in config file:
```bash
nano ~/catkin_ws/src/water_drone_demo/config/config.yaml
```

## Set Ethereum account and get Rinkeby ETH

1. Install Metamask to your browser and generate an Etherium account. Instructions are [here](https://docs.oceanprotocol.com/tutorials/metamask-setup/).
2. Get Rinkeby ETH from a [fauset](https://faucet.rinkeby.io/).
3. [Export the private key from Metamask](https://metamask.zendesk.com/hc/en-us/articles/360015289632-How-to-Export-an-Account-Private-Key) and write it to `ocean/test_key` in config file:
```bash
nano ~/catkin_ws/src/water_drone_demo/config/config.yaml
```

## Install Python libraries

Create Python virtual env and install libraries:
```bash
python -m venv venv
source venv/bin/activate 
pip install ocean-lib rospkg catkin_pkg ipfshttpclient
```

## Get Rinkeby test OCEAN
Get Rinkeby OCEAN via this [fauset](https://faucet.rinkeby.oceanprotocol.com/).

## Run robonomics local node

To create a local robonomics network go to the folder with the robonomic binary file and run:
```bash
./robonomics --dev --rpc-cors all
```
Go to the [Robonomics Portal](https://parachain.robonomics.network/#/explorer) and switch to local node.
Create DRONE and WORK accounts (instructions [here](https://wiki.robonomics.network/docs/create-account-in-dapp/)).

Write the path to your robonomics binary file and your accounts adresses and key to config file:
```bash
nano ~/catkin_ws/src/water_drone_demo/config/config.yaml
```
Example of config file (without secret keys) you can find [here](https://github.com/LoSk-p/water_drone_demo/blob/main/config/config.yaml).

It's important to remove `db` derictory before next launches using
```bash
rm -rf /home/$USER/.local/share/robonomics/chains/dev/db
```

## Run simulation

Run IPFS daemon:
```bash
ipfs daemon
```
In new terminal run Heron simulation:
```bash
roslaunch heron_gazebo heron_world.launch
```
In new terminal run control package:
```bash
roslaunch water_drone_demo send_to_ocean.launch
```
In new terminal send launch transaction frome WORK to DRONE:
```bash
echo "ON" | ./robonomics io write launch -r <drone_address> -s <employer_key>
```

## Results
You can find your datatoken with telemetry in [Ocean Market](https://market.oceanprotocol.com/). Connect your Metamask account and choose Rinkeby network in Metamask. Then go to `HISTORY` and you'll see your datatoken.

Also in [Robonomics Portal](https://parachain.robonomics.network/#/explorer) you can see the hash of your file with telemetry in IPFS. For that go `Developer/Chain state`, in query choose `datalog`, choose your DRONE account and press `+` button.

# FC-Sim

~~~
mkdir ~/ws_sim && cd ~/ws_sim
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
git checkout v1.17.0-beta1
git submodule update --init --recursive
bash ./Tools/setup/ubuntu.sh --no-nuttx
make clean
rm -rf build/
make px4_sitl
~~~
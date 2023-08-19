# NS3-NR-C-V2X
This is a sub-module of our [co-simulator](https://github.com/S-kewen/carla-generator) for NR C-V2X vehicle network simulation.

## Installation
### Requirements
All the codes are tested in the following environment:
* Ubuntu 20.04 (you can use WSL Ubuntu 20.04 in Windows 10)
* CMake 3.16.3
* NS-3 3.36
* ZMQ 4.3.4

## Quick demo
### a. Install CMake, ZMQ, and libjson (if already exists can skip )
#### install cmake
```
sudo apt-get update
sudo apt-get install clang
sudo apt install cmake
sudo apt-get install libtool pkg-config build-essential autoconf automake
```
#### install libsodium
```
git clone https://github.com/jedisct1/libsodium
cd libsodium
git checkout 1.0.10
./autogen.sh -f -s
./configure && make check 
sudo make install
sudo ldconfig
cd ..
```
#### install zmq
```
sudo apt-get install libzmq3-dev
```
#### install cppzmq
```
git clone https://github.com/zeromq/libzmq.git
cd libzmq
./autogen.sh
./configure -with-libsodium && make 
sudo make install 
sudo ldconfig
cd ../
```
#### install zmqpp
```
git clone https://github.com/zeromq/zmqpp.git
cd zmqpp
mkdir build
cd build 
cmake ..
make 
sudo make install
```
#### install libjson
```
sudo apt-get install libjsoncpp-dev 
sudo ln -s /usr/include/jsoncpp/json/ /usr/include/json
```
### b. Build NS-3
```
git clone https://github.com/S-kewen/NS3-NR-C-V2X
cd NS3-NR-C-V2X
rm -R build
rm -R cmake-cache
rm .lock-ns3_linux_build
./ns3 configure
```
### c. Run
```
./ns3 run NR_C_V2X -- --nodes={your_nodes} --zmqPort={your_zmq_port}
```
- _nodes_: number of networked devices in the CARLA, such LiDAR-Vehicle and edge server.
- _zmqPort_: default is 5557.

<!-- ## Citation 
If you find this project useful in your research, please consider cite:
```
XXXX
``` -->

## Acknowledgment
Part of our code refers to the work [Millicar](https://github.com/signetlabdei/millicar).

## Contribution
welcome to contribute to this repo, please feel free to contact us with any potential contributions.
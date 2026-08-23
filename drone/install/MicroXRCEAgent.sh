#!/bin/bash

cd /tmp

git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    
cd Micro-XRCE-DDS-Agent && mkdir build && cd build

cmake .. -DUAGENT_P2P_PROFILE=OFF
    
make -j$(nproc)
    
mkdir -p /MicroXRCEAgent/bin

cp MicroXRCEAgent /MicroXRCEAgent/bin

mkdir -p /MicroXRCEAgent/lib

find /tmp/Micro-XRCE-DDS-Agent/build -type f -name "*.so*" -exec cp -a {} /MicroXRCEAgent/lib/ \;
find /tmp/Micro-XRCE-DDS-Agent/build -type l -name "*.so*" -exec cp -a {} /MicroXRCEAgent/lib/ \;

rm -rf /tmp/Micro-XRCE-DDS-Agent

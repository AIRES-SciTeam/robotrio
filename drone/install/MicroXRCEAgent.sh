#!/bin/bash

cd /tmp

git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    
cd Micro-XRCE-DDS-Agent && mkdir build && cd build

cmake .. -DUAGENT_P2P_PROFILE=OFF
    
make
    
mkdir -p /MicroXRCEAgent/bin

cp MicroXRCEAgent /MicroXRCEAgent/bin

export PATH="/MicroXRCEAgent/bin:$PATH"

mkdir -p /MicroXRCEAgent/lib

find /tmp/Micro-XRCE-DDS-Agent/build -type f -name "*.so*" -exec cp -a {} /MicroXRCEAgent/lib/ \;
find /tmp/Micro-XRCE-DDS-Agent/build -type l -name "*.so*" -exec cp -a {} /MicroXRCEAgent/lib/ \;

export LD_LIBRARY_PATH="/MicroXRCEAgent/lib:$LD_LIBRARY_PATH"

rm -rf /tmp/Micro-XRCE-DDS-Agent

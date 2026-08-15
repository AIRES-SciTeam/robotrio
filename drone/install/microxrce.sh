#!/bin/bash

cd /tmp

git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    
cd Micro-XRCE-DDS-Agent && mkdir build && cd build

cmake .. -DUAGENT_P2P_PROFILE=OFF -DBUILD_SHARED_LIBS=OFF
    
make
    
cp MicroXRCEAgent /robotrio/drone/bin/
    
chmod +x /robotrio/drone/bin/MicroXRCEAgent
  
rm -rf /tmp/Micro-XRCE-DDS-Agent

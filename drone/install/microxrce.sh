#!/bin/bash

git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    
cd Micro-XRCE-DDS-Agent && mkdir build && cd build

cmake .. -DUAGENT_USE_SYSTEM_FASTCDR=ON -DUAGENT_USE_SYSTEM_FASTDDS=ON -DUAGENT_P2P_PROFILE=OFF
    
make
    
cp MicroXRCEAgent /robotrio/drone/bin/
    
chmod +x /robotrio/drone/bin/MicroXRCEAgent
  
rm -rf /tmp/Micro-XRCE-DDS-Agent

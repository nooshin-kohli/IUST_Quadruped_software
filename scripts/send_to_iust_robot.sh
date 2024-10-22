#!/bin/bash
# Author: aligolestaneh
# Date: 01/24/2023
# This shell script is modified based on MIT corresponding script to send software to real robot.
# Unlike MIT, we can select iust_cheetah's ip address and controller by options.
# wifi ip is the default address and the iust_ctrl is the default controller if don't specify it.

echo "[USAGE] sh send_to_iust_cheetah.sh [Controller_path/executable] [wifi|wire]"
echo "[EXAMPLE] sh send_to_iust_cheetah.sh IUST_Controller/iust_ctrl wifi"
## Set robot's ip
wifiip=172.18.252.145
# wireip=10.0.0.21
name=up2

## Create a prepare dir robot-software
set -e
#DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
DIR="$(pwd)"
cd ${DIR}/../mc-build/
rm -rf robot-software
mkdir robot-software
mkdir robot-software/build
#cp common/test-common robot-software/build
if [ -z "$1" -o "$1" = "iust" ]
then
  echo "[INFO] Copying dafault controller: iust_ctrl..."
  cp ../mc-build/user/IUST_Controller/iust_ctrl robot-software/build
else
  if [ "$1" = "jpos" ]; then
    echo "[INFO] Copying jpos_ctrl ..."
    cp ../mc-build/user/MiLAB_JPos_Controller/jpos_ctrl robot-software/build
  elif [ "$1" = "spi" ]; then
    echo "[INFO] Copying spi_ctrl..."
    cp ../mc-build/user/MiLAB_Spi_Controller/spi_ctrl robot-software/build
  elif [ "$1" = "low" ]; then
    echo "[INFO] Copying spi_ctrl..."
    cp ../mc-build/user/MiLAB_Lowlevel_Controller/low_ctrl robot-software/build
  else
    echo "[ERROR] $1 is not a existed controller"
    echo "[USAGE] sh send_to_milab_cheetah.sh [mpc|spi|jpos| ] [ |wifi|wire]"
    echo "[EXAMPLE] sh send_to_milab_cheetah.sh spi wifi"
    exit 1
  fi
fi
find . -name \*.so* -not -path "./robot-software/build/*" -exec cp {} ./robot-software/build \;
cp ../scripts/run_* ./robot-software/build
cp ../scripts/setup_network_mc.py ./robot-software/build
cp ../scripts/run_mc_debug.sh ./robot-software/build
cp ../scripts/config_network_lcm.sh ./robot-software
cp -r ../robot robot-software
cp -r ../config robot-software
chmod +x ./robot-software/build/*

## Send dir to real robot's computer
if [ -n "$2" ]
then
  if [ "$2" = "wifi" ]
  then
    echo "[INFO] Using Wifi connection ip address."
    echo "[INFO] Sending ... (It may take few minutes)."
    scp -rq robot-software $name@$wifiip:~/
  else
    echo "[INFO] Using wire connection ip address.  "
    echo "[INFO] Sending ... (It may take few minutes)."
    scp -rq robot-software $name@$wireip:~/
  fi
else
    echo "[INFO] No connection type specified, using wifi connection as default."
    echo "[INFO] Sending ... (It may take few minutes)."
    scp -rq robot-software $name@$wifiip:~/
fi

## Check result
if [ $? -eq 0 ]
then
  echo "[INFO] Finish software sending successfully."
  exit 0
else
  echo "[ERROR] Something wrong, sending failed."
  exit 1
fi

#!/usr/bin/env bash
# Run ON YOUR PC (192.168.1.114), in a local terminal:
#
#   scp pi@192.168.1.50:/home/pi/RL/hardware_adapter/python/testuart.py \
#       ~/RL/CatSwarm/ComNet/testuart.py
#   chmod +x ~/RL/CatSwarm/ComNet/testuart.py
#
# Or from this repo on the Pi, copy the same file:
#   scp pi@192.168.1.50:/home/pi/RL/deployscripts/testuart/testuart.py \
#       ~/RL/CatSwarm/ComNet/testuart.py

echo "Run on PC:"
echo "  mkdir -p ~/RL/CatSwarm/ComNet"
echo "  scp pi@192.168.1.50:/home/pi/RL/hardware_adapter/python/testuart.py ~/RL/CatSwarm/ComNet/"
echo "  chmod +x ~/RL/CatSwarm/ComNet/testuart.py"
echo "  python3 ~/RL/CatSwarm/ComNet/testuart.py --side pc"

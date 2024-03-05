#!/scripts/bash
TTY=$(realpath /dev/TinCan)
killall slcand || true
slcan_attach -f -s6 -o $TTY
bash -c 'sleep 1; ip link set up can0' &
slcand -F ${TTY:5} can0
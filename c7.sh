#!/bin/bash

echo "Hello from c7-guest!"
cd /root/rtete
make -C /usr/src/kernels/$(uname -r) SUBDIRS=/root/rtete clean modules
sync
rmmod rtete
rmmod snd-aloop
dmesg -c > before.log
sync
modprobe snd-aloop
insmod ./rtete.ko pcm_tx=/dev/snd/pcmC1D0p pcm_rx=/dev/snd/pcmC1D1c
cat '/proc/asound/card1/cable#0'
sleep 10
ifconfig rtete0
cat '/proc/asound/card1/cable#0'
rmmod rtete
dmesg -c > after.log
sync

#!/bin/sh

cards=$(lspci | grep 1234:beef | wc -l)
count=0

for card in $(seq 0 $(($cards - 1)))
do
    mknod /dev/echo-ctrl$card c 64 $count
    count=$((count + 1))
    mknod /dev/echo-event$card c 64 $count
    count=$((count + 1))
done

#!/bin/sh

if [ "$#" -ne 2 ]; then
    echo "Usage: $0 file_name 1-auto create file | 0-shell create file"
    exit 1
fi

file_name=$1
option=$2

if [ $option -eq 1 ]; then
    insmod $file_name createFile=1
elif [ $option -eq 0 ]; then
    insmod $file_name createFile=0
    cards=$(lspci | grep 1234:beef | wc -l)
    count=0

    for card in $(seq 0 $(($cards - 1)))
    do
        mknod /dev/echo-ctrl$card c 64 $count
        count=$((count + 1))
        mknod /dev/echo-event$card c 64 $count
        count=$((count + 1))
    done
else
    echo "Invalid option. Please use 1 or 0."
    exit 1
fi


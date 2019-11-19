#!/bin/sh

rm -rf "/var/lib/bluetooth/*"
hciconfig hci0 reset
python test1.py
sleep 1


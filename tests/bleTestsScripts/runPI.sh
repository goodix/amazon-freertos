#!/bin/sh

scp * root@192.168.43.204: 
ssh -t -t 192.168.43.204 -l root << 'ENDSSH'
rm -rf "/var/lib/bluetooth/*"
hciconfig hci0 reset
python test1.py
sleep 1
ENDSSH


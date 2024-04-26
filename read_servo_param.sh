#!/bin/bash

echo "ethercat upload -p 0 -a 0 -t int8 0x6098 0x00 --- home method"
ethercat upload -p 0 -a 0 -t int8 0x6098 0x00
sleep 1

echo "ethercat upload -p 0 -a 0 -t uint32 0x6099 0x01 --- home vel1"
ethercat upload -p 0 -a 0 -t uint32 0x6099 0x01
sleep 1

echo  "ethercat upload -p 0 -a 0 -t uint32 0x6099 0x02 --- home vel2"
ethercat upload -p 0 -a 0 -t uint32 0x6099 0x02
sleep 1

echo "ethercat upload -p 0 -a 0 -t uint32 0x609A 0x00 --- home ACC"
ethercat upload -p 0 -a 0 -t uint32 0x609A 0x00
sleep 1

echo "ethercat upload -p 0 -a 0 -t uint32 0x607F 0x00 --- home limit vel"
ethercat upload -p 0 -a 0 -t uint32 0x607F 0x00
sleep 1

echo "ethercat upload -p 0 -a 0 -t uint16 0x60E0 0x00 --- home torque +"
ethercat upload -p 0 -a 0 -t uint16 0x60E0 0x00
sleep 1

echo "ethercat upload -p 0 -a 0 -t uint16 0x60E1 0x00 --- home torque -"
ethercat upload -p 0 -a 0 -t uint16 0x60E1 0x00
sleep 1

echo "ethercat upload -p 0 -a 0 -t uint16 0x6072 0x00 --- Max torque"
ethercat upload -p 0 -a 0 -t uint16 0x6072 0x00
sleep 1

echo "ethercat upload -p 0 -a 0 -t uint32 0x6080 0x00 --- Max motor vel"
ethercat upload -p 0 -a 0 -t uint32 0x6080 0x00
sleep 1

echo "done"

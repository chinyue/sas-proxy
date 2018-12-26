# SAS Proxy

This is a SAS (Steering Angle Sensor) proxy that forwards SAS messages from one
CAN bus to another. It listens for CAN messages which contains SAS info, scales
up the angle, and send it out on another CAN bus.

## Why You Need a SAS Proxy

I wrote the SAS proxy to solve my own problem.

I own a '11 Subaru Forester XT which has a 19:1 steering rack. I installed
'15 STI steering rack which is 13:1 and then VDC kicks in engaging brakes when
the car is in turn with some speed. VDC considers the car is slipping because
the speed differences detected between inner and outer wheels is too large
for the turn. However it has no idea the car is running on a faster rack.

One way to solve the problem is to update the VDC code so it knows the new
configuration of the rack, but there's no such aftermarket solution. So I went
the other way, scale up the steering angle so VDC sees angle similar to stock
rack.

## Hardware Setup

You need:
- 1x Arduino Uno
- 2x Seeed-Studio CAN BUS shields

## Software Setup

1. Install the [CAN BUS Shield library](https://github.com/Seeed-Studio/CAN_BUS_Shield/blob/master/README.md#installation)
1. Check out sas-proxy from GitHub.
1. Open it with Arduino IDE.
1. Build and Upload.

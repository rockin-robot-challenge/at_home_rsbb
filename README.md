RoCKIn@Home Referee, Scoring and Benchmarking Box
=================================================

This repository contains the RoCKIn@Home Referee, Scoring and Benchmarking Box.

The repository *RoAH RSBB Comm* from https://github.com/rockin-robot-challenge/at_home_rsbb_comm
is included as a git submodule.

:warning: Please remember to always update right before the competitions!
```bash
git pull
git submodule update --init
```


## Dependencies

You need to have installed a C++11 compiler, CMake, Boost, Protobuf
and OpenSSL.

If you are using Ubuntu, install the dependencies with:
```bash
sudo apt-get install build-essential cmake libboost-all-dev libprotoc-dev protobuf-compiler libssl-dev
```

Furthermore, you need to use at least ROS Hydro, follow the
instructions at http://wiki.ros.org/ROS/Installation/ .

This was tested with Ubuntu 12.04.5 LTS (Precise Pangolin) and
14.04.1 LTS (Trusty Tahr).

This package depends on the `roah_devices` and the `rockin_benchmarking`
packages, which must be available in the Catkin workspace.
- `roah_devices` can be cloned or downloaded from http://github.com/joaocgreis/roah_devices
- `rockin_benchmarking` can be downloaded from http://users.isr.tecnico.ulisboa.pt/~jreis/rockin/rockin_benchmarking_2182.tar.xz

For `rockin_benchmarking` to run, it is also necessary to install the Levenshtein module for Python:
```bash
sudo easy_install python-Levenshtein
```


## Compiling

After `git clone` and after every `git pull`, please do:
```bash
git submodule update --init
```

Compile as a normal ROS package in your Catkin workspace. Make sure
roah_devices is available.


## Running

You can run the full RSBB including the Core, the Interface and the
devices node with:
```bash
roslaunch roah_rsbb roah_rsbb.launch
```

For a test with dummy home devices use:
```bash
roslaunch roah_rsbb roah_rsbb_dummy.launch rsbb_host:=192.168.1.255 --screen
```

The `rsbb_host` parameter should be set to the `Bcast` of the interface you want to use, as reported by `ifconfig`. Do not run the RSBB in the same computer as the client (robot).

It may be necessary to delete the rqt cache for the new components to
appear:
```bash
rm ~/.config/ros.org/rqt_gui.ini
```


## Securing the RSBB

Make sure that you run these commands in whatever computer runs the RSBB:
```bash
sudo iptables -A INPUT -i lo -p tcp -m tcp --dport 11311 -j ACCEPT
sudo iptables -A INPUT -p tcp -m tcp --dport 11311 -j DROP
```

You might add this to `/etc/rc.local`, before the `exit` command.

To be able to connect from other computers safely, you must install
the `openssh-server` package:
```bash
sudo apt-get install openssh-server
```

Make sure the `ROS_IP` variable is set correctly.


#### Connecting from remote computers

To launch RSBB clients in other computers, you must have the
`openssh-server` package installed in the server and be running the
RSBB. Then, in the remote computer do:
```bash
ssh -L 127.0.0.1:11311:10.0.0.1:11311 rockin@10.0.0.1
```

In this example, the user is named `rockin` and the server is at
`10.0.0.1`. The `127.0.0.1` at the beginning is mandatory.

Make sure the `ROS_IP` variable is set correctly.

Then, just run the client as if the ROS master were local:
```bash
roslaunch roah_rsbb roah_rsbb_client.launch
```

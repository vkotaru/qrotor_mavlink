# qrotor_mavlink
Custom C++, Eigen wrapper for `libmavconn` from [mavros](https://github.com/mavlink/mavros/tree/master)

> :warning: `qrotor_mavlink` currently only supports `UDP`

> :warning: libmavconn from `qrotor_mavlink`  will have version conficts with the `libmavconn` from `ros-<version>-mavros`.

##### generate cpp msg header files (if not already generated)
```
bash install_mavlink_msgs.sh
```

##### build
```
mkdir build
cd build
cmake ..
make
```


---
(Alternately, https://github.com/rosflight/firmware/blob/master/comms/mavlink/mavlink.cpp and https://github.com/mavlink/mavlink/blob/master/examples/linux/mavlink_udp.c can be used together)


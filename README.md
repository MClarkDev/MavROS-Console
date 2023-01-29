# MavROS Console

This console is a basic interface designed to control a MavROS remote node. 

![MavROS Console Screenshot](https://ftp.mclarkdev.com/uploads/media/mavConsole.png)

## Setup

To connect to a remote node you must first set an environment variable.

```
export ROS_MASTER_URI=http://<ip>:11311/
python mavConsole.py
```


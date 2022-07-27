# Docker_OT2

- Builds the demo Docker container to be run on terminal connected to the OT2 robot.
- Interfaces with another docker container(master) that provides the robot configuration and the protocol configuration
- Provides the ROS interface (demo > action_client node) that forwards the configuration file to the OT2 robot 

## Configuation Instructions/Setup Notes

- The host computer master/scheduler terminal must be on the same network, verified by successfull pinging.


## Runtime Instructions

1. Clone the repository

```git clone https://github.com/shirleyzzr1/Docker_OT2.git```

2. Change directory into the newly-cloned repository

```cd Docker_OT2```

3. Build the docker image (might be required to prepend docker commands with sudo)
   
```docker build -t ot2```

4. Launch a container instance of the newly-built image
   
```docker run -it --net=host ot2 ```

5. Run the action_server node
```ros2 run demo action_client```


NEXT: Launch the action_server node automatically on container startup
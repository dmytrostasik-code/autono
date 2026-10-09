# ArduRover path-following assignment

In this test assignment you will implement a path-following controller for a differential drive robot. The code already contains a setup built for you which includes:

- Gazebo simulation with a differential drive rover, ArduRover SITL, and MAVROS
- A trajectory controller node that loads a path, reads ground-truth odometry (`/ground_truth/odom`), arms the rover, and switches it to GUIDED mode
- A path recorder, a path scorer, and RViz visualization (reference path in green, driven path in red)

Localization is given. Your task is the tracking law: implement `Control()` in [`src/ardurover_nav/src/ardurover_controller.cpp`](src/ardurover_nav/src/ardurover_controller.cpp). You may change the controller class as needed.

There are two launch files:

- `ros2 launch ardurover_nav sim.launch.py` — simulation only (Gazebo, ArduRover SITL, MAVROS). Use this to drive the rover from QGroundControl and record a path.
- `ros2 launch ardurover_nav control.launch.py` — simulation plus your controller, RViz, and the scorer. This is the command that tests your controller.

After `./scripts/build.sh` and `source install/setup.bash`:

```bash
ros2 launch ardurover_nav control.launch.py path_file:=/home/developer/ardurover_navigation/paths/0-drive-straight.path
```

Rebuild after you change the controller.

`control.launch.py` starts `path_scorer_node`, which samples the rover pose and writes `paths/score.txt` when the last waypoint is reached (within 1 m for 1 s) or after 180 s:

```
score = 100 * completion * exp(-rms_cte / 0.75) * exp(-max_cte / 4.0)
```

- **completion** — fraction of path length reached
- **rms_cte / max_cte** — RMS and max distance to the reference path (m)

Higher is better. A perfect run along the whole path scores 100.

## Deliverables

- Implementation of the controller, along with any other code you changed. Ideally a fork of this repo and a link to it. Your code should run, I can clone your repo, run this command and see your controller following 3 different path that come with this repo

    ```bash
    ros2 launch ardurover_nav control.launch.py path_file:=/home/developer/ardurover_navigation/paths/0-drive-straight.path
    ```

    ```bash
    ros2 launch ardurover_nav control.launch.py path_file:=/home/developer/ardurover_navigation/paths/1-drive-with-turns.path
    ```

    ```bash
    ros2 launch ardurover_nav control.launch.py path_file:=/home/developer/ardurover_navigation/paths/2-complicated.path
    ```

- A short report documenting how the controller works. For example, if you used a PID controller, explain the principle, which commands you send, how they are calculated, and include any mathematical formulas.
- A video of the screen recording of the Gazebo and RViz screens, showing the controller following a given path (you can choose any path you want for the video).

# Setup Instructions
## Local development environment
 _The code was tested and run on Ubuntu 24 system with NVIDIA GPU, it is not guaranteed it will work well on Windows or Mac operating system. If you don't have access to Ubuntu computer you can try to adapt this repo to run in another operating system, but this is not recommended._

You need Docker, X11, and [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html). A GPU is optional.

```bash
cd /path/to/ardurover_navigation
./docker/build.sh          # can take some time, be oatient
./docker/run.sh            # opens a shell in the container
```

`./docker/run.sh` again attaches if the container already exists. Extra shells: `./docker/attach.sh`.

Build the code inside the container:

```bash
./scripts/build.sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Then use the launch commands from the assignment section above.

To run simulation only withou the controller
```bash
ros2 launch ardurover_nav sim.launch.py
```

or to test your controller

```bash
ros2 launch ardurover_nav control.launch.py path_file:=/home/developer/ardurover_navigation/paths/0-drive-straight.path
```

## VS Code / Cursor

Attaching the editor to the container is optional. See [ide-setup.md](ide-setup.md).

## Connect from QGroundControl

On the host, add a UDP connection to `127.0.0.1:14550` (or wait for auto-connect). Use this to arm in **Manual** / **Acro** and drive with a joystick or QGC virtual joystick.

## Record a path

1. Start the simulation only: `ros2 launch ardurover_nav sim.launch.py`
2. Connect QGroundControl, arm, and drive the rover.
3. In a second container shell:

```bash
ros2 run ardurover_nav path_recorder_node
```

The node samples `x y yaw` every 200 ms from `/ground_truth/odom` and writes `paths/recorded.path`. Ctrl+C saves the file.

Restart the sim so the rover is back at spawn before following the path.

# robot-mBuff

ROS 2 Jazzy workspace and Docker container for the **mBuff** robot.

This repository holds:
- A Dockerfile to build a ROS 2 Jazzy development container for Raspberry Pi.
- A colcon workspace (`ws/`) with ROS 2 Python packages:
  - **msense_hat_node** — stub node for the Sense HAT.
  - **web_bridge_node** — stub node for bridging web commands.
  - **mbuff_bringup** — launch package to start everything together.

## Getting Started

### 1. Build the container
```bash
docker compose build robot-mbuff
```

### 2. Run the container
```bash
docker compose run --rm robot-mbuff bash
```

### 3. Build the workspace
Inside the container:
```bash
cd /ws
colcon build
. install/setup.bash
```

### 4. Launch nodes
Run them all together:
```bash
ros2 launch mbuff_bringup mbuff_min.launch.py
```

You should see heartbeat logs from both `msense_hat_node` and `web_bridge_node`.

## Repository Layout
```
robot-mBuff/
 ├── Dockerfile
 ├── entrypoint.sh
 ├── ws/
 │   └── src/
 │       ├── msense_hat_node/
 │       ├── web_bridge_node/
 │       └── mbuff_bringup/
 └── docker-compose.yml  (at parent repo)
```

## Next Steps
- Implement Sense HAT data publishing in `msense_hat_node`.
- Extend `web_bridge_node` to accept HTTP/WebSocket commands.
- Add camera + AI package later.

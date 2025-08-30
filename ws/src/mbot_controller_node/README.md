# mBot Controller Node

This package provides a ROS 2 node (`mbot_controller`) that listens for
high-level motion commands on the `/mbuff/move` topic and forwards
them to the mBot robot over a serial connection.  It serves as the
bridge between the web interface exposed by `web_bridge_node` and the
physical robot.

## Supported commands

The node recognises a wide range of high-level commands.  Commands
are case-insensitive on input; they are converted to uppercase and
sent verbatim over the mBot’s serial connection.  Any other
command will be ignored and a warning will appear in the ROS logs.

### Basic movement

| Command   | Description                             |
|----------|-----------------------------------------|
| `FORWARD`  | Move the robot forward                    |
| `BACKWARD` | Move the robot backwards                  |
| `LEFT`     | Move the robot left (strafe)             |
| `RIGHT`    | Move the robot right (strafe)            |
| `STOP`     | Stop all movement                        |

### Spinning

These commands correspond to the coloured corner buttons on the
interface.  They cause the mBot to spin in place or while reversing.

| Command            | Description                            |
|-------------------|----------------------------------------|
| `SPIN_LEFT`        | Spin on the spot to the left             |
| `SPIN_RIGHT`       | Spin on the spot to the right            |
| `SPIN_BACK_LEFT`   | Spin backwards while turning left        |
| `SPIN_BACK_RIGHT`  | Spin backwards while turning right       |

### Speed control

| Command    | Description                       |
|-----------|-----------------------------------|
| `SPEEDUP`   | Increase the robot’s speed         |
| `SPEEDDOWN` | Decrease the robot’s speed         |

### Dance routines

Use these to trigger pre-programmed dance moves (IDs 1–4):

| Command | Description                          |
|--------|--------------------------------------|
| `DANCE1` | First dance routine (Wiggle Dance)   |
| `DANCE2` | Second dance routine (Spin Move)     |
| `DANCE3` | Third dance routine (Happy Bounce)   |
| `DANCE4` | Fourth dance routine (Robot Shuffle) |

### Songs

Play one of four built-in songs:

| Command | Description                          |
|--------|--------------------------------------|
| `SONG1` | Play the first song (Happy Song)     |
| `SONG2` | Play the second song (Robot Beeps)   |
| `SONG3` | Play the third song (Fun Tune)       |
| `SONG4` | Play the fourth song (Dance Beat)    |

### Sayings

Make the robot speak one of six phrases:

| Command   | Description                  |
|----------|------------------------------|
| `SAYING1` | Say “Hello Friend”           |
| `SAYING2` | Say “I am mBuff”             |
| `SAYING3` | Say “Let’s play”             |
| `SAYING4` | Say “You are awesome”        |
| `SAYING5` | Say “Time to dance”          |
| `SAYING6` | Say “Beep boop”              |

Incoming commands are normalised to uppercase and then sent
verbatim to the mBot’s serial interface.  You can tailor the mBot
firmware to interpret these strings accordingly.

## Serial configuration

The serial port and baud rate are configurable via environment
variables:

- **`MBOT_SERIAL_PORT`** — path to the serial device (default:
  `/dev/ttyUSB0`).
- **`MBOT_BAUDRATE`** — communication speed (default: `9600`).

For reliability the node opens and closes the serial port for each
command.  It waits for a short initialisation delay (default: 1 second)
after opening the port before transmitting.  If your mBot firmware
requires more or less time to become ready, you can adjust the
`SERIAL_INIT_DELAY` constant in
`mbot_controller_node/mbot_controller_node.py`.

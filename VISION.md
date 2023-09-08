# Pupper + Vision

We use the OAKD camera on the front of Pupper to see the world.

## Installation

We follow the directions from https://docs.luxonis.com/projects/api/en/latest/install.

1. `bash -c "$(curl -fL https://docs.luxonis.com/install_dependencies.sh)"`
2. `python3 -m pip install depthai`

3. We can then run Python example code with:
    ```bash
    git clone https://github.com/luxonis/depthai-python.git
    cd depthai-python
    ```
Take a look at https://docs.luxonis.com/projects/api/en/latest/tutorials/hello_world/ to understand how to interface with the `depthai` API.

## Running Pupper Fetch

The script `run_camera.py` will constantly track a yellow ball (tuned for a Spikeball) and return the yaw and pitch offsets that the ball is with respect to the head of the robot. It then sends over these offsets to the main script `run_djipupper.py` which will use them if the ball tracking mode is activate with the joystick. So to run this, you will need 2 terminal windows on the Raspi (I recommend using tmux) and run
```bash
# Terminal 1
python3 run_djipupper.py

# Terminal 2
python3 run_camera.py
```

The new joystick commands to control Pupper are:
- Left joystick is translational x,y velocity
- Right joystick is pitch and yaw
- Left switch toggles between activate (standing up) and deactivate (lying down without motors powered)
- Right switch toggles between standing and moving
- Left bumper activates either homing or the autonomous mode depending on the direction
- Right bumper can swap between a trot gait and the standard walk gait

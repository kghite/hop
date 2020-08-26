# Hop FSM State Table

| State | Green LED | Red LED | Description |
|-------|-----------|---------|-------------|
| booting | blink | off | Boot the robot and run inits |
| standing | on | blink | Stand up from the ground pose |
| stable | on | off | Balance in a stable standing state |
| teleop | on | blink | Move to follow teleop in balance |
| execute | on | blink | Execute file of mission states |
| fallen | off | blink | Recover from fall to ground pose |
| estopped | off | on | Handle a safety or hardware estop |
| stopping | on | blink | Shut down the robot |
| testing | on | blink | Testing state for development|
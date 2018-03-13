# EE183DA Lab 3/4

**Installation**

This installation guide assumes you have already constructed the paperbot as shown here:

https://git.uclalemur.com/mehtank/paperbot/

This small guide is simply to install our version of the code.
Steps:
1. Download and install MatlabWebSocket here: https://github.com/jebej/MatlabWebSocket.
2. Download and extract code. Upload paperbot.ino code to the arduino.
3. The PaperbotClient.m is the object that interacts with the arduino.

**Usage**

See the example main.m for a concrete example.
The PaperbotClient code can be initialized as follows:
```
client = PaperbotClient(x_dimension, y_dimension, angle_offset, initial_state, websocket_address);
```
After initiliazation, the pathing function can be called to set the path from the bot's current state to some goal state.
```
client.getPath(x_goal, y_goal);
```
Finally, the bot can be driven with the following function.
```
client.drivePath();
```
**Requirements**

The requirements are everything here:https://git.uclalemur.com/mehtank/paperbot/

Additionally, it requires installation of this library: https://github.com/jebej/MatlabWebSocket


# Miniproject Altitude control


### Session 1, 24/12/2015

__Goals of the session:__

- Form groups, find a name
- Prepare required software on your computers
- Identify and share tasks between group members
- Tests sensors, make first measures


__By the end of the session:__

- For each group, send an email with the name of the members and the name of the group


__By next session:__

- Send one document per group (~1 page) motivating choices made during the first session:
the methods used (which filter, which sensor, ...),
planning with the different tasks and the associated group member


### Session2, 01/12/2015

- First flight tests


### Session 3, 08/12/2015

- Tuning and data logging


### Session 4, 15/12/2015

- Competition, recording of video
- Presentation


_For each group, we will run 4 tests of 1 minute each (see bellow). 
The slots are 15 minutes long, the first few minutes will be to prepare the quad and run 
quick tests, and we dedicate the last 5 minutes for the tests. It means that you will have 
very little to no time to test your code before the evaluation._

__About the tests:__
* Take-off (1min): Quad on the floor, it is set to GUIDED mode with altitude command at -1 
meter. Autonomous flight until the end.
* Downward step (1min): Quad stabilized at -1 meter, the altitude command is set to -0.5 
meter. Autonomous flight until the end.
* Upward step (1min): Quad stabilized at -0.5 meter, the altitude command is set to -1 
meter. Autonomous flight until the end.
* Box (1min): Quad stabilized at -1 meter, a 30cm box is inserted under the quad. 
Autonomous flight until the end. The quad should not be disturbed by the box.

You will not be allowed to change parameters during and between the tests

For each test, we will measure:
* Rise time: ie. the time required to reach the command altitude.
* Stabilization time: the time required to reach a window of +/- 20cm around the target 
altitude and never go out.
* Overshoot: Maximum error after rise time and before stabilization time.
* Precision: Average deviation from target altitude once stabilized.
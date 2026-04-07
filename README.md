
personal drone project with the objective of learning about esp-idf and free rtos aswell as developing my embedded systems programming skills


Currently drone is in a semi working state but it is not stabalising properly on take off, im currently in the process of trying to improve how it flys and reduce oscillations but changing things like PID tuning and joystick filtering


future improvements:
- make it so button from control makes the drone leave startup mode and then also emergency stop once its in operating mode
- add LED to drone which indicates when its in start up mode
- add altitude detection so drone maintains its level instead of having to set the vertical speed using the joysticks
- also add a safety switch on the drone


no full style guide but naming convention im mostly following:
variables -> camelCase
functions -> snake_case
macros -> ALLCAPS
types -> prefix_t

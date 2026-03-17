# Failsafe command from supervisor to control
uint8 CMD_NONE = 0
uint8 CMD_HOVER = 1
uint8 CMD_LAND = 2
uint8 CMD_RTL = 3
uint8 CMD_DISARM = 4
uint8 CMD_EMERGENCY_LAND = 5

builtin_interfaces/Time stamp
uint8 command
uint8 priority           # Higher = more urgent
string reason            # Human-readable explanation

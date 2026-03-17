# System-wide health and mode status
uint8 MODE_PREFLIGHT = 0
uint8 MODE_ARMED = 1
uint8 MODE_TAKEOFF = 2
uint8 MODE_MISSION = 3
uint8 MODE_RTL = 4
uint8 MODE_HOVER = 5
uint8 MODE_LAND = 6
uint8 MODE_EMERGENCY = 7
uint8 MODE_DISARMED = 8

uint8 HEALTH_OK = 0
uint8 HEALTH_WARNING = 1
uint8 HEALTH_CRITICAL = 2
uint8 HEALTH_FAILED = 3

builtin_interfaces/Time stamp
uint8 mode
uint8 health
float32 battery_voltage
float32 battery_percentage
float32 vio_confidence
uint32 active_waypoint_id
uint32 total_waypoints
string status_message

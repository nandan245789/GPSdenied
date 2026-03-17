# Per-subsystem health report
builtin_interfaces/Time stamp
string subsystem_name    # e.g., "perception", "state_estimation"
uint8 STATUS_OK = 0
uint8 STATUS_DEGRADED = 1
uint8 STATUS_FAILED = 2
uint8 status
float32 cpu_usage        # percentage
float32 loop_time_ms     # main loop duration
string details           # optional diagnostic text

# Safety event reason codes.
uint8 REASON_UNKNOWN=0
uint8 REASON_LASER=1
uint8 REASON_TIMEOUT=2

# Time when the safety event occurred.
builtin_interfaces/Time stamp

# Stable publisher/sensor identifier. Stop conditions are tracked independently
# by source_id and reason so one source cannot clear another source's stop.
string source_id

# Whether a safety stop is required.
bool stop

# Reason code for the safety event.
uint8 reason

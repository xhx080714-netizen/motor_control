# Safety event reason codes.
uint8 REASON_UNKNOWN=0
uint8 REASON_LASER=1
uint8 REASON_TIMEOUT=2

# Time when the safety event occurred.
builtin_interfaces/Time stamp

# Whether a safety stop is required.
bool stop

# Reason code for the safety event.
uint8 reason


std_msgs/Header header

# Desired or measured RPM indexed by physical wheel position. These fields do
# not identify driver-board M1/M2 slots. No hardware mapping is implemented
# until raised-wheel tests confirm every wheel and direction.
float64 left_front
float64 left_rear
float64 right_front
float64 right_rear

# The signs are meaningful in the physical-wheel coordinate convention. This
# does not imply that the board-channel mapping has been confirmed.
bool direction_valid

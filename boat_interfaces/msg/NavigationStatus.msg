# Progress report that a navigation layer sends to the layer above it:
# guidance -> local planner -> global planner -> mission manager.
# Each report names the goal it refers to, so the receiver can ignore
# reports about a goal it no longer works on.

uint8 IDLE=0       # nothing to do: no goal or no path yet
uint8 ACTIVE=1     # working towards the goal
uint8 SUCCEEDED=2  # the goal was reached
uint8 BLOCKED=3    # cannot make progress right now, still trying (see detail)

std_msgs/Header header
uint8 state
geometry_msgs/Point goal  # meaningless when state is IDLE
string detail

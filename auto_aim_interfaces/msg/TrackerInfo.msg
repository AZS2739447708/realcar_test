# Difference between the current measurement and prediction
float64 position_diff
float64 yaw_diff
# Unfiltered position and yaw
geometry_msgs/Point position
float64 yaw
float64[] sigma2_state  # state: xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r

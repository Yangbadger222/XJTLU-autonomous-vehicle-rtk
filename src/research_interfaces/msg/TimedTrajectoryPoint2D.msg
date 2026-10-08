float64 t
float64 x
float64 y
float64 yaw
float64 v
float64 w
float64 a
float64 alpha
float64 curvature
# Curvature is undefined at zero translation. Rotation is an explicit mode,
# checked by yaw/alpha/wheel limits and the complete swept footprint.
uint8 FORWARD=0
uint8 ROTATE_IN_PLACE=1
uint8 motion_mode

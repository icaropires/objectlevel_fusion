# Enumeration
uint8 STATE_X_IDX = 0
uint8 STATE_Y_IDX = 1
uint8 STATE_VELOCITY_X_IDX = 2
uint8 STATE_VELOCITY_Y_IDX = 3
uint8 STATE_ACCELERATION_X_IDX = 4
uint8 STATE_ACCELERATION_Y_IDX = 5
uint8 STATE_YAW_IDX = 6
uint8 STATE_YAW_RATE_IDX = 7

uint8 STATE_SIZE = 8

# Size is STATE_SIZE
float32[8] state # [x, y, velocity x, velocity y, acceleration x, acceleration y, yaw, yaw rate]

# Square matrix, size is the size of state squared
# float32[64] covariation # Only 1d array supported. Access with `covariation[i*8 + j]`

# Using covariation as size of CTRA squared for now, until remodel CTRA or figure out
#   how to transform from 6x6 to 8x8 and back
float32[36] covariation

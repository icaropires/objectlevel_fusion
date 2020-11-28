float32[8] state # [x, y, velocity x, velocity y, acceleration x, acceleration y, yaw, yaw rate]

# Square matrix, total size is the size of state squared
float32[64] covariation # Only 1d array supported. Access with `covariation[i*8 + j]`

# Report

## Task 1

The observation model with motion constriant is mainly implemented in the function `CorrectErrorEstimationPoseVelCons. The state of y,z velocity has been added as the last two row of state vector. Hence, the Matrix of G and C have been changed also.

Benchmark of lidar odom

max	1.500344
mean	0.904072
median	0.894893
min	0.161967
rmse	0.918961
sse	3625.390529
std	0.164752

Benchmark of fused odom

max	1.556618
mean	0.903733
median	0.895340
min	0.046508
rmse	0.921649
sse	3646.628639
std	0.180838

From the observation, the performance doesn't change too much with the added constraints. Further parameter tuning should be done.

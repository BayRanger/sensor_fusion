## Task 1&2:

The codeblocks are filled in. The green line shows the optimized tragectory.

benchmark of optimized tragectory with imu.

max	6.798616
mean	1.220208
median	0.873224
min	0.118435
rmse	1.601749
sse	4892.595922
std	1.037637

benchmark of lidar odometry with imu.

max	34.013475
mean	15.327241
median	15.077945
min	0.000001
rmse	17.886259
sse	610084.133610
std	9.219216


<img src="doc/images/map.png" alt="Terminator" width="100%">

<img src="doc/images/odom.png" alt="Terminator" width="100%">

## Task 2

After disabling the usage of IMU, the result has also been obviously improved after optimization. From the observation of the map, the improvement could not been easily observed...

optimized benchmark

max	1.375001
mean	0.358914
median	0.332925
min	0.023321
rmse	0.413774
sse	327.693883
std	0.205887

lidar benchmark

max	21.144567
mean	10.719335
median	11.076510
min	0.000001
rmse	12.006945
sse	275935.116542
std	5.409490

<img src="doc/images/map2.png" alt="Terminator" width="100%"> 
<img src="doc/images/odom2.png" alt="Terminator" width="100%">

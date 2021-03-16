## The parameter tuning

Regarding the experience of paramter tuning, when we fix Q and increase R, it means we trust less on the measurements, since we assume there is much noise on the measurement.

On the other hand, when we decrease R or increase Q, it means we trust less on the measurement, in our case, it means we assume we trust more on the prediction of IMU compared with the measurement from lidar.

Based on the experiments on the Kitti data, when I set R to a pretty small value, and increase Q, the performance gets better. It means the measurement from lidar is accurate compared with the prediction from IMU.

The following tests show the result, and parameter 3 shows the best performance, in which Q is pretty large compared with R.

**Parameter 1**

```
        process:
            gyro: 1.0e-4
            accel: 2.5e-3
            bias_accel: 2.5e-3
            bias_gyro: 1.0e-4
        measurement:
            pose:
                pos: 1.0e-3
                ori: 1.0e-4
            pos: 1.0e-4  
            vel: 2.5e-3
```

1. The result analysis of  laser data

max	1.912693
mean	0.899127
median	0.865390
min	0.415304
rmse	0.916796
sse	2942.641849
std	0.179125

2. The result of untuned filter data

max	1.891277
mean	0.899247
median	0.862092
min	0.419038
rmse	0.917990
sse	2950.309768
std	0.184551

**parameter 2**

```
        process:
            gyro: 1.0e-3
            accel: 2.5e-2
            bias_accel: 2.5e-2
            bias_gyro: 1.0e-3
        measurement:
            pose:
                pos: 1.0e-3
                ori: 1.0e-4
            pos: 1.0e-4  
            vel: 2.5e-3
```

3. The result of laser data

max	1.136680
mean	0.229616
median	0.163051
min	0.017465
rmse	0.287662
sse	349.865310
std	0.173281

4. the result of filter data

max	1.160671
mean	0.240456
median	0.181685
min	0.012108
rmse	0.295216
sse	368.481544
std	0.171270

**parameter 3**

```
        process:
            gyro: 1.0e-2
            accel: 2.5e-1
            bias_accel: 2.5e-1
            bias_gyro: 1.0e-2
        measurement:
            pose:
                pos: 1.0e-3
                ori: 1.0e-4
            pos: 1.0e-4  
            vel: 2.5e-3
```

5. the result of laser

max	1.500344
mean	0.902015
median	0.893106
min	0.161967
rmse	0.917140
sse	3584.959773
std	0.165872

6. the result of filter data

max	1.513724
mean	0.901709
median	0.897900
min	0.055016
rmse	0.918151
sse	3592.871413
std	0.172981

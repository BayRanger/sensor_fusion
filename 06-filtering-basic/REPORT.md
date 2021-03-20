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

without gyro and accel

max	1.500344
mean	0.900750
median	0.893803
min	0.161967
rmse	0.916304
sse	3377.763876
std	0.168113

2. The result of untuned filter data

max	1.891277
mean	0.899247
median	0.862092
min	0.419038
rmse	0.917990
sse	2950.309768
std	0.184551

without gyro and accel

max	1.748696
mean	0.909889
median	0.905628
min	0.059124
rmse	0.933167
sse	3503.227476
std	0.207127

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

Without accl and gyro bias

max	1.500344
mean	0.900558
median	0.893220
min	0.161967
rmse	0.915732
sse	3596.603998
std	0.166014

4. the result of filter data

max	1.160671
mean	0.240456
median	0.181685
min	0.012108
rmse	0.295216
sse	368.481544
std	0.171270

Without accl and gyro bias

max	1.625812
mean	0.913063
median	0.903699
min	0.059087
rmse	0.941496
sse	3801.836272
std	0.229632

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

without gyro and accel bias

max	1.500344
mean	0.902623
median	0.894268
min	0.161967
rmse	0.917727
sse	3672.929950
std	0.165810

6. the result of filter data

max	1.513724
mean	0.901709
median	0.897900
min	0.055016
rmse	0.918151
sse	3592.871413
std	0.172981

without gyro and accel bias

max	1.399959
mean	0.791512
median	0.770033
min	0.058737
rmse	0.812926
sse	2881.961358
std	0.185360

### Task 3

To erase the gyro and accelerator from the state, I have changed the dimension of vector **X**, Matrix **P**, **B**,**F** and** G** in the code. But, a more smarter way should be simply set the relevent entry as zero so as to fit the previous model.

Based on the provided code base, the accl bias and gyro bias will not update if the corresponding P  entry is larger than e-5. From my understanding, the intention is that if the intial noise of gyro and acclerator exists, we will not consider its impact on the state vector as it is not accurate. Theoritically, their bais should be in consideration since it could have some influence on the state of the car. But, after the comparison of the state vector with and without gyro and accel bias, I would say it does not make much difference.

## Experiment


| window size | 5 | 10 | 20 | 40 | X |
| - | - | - | - | - | - |
| Improvement | 0.54 | 0.28 | 0.65 | 0.449 / 0.37 |   |

Conclusion: The experiment data is shown above, incresing the window size would increase the working load and time of the computer, but could not improve efficiency sometimes.

The table shows best performance appears when the window size is 20, and from opinion, window size 5 provides a enough good output.

### Sliding Window Size vs Performance

slide window size 40:

Lidar Odom:

max      3.767447
mean      1.803930
median      1.851701
min      0.000001
rmse      1.956166
sse      17326.787012
std      0.756587

Optimized:

max      3.755586
mean      1.354122
median      1.285346
min      0.000001
rmse      1.544220
sse      10797.539279
std      0.742273

slide window size 20:

Lidar Odom:

max      3.767447
mean      1.803930
median      1.851701
min      0.000001
rmse      1.956166
sse      17326.787012
std      0.756587

Optimized:

max      3.767447
mean      1.150426
median      1.114598
min      0.000001
rmse      1.309444
sse      7763.906568
std      0.625430

Window Length: 40

Lidar Odom

max      4.003096
mean      1.935797
median      1.987574
min      0.000001
rmse      2.099044
sse      19954.702430
std      0.811588

Optimized Odom

max      4.027566
mean      1.569013
median      1.543497
min      0.000001
rmse      1.738434
sse      13687.325908
std      0.748565

Window Length: 10

Lidar Odom

max      3.414863
mean      1.599265
median      1.642315
min      0.000001
rmse      1.733028
sse      13596.322588
std      0.667634

optimized

max      3.402521
mean      1.319541
median      1.305639
min      0.000001
rmse      1.459341
sse      9641.038050
std      0.623287

WIndow Length: 5

Lidar Odom

max      3.414863
mean      1.599265
median      1.642315
min      0.000001
rmse      1.733028
sse      13596.322588
std      0.667634

optimized

max      3.421523
mean      1.057977
median      0.959164
min      0.000001
rmse      1.201034
sse      6530.118192
std      0.568478

From the observation, the mean error is smller by 0.6 after optimization.

### Compared with EKF

Compared with the ESKF performance in the charpter 8, the optimization-based method gives a more obvious improvement.

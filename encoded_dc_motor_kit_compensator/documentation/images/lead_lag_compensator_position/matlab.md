# MATLAB CODE 
Here is the matlab code for the step test:

```matlab
clc;
clear;
num = [0.755];
den = [1, 13.87, 34.91, 0];
gs = tf(num, den)

num_lc = [1, 0.08097];
den_lc = [1, 0.00705];
gs_lc = tf(num_lc, den_lc);

num_lc = [1, 19.55];
den_lc = [1, 25.01];
gs_lc_2 = tf(num_lc, den_lc);

k = 216.88;
gs_lc = series( gs_lc, gs_lc_2)
gs_lc = series(k, gs_lc);

gs_top = series(gs_lc, gs);
gs_total = feedback(gs_top, 1);

% Display step info
disp('Step Info using step');
stepinfo(gs_total);

% Discretize gs_lc
gs_lc_discrete = c2d(gs_lc, 0.01, 'tustin')
```

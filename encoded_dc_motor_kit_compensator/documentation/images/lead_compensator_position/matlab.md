# MATLAB CODE 
Here is the matlab code for the step test:

```matlab
clc;
clear;
num = [0.755];
den = [1, 13.87, 34.91, 0];
gs = tf(num, den)

num_lc = [1, 11.55];
den_lc = [1, 50.72];
gs_lc = tf(num_lc, den_lc);
k = 378.07;
gs_lc = series(k, gs_lc);

gs_top = series(gs_lc, gs);
gs_total = feedback(gs_top, 1);

% Display step info
disp('Step Info using step');
stepinfo(gs_total);

% Discretize gs_lc
gs_lc_discrete = c2d(gs_lc, 0.01, 'tustin')
```

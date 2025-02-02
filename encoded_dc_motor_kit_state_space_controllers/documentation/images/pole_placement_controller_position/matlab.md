# MATLAB CODE 
Here is the matlab code for the step test:

```matlab
clc;
clear;
%%DESIGN OF SERVO SYSTEMS
%%Design of Type 1 Servo System when the Plant Has An Integrator.
num = 0.755;
den = [1, 13.87, 34.91];
A = [0  1         0;
     0  0         1;
     0 -34.91  -13.87];

B = [0  ;  0; 0.755];

C = [1 0 0];


P = [A     B;
     -C    0];
rank(P);%%has rand n+1 thus the system is completely statecontrollable
A_hat = [ A          zeros( 3, 1); 
         -C          0];
B_hat = [B;
         0];
J = [(-2+j*4) (-2-j*4) (-1) 0.3];

Khat = acker( A_hat, B_hat, J)
K = [ Khat(1) Khat(2) Khat(3)];
K_1 = Khat(4);
AA = [A-B*K   B*K_1;
      -C      0];
BB = [ 0; 0; 0; 1];
CC = [0 C ];
DD = [0];
rlocus( AA, BB, CC, DD);
sys = ss( AA, BB, CC, DD);
pole( sys);

%***** To obtain response curves x1 versus t, x2 versus t,
% x3 versus t, x4 versus t, and x5 versus t, separately, enter
% the following command *****
figure;
t = 0:0.02:6;
[y,x,t] = step(AA,BB,CC,DD,1,t);
x1 = [1 0 0 0]*x';%%position
x2 = [0 1 0 0]*x';%%velocity
x3 = [0 0 1 0]*x';%%acceration
x4 = [0 0 0 1]*x';%%error
subplot(3,2,1); plot(t,x1); grid
title('x1 versus t')
xlabel('t Sec'); ylabel('x1')
subplot(3,2,2); plot(t,x2); grid
title('x2 versus t')
xlabel('t Sec'); ylabel('x2')
subplot(3,2,3); plot(t,x3); grid
title('x3 versus t')
xlabel('t Sec'); ylabel('x3')
```


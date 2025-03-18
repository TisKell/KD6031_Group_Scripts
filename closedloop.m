
pi = 3.14;
rad = 3.13 / 200;  % m
rho = 1000;  % kg/m3
g = 9.81;  % m/s
mu = 1.002e-3;
L = 4 / 100;  % m
A_t = 0.44;  % m2

% Constant k for matrix A
k = (pi * rad^4 * rho * g)/(8 * mu * L);

A = k/A_t * [-1    0;
              1   -1];
B = 1/A_t * [0;
             1];
C = [0    1/A_t;
     0  -k/A_t^2];
D = [0];

% tank heights
states = {'h1' 'h2'};
% pump flow rate
inputs = {'u'};
% tank 2 pressure
outputs = {'p_2'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

poles = eig(A);

% Set up the Q and R matrices for LQR to tune the system
Q = 75e-3 * C'*C;

R = 1;

% Compute the LQR gain
K = lqr(A, B, Q, R);

% Update closed-loop system with LQR control
Ac = [(A - B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

% tank heights
states = {'h1' 'h2'};
% pump flow rate
inputs = {'u'};
% tank 2 pressure
outputs = {'p_2'};

% Define new system
sys_cl = ss(Ac, Bc, Cc, Dc, 'statename', states, 'inputname', inputs, 'outputname', outputs);

% Simulate the system to check the response
t = 0:0.01:1;
r = 0.45 * ones(size(t));
[y, t, x] = lsim(sys_cl, r, t);

% Plot the response
plot(t, x(:,2))
xlabel('Time / s');
ylabel('Height / m');

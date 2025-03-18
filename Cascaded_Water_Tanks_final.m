% Group Assignment - Control of Water Level for Two Cascading Tanks
% Module Code: KD6031
% Module Title: Instrumentation and Control of Dynamical Systems
% Student Names: Amy Kell       - 19004417
%                Thomas Gardner - 23040363
%                Khaled Ibrahim - 22023622 
%                Constantin Ott - 24056675

%% Reset the System
clear all;
close all;

%% Generall Constants

g = 9.81;                               % Gravitational constant [m/s^2]
eta = 1.002e-3;                         % Viscosity of water [Pa*s]
q_in = 3.4e-3;                          % Waterflow in Tank 1 [m^3/s]
sse_req = 1;                            % Max. required Steady state error [%]
Ts = 600;                               % Settling Time [s]

%% Constants Tanks
d_tank1 = 0.75;                         % Diameter Tank 1 [m]
d_tank2 = 0.75;                         % Diameter Tank 2 [m]

r_tank1 = d_tank1/2;                    % Radius Tank 1 [m]
r_tank2 = d_tank2/2;                    % Radius Tank 2 [m]

A_tank1 = pi*r_tank1^2;                 % Cross-sectional area Tank 1 [m^2]
A_tank2 = pi*r_tank2^2;                 % Cross-sectional area Tank 2 [m^2]

%% Constants Pipes
d_pipe1 = 3.13e-2;                      % Diameter Pipe 1 [m]
d_pipe2 = 3.13e-2;                      % Diameter Pipe 2 [m]

r_pipe1 = d_pipe1/2;                    % Radius Tank 1 [m]
r_pipe2 = d_pipe2/2;                    % Radius Tank 2 [m]

L_pipe1 = 4e-2;                         % Length Pipe 1 [m]
L_pipe2 = 4e-2;                         % Length Pipe 2 [m]

A_pipe1 = pi*r_pipe1^2;                 % Cross-sectional area Pipe 1 [m^2]
A_pipe2 = pi*r_pipe2^2;                 % Cross-sectional area Pipe 2 [m^2]

R_pipe1 = 8*eta*L_pipe1/(pi*r_pipe1^4);  % Resistance Laminar flow Pipe 1 [kg/(s*m^4)]
R_pipe2 = 8*eta*L_pipe2/(pi*r_pipe2^4);  % Resistance Laminar flow Pipe 2 [kg/(s*m^4)]

%% Constants Transferfunction (polynomial form)
% Derivative of a0, b0, b1, b2 is stated in the documentation on page 9-11
a0 = R_pipe2;

b0 = 1;
b1 = A_tank1*R_pipe1 + A_tank2*R_pipe2;
b2 = A_tank1*A_tank2*R_pipe1*R_pipe2;

%% Definition of System Transferfunction

G_p = tf(a0, [b2 b1 b0]);               % Transferfunction of the plant (PT2-Transferfunction)

%% Step Response of the Open-Loop System
figure('Name', 'Step-Response Open-Loop','NumberTitle','off')
step(q_in*G_p);grid;
xlabel('Time t in s'); ylabel('Height h2 in m');

%% Step Response of the Closed-Loop System
figure('Name', 'Step-Response Closed-Loop', 'NumberTitle','off')
G_r = feedback(G_p,1);                   % Transfer function closed-loop without controller
step(q_in*G_r);grid
xlabel('Time t in s'); ylabel('Height h2 in m');

%% Step Response with feedback gain
% Derivative of feedback gain is stated in the documentation on page 15 - 17
H = 0.0034/0.45;
figure('Name', 'Step-Response Closed-Loop feedback gain', 'NumberTitle','off')
G_r = feedback(G_p,H);
step(q_in*G_r);grid;
xlabel('Time t in s'), ylabel('Height h2 in m');

%% Calculate Controll parameters
% Exact derivative and the referances are stated in the documentation on
% page 19 - 26
xi = sqrt(log(sse_req/100)^2/(log(sse_req/100)^2 + pi^2));

omega_n = 4/(600*xi);

s1 = -omega_n*xi + omega_n*sqrt(1-xi^2)*1i;
s2 = -omega_n*xi - omega_n*sqrt(1-xi^2)*1i;
sp = pole(G_p);

figure('Name', 'Pole-Zero Map', 'NumberTitle','off')
plot(real(s1),imag(s1),'rx',real(s2),imag(s2),'rx',real(sp(1)),imag(sp(1)),'rx',real(sp(2)),imag(sp(2)),'rx', 'LineWidth',2 );grid

phi1 = 180 - rad2deg(atan2(abs(imag(s1)),(abs(real(s1))-abs(real(sp(1))))));
phi2 = 180 - rad2deg(atan2(abs(imag(s2)),(abs(real(s2))-abs(real(sp(2))))));
phi_i = 180 - rad2deg(atan2(abs(imag(s2)),abs(real(s2))));
phi_pd = -180 + phi1 + phi2;

z_pd = abs(real(s1)) + abs(imag(s1))/tan(deg2rad(phi_pd));  
k = abs(abs(s1-sp(1))*abs(s1-sp(2))/(a0*abs(s1+z_pd)));

kd = k; kd=kd*10^8;
kp = k*z_pd/2.6; kp = kp * 10^8;
ki = k*3.129e-4*z_pd; ki = ki * 10^8;


%% Control Cycle

G_c = tf([kd kp ki],[1 0]);             % Transfer function PID-Controller

G_r = feedback(G_c*G_p,H);              % Transfer function closed-loop with controller
figure('Name','Step-Response Controlled System','NumberTitle','off')
step(q_in*G_r,[0 600]);ylabel('Height h2 in m ');grid;

%% Stability
figure('Name','Stability Controlled System', 'NumberTitle','off')
pzmap(G_r); grid;
p_r = pole(G_r);                        % Calculates the poles of the controlled system
if (real(p_r(1)) < 0 && real(p_r(2)) && real(p_r(3)) < 0)
  fprintf('The system is stable. The poles are located at %s , %s + %si and %s + %si \n \n',p_r(1), p_r(2), imag(p_r(2)), p_r(3), imag(p_r(2)));
end

%% Steady-State error
[y t] = step(q_in*G_r, [0 600]);
sse = 0.45 - y(end);
figure('Name', 'steady-state error', 'NumberTitle','off');
plot(t,y,'k',[0 600], [y(end) y(end)],'b--', [0 600],[0.45 0.45], 'r--');grid
xlabel('Time t in s'); ylabel('Height h2 in m');
axis([0 600 0 0.5]);
xlabel('Time t in [s]')
ylabel('Height h2 in [m] ');
legend('Step-Response', 'Actual steady state', 'Desired steady state')
fprintf('The stead-state error is %f percent \n \n',(0.45-y(end))*100)

% End of script
clear all
close all
clc

%% constants
DT1     = .01;          % 100hz controller loop
DT2     = .05;          % 20hz outer controller loop
m_w     = 27e-3;        % mass of one wheel in Kg 
m_b     = .180 - 2*m_w; % Mip body mass without wheels 
R       = 34e-3;        % radius of wheel in m
L       = 47.7e-3;      % center of wheel to Center of mass
I_b     = 0.000263;     % Inertia of body about wheel axis Kg*m^2
g       = 9.81;         % gravity m/s^2
G       = 35.5555;      % gearbox ratio
tau_s   = 0.003;        % Motor armature stall Torque @ V_nominal
w_f     = 1760;         % Motor armature free run speed at nominal
V_b     = 7.4;          % motor nominal drive voltage
I_m     = 3.6e-8;       % inertia of one motor armature kg*m^2

% add gearbox effect for motor inertia at gearbox output
I_gb = I_m*G^2;

% add inertia of wheels modeled as disks and times to for both sides
I_w = 2 * (I_gb+((m_w*R^2)/2));

% motor equation used: t = e*u - f*w
e = 2*G*tau_s;        % stall torque of two motors
f = 2*G*G*tau_s/w_f;  % constant provides zero torque @ free run

%% inner loop plant including motor dynamics
a = I_w + (m_b + m_w)*R^2;
b = m_b * R * L;
c = I_b + m_b*L^2;
d = m_b * g * L;
 
numG1 = [-e*(a+b), 0];
% denG1 = [(a*c + b^2), f*(b+c), -a*d, -f*d];
denG1 = [(a*c - b^2), f*(a+2*b+c), -a*d, -f*d];
% Make TF monic 
numG1 = (1/denG1(1))*numG1; 
denG1 = (1/denG1(1))*denG1;
%make the TF
G1 = tf(numG1,denG1);
G1
disp('G1 poles')
roots(denG1)

%% outer loop plant
numG2 = [-(b*c), 0, d];
denG2 = [a+b, 0,0];
% make monic
numG2 = (1/denG2(1))*numG2;
denG2 = (1/denG2(1))*denG2;
% make the TF
G2=tf(numG2,denG2);
G2

%% D1 Design

% figure(50)
% rlocus(G1)
% figure(51)
% bode(G1)

numD1 = conv([1, 6.4742],[1, 22.0258]);
denD1 = conv([1, 0.0386],[1, 73.1114]);

D1 = tf(numD1,denD1);
K1 = -4.95;

D1 = K1*D1;
% 
L1 = G1*D1;
% 
% figure(52)
% rlocus(L1)
% figure(53)
% margin(L1)
% figure(54)
% bode(L1/(1+L1))
figure(55)
step(L1/(1+L1))

D1z = c2d(D1,DT1)

%% D2 Design
% 
% figure(60)
% rlocus(-G2)
% figure(61)
% bode(-G2)

numD2 = [1, 0.1];
denD2 = [1, 10];

D2 = tf(numD2,denD2);
K2 = 1;

D2 = K2*D2;

L2 = G2*D2;
% 
% figure(62)
% rlocus(L2)
% figure(63)
% margin(L2)
% figure(64)
% bode(L2/(1+L2))
% figure(65)
% step(L2/(1+L2))

T1 = L1/(1+L1);

% figure(66)
% step(L2*T1/(1+L2*T1))
% figure(67)
% margin(L2*T1/(1+L2*T1))

D2z = c2d(D2,DT2)

%%

% D1 = d2c(tf([-4.9445, 8.862, -3.967],[1, -1.481, 0.4812], 0.01));
% D2 = d2c(tf([0.1636, -0.16289],[1, -0.6326], 0.05));
% 
% L1 = G1*D1;
% L2 = G2*D2;
% 
% T1 = L1/(1+L1);
% 
% figure(70)
% step(L2*T1/(1+L2*T1))

%%
% 
% step()
% pade()
% bode()
% rlocus()
% tf()
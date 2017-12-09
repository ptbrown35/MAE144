%% constants
DT      = .005;         % 100hz controller loop
DT2     = .01;          % 50hz outer controller loop
m_w     = .027;         % mass of one wheel in Kg 
m_b     = .180 - 2*m_w; % Mip body mass without wheels 
R       = .034;         % radius of wheel in m
L       = 0.0477;       % center of wheel to Center of mass
I_r     = 0.0004;       % Inertia of body about wheel axis Kg*m^2
g       = 9.81;         % gravity m/s^2
R_gb    = 35.5555;      % gearbox ratio
tau_s   = 0.003;        % Motor armature stall Torque @ V_nominal
w_free  = 1760;         % Motor armature free run speed at nominal
V_n     = 7.4;          % motor nominal drive voltage
I_arm   = 3.6*10^-8;    % inertia of one motor armature kg*m^2

% add gearbox effect for motor inertia at gearbox output
I_gb = I_arm*R_gb^2;
 
% add inertia of wheels modeled as disks and times to for both sides
I_w = 2 * (I_gb+(m_w*R^2)/2);
 
% motor equation used: t = e*u - f*w
e = 2 * tau_s * R_gb; % stall torque of two motors
f  = e / (w_free/R_gb);   % constant provides zero torque @ free run
 
%% inner loop plant including motor dynamics
b = m_b * R * L;
c = I_r + m_b*L^2;
d = m_b * g * L;
a = I_w + (m_b + m_w)*R^2;
 
numG1 = [-e*(a+b), 0];
denG1 = [(a*c + b^2), f*(b+c), -a*d, -f*d];
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

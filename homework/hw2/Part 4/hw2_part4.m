clear all
close all
clc

%%
load('data3.mat')

%%

time = 1/100 * [0:length(theta_a)-1]';
figure(1)
plot(time,theta_a)
hold on
plot(time,theta_g)
plot(time,theta_f)

legend('theta\_a', 'theta\_g', 'theta\_f', 'Location', 'NorthEast')
grid on
xlabel('time [s]')
ylabel('theta [rad]')

saveas(figure(1),'data3.jpg')
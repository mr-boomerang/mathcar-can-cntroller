clear all;
close all;
% load data
load('matlab_08_56.mat');
% get size of data
x=size(wheel_l_speed_filtered);
% get total speed (left+right)/2 and convert to kmph
total_speed=wheel_r_speed_filtered+wheel_l_speed_filtered;
total_speed=total_speed*18/5/2;

% plot
figure(1);
hold on;
grid on;
plot(total_speed,'b');
plot(gateway_speed*18/5,'r');

% helpful info
legend('encoder speed','gateway speed');
xlabel('time each sample at 10 ms total run ~9 min');
ylabel('speedin kmph');
title('controller velocity from gateway vs encoder');
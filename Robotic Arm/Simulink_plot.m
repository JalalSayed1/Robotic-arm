
figure(1)										% define figure window number
clf												% clear figure
hold on
% grid on

plot(tout, theta_M_out) % theta M
plot(tout,theta_F_out) % theta F
plot(tout,theta_Fdot_out) % theta F dot

title("Simulink response")
%       theta M         theta F         theta F dot
legend('Motor angle', 'Forearm angle', 'Speed of forearm')
xlabel('Time (s)')
ylabel('Radians')

hold off
%%% PLOT THE PID STEP RESPONSE

% T = 0.01;
% Ts = 0.00001;
% t = 0:Ts:T;

% model = sim('Impedance_Gantry_Model_V1.slx')

% test = model.response.Data();
% 
% plot(t,test)
% xlabel('Time (s)')
% ylabel('Amplitude')
% title('Step response of Gripper PID position controller')

%%% Plot Input for gripper and z axis

T = 5;
Ts = 0.0125;
t = 0:Ts:T;

model = sim('Impedance_Gantry_Model_V1.slx')

test = model.response.Data();

plot(t,test)
xlabel('Time (s)')
ylabel('Amplitude')
title('Z axis input')
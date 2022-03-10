%%% Plot Normal Force

T = 5;
Ts = 0.0125;
t = 0:Ts:T;

model = sim('Impedance_Gantry_Model_V2.slx')

test = model.FNormal.Data();

plot(t,test)
xlabel('Time (s)')
ylabel('Displacement (m)')
title('Z axis initial profile')
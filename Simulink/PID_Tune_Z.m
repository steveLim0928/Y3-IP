load z_axis_model.mat;

emin = 9999;
kpval = linspace(1650,1750,3000);
kdval = linspace(1,20,100);
kp = 0;
kd = 0;

T = 0.3;
Ts = 0.001;
t = 0:Ts:T;

for p = kpval
    for d = kdval
        tfz = tf([1],[0.16875,d,p]);
        y = step(tfz,t);
        y_ideal = impulse(syscz,t);
        error = norm(y_ideal-y);
        if error < emin
            emin = error;
            kp = p;
            kd = d;
        end
    end
end

emin
tfz = tf([1],[0.16875,kd,kp]);
y = step(tfz,t);
plot(t,y);
hold on
plot(t,y_ideal);
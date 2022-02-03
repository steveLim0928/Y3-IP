load y_axis_model.mat;

emin = 9999;
kpval = linspace(1000,10000,1000);
kdval = linspace(10,100 ,100);
kp = 0;
kd = 0;

T = 0.3;
Ts = 0.001;
t = 0:Ts:T;

for p = kpval
    for d = kdval
        tfy = tf([1],[1.85625,d,p]);
        y = step(tfy,t);
        y_ideal = impulse(syscy,t);
        error = norm(y_ideal-y);
        if error < emin
            emin = error;
            kp = p;
            kd = d;
        end
    end
end

emin
tfy = tf([1],[1.85625,kd,kp]);
y = step(tfy,t);
plot(t,y);
hold on
plot(t,y_ideal);
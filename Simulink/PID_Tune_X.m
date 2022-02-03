load x-axis_7th_order_model.mat;

emin = 9999;
kpval = linspace(1,10000,1000);
kdval = linspace(1,100 ,100);
kp = 0;
kd = 0;

T = 0.3;
Ts = 0.001;
t = 0:Ts:T;

for p = kpval
    for d = kdval
        tfx = tf([1000],[6.17625,d,p]);
        x = step(tfx,t);
        x_ideal = impulse(sysc,t);
        error = norm(x_ideal-x);
        if error < emin
            emin = error;
            kp = p;
            kd = d;
        end
    end
end

emin
tfx = tf([1000],[6.17625,kd,kp]);
x = step(tfx,t);
plot(t,x);
hold on
plot(t,x_ideal);
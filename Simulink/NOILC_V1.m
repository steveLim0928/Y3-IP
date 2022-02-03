close all;
clear;
clc;

load x-axis_7th_order_model.mat;
load y_axis_model.mat;
load z_axis_model.mat;
load trajectories_30upm_100Hz_no_offset.mat;

% tracking reference
refx = x_profile_30upm(:,2);
refy = y_profile_30upm(:,2);
refz = z_profile_30upm(:,2);

T = 2;
Ts = 0.01;

N = 50;

t = 0:Ts:T;
t(1) = []; % remove t = 0

% Mass for each axis independently
mx = 0.64*0.05*0.05*2700;
my = 0.05*0.52*0.05*2700;
mz = 0.025*0.025*0.1*2700;
%% 

% PD val for each axis to mimic actual TF
kpx = 27146.6747627399;
kdx = 1020.02839756073;
kpy = 131139.225952649;
kdy = 1480.57986826149;
kpz = 33734.3806;
kdz = 163.3421;
%% 

% Define model tf
 TFx = tf([kdx kpx], [(mx+my+mz)*12.345679 kdx*12.345679 kpx*12.345679]);
 TFy = tf([kdy kpy], [(my+mz)*487.8 kdy*487.8 kpy*487.8]);
 TFz = tf([kdz kpz], [mz*1680.672269 kdz*1680.672269 kpz*1680.672269]);
%% 

% discretise system
 sysdx = ss(c2d(TFx,Ts));
 sysdy = ss(c2d(TFy,Ts));
 sysdz = ss(c2d(TFz,Ts));

% discretise system
% sysdx = ss(c2d(sysc,Ts));
% sysdy = ss(c2d(syscy,Ts));
% sysdz = ss(c2d(syscz,Ts));

% create G matrix
g_x = impulse(sysdx,T);
g_x(1) = []; % remove the diagonal 
%g_x(end) = [];
g_x = g_x*Ts;
G_x = toeplitz(g_x,g_x*0); % transfer to matrix with diagonal = CB

g_y = impulse(sysdy,T);
g_y(1) = []; % remove the diagonal 0
g_y = g_y*Ts;
G_y = toeplitz(g_y,g_y*0); % transfer to matrix with diagonal = CB

g_z = impulse(sysdz,T);
g_z(1) = []; % remove the diagonal 0
g_z = g_z*Ts;
G_z = toeplitz(g_z,g_z*0); % transfer to matrix with diagonal = CB

% ILC weights 
Rx = eye(size(G_x))*0.1; 
Qx = eye(size(G_x))*30;

Ry = eye(size(G_y))*0.001; 
Qy = eye(size(G_y))*400;

Rz = eye(size(G_z))*0.0001; 
Qz = eye(size(G_z))*1500;


% create vectors to store results 
u_x = 0*t'; % start from zero. Could start from ref, but results worse
%e_x = refx - G_x*u_x;
%enorm0_x = norm(e_x) %initial error
enorm_x = zeros(N,1); %create N-by-1 matrix of all 0

u_y = 0*t'; % start from zero. Could start from ref, but results worse
%e_y = refy - G_y*u_y;
%enorm0_y = norm(e_y) %initial error
enorm_y = zeros(N,1); %create N-by-1 matrix of all 0

u_z = 0*t'; % start from zero. Could start from ref, but results worse
%e_z = refz - G_z*u_z;
%enorm0_z = norm(e_z) %initial error
enorm_z = zeros(N,1); %create N-by-1 matrix of all 0

%noise = 0.0001*sin(2*pi*0.5*t);
%noise = 0.0001;


for i=1:N
    i
    if(0) %run uing lsim
        yx = lsim(TFx,u_x,t);
        yy = lsim(TFy,u_y,t);
        yz = lsim(TFz,u_z,t);
    elseif(0)
        yx = G_x*u_x;
        yy = G_y*u_y;
        yz = G_z*u_z;
    else
       % model = sim()
        inputx = timeseries(u_x,t);
        inputy = timeseries(u_y,t);
        inputz = timeseries(u_z,t);

        model = sim('Gantry_Model_V1.slx')

        yx = model.outputx.Data();
        yx(1) = [];
        yy = model.outputy.Data();
        yy(1) = [];
        yz = model.outputz.Data();
        yz(1) = [];
       
    end

    e_x = refx - yx;
    e_y = refy - yy;
    e_z = -refz - yz;

    enorm_x(i) = norm(e_x);
    enorm_y(i) = norm(e_y);
    enorm_z(i) = norm(e_z);
    
    u_x  = u_x + inv(Rx+Qx*(G_x'*G_x))*Qx*G_x'*e_x;
%     u_y  = u_y + inv(Ry+Qy*(G_y'*G_y))*Qy*G_y'*e_y;
    u_y  = u_y + inv(Ry+G_y'*Qy*G_y)*G_y'* Qy*e_y;
    u_z  = u_z + inv(Rz+Qz*(G_z'*G_z))*Qz*G_z'*e_z;

   % u = u + noise;
    

%     e_x = refx - G_x*u_x;
%     e_y = refy - G_y*u_y;
%     e_z = refz - G_z*u_z;
    

end

final_enorm_x = norm(e_x)
enorm_x(1)
accuracy_x = enorm_x(25)/enorm_x(1)*100
final_enorm_y = norm(e_y)
enorm_y(1)
accuracy_y = enorm_y(25)/enorm_y(1)*100
final_enorm_z = norm(e_z)
enorm_z(1)
accuracy_z = enorm_z(25)/enorm_z(1)*100

subplot(3,3,1)
plot(1:N,enorm_x); xlabel('Trial, k'); ylabel('Error Norm'); title('x axis');
subplot(3,3,2)
plot(1:N,enorm_y); xlabel('Trial, k'); ylabel('Error Norm'); title('y axis');
subplot(3,3,3)
plot(1:N,enorm_z); xlabel('Trial, k'); ylabel('Error Norm'); title('z axis');
% subplot(3,2,4)
% plot(t,u_z); xlabel('Time (s)'); ylabel('u');
% subplot(3,2,2)
% semilogy(1:N,enorm_x); xlabel('Trial, k'); ylabel('Error Norm');
% subplot(3,2,3)
% plot(t,u_x); xlabel('Time (s)'); ylabel('u');
% subplot(3,2,4)
% plot(t,e_x); xlabel('Time (s)'); ylabel('e');
% subplot(3,2,5)
% plot(t,refx); xlabel('Time (s)'); ylabel('refx');
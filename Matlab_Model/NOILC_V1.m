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

N = 100;

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
% TFx = tf([kdx kpx], [mx+my+mz kdx kpx]);
% TFy = tf([kdy kpy], [my+mz kdy kpy]);
% TFz = tf([kdz kpz], [mz kdz kpz]);
%% 

% discretise system
% sysdx = ss(c2d(TFx,Ts));
% sysdy = ss(c2d(TFy,Ts));
% sysdz = ss(c2d(TFz,Ts));

% discretise system
sysdx = ss(c2d(sysc,Ts));
sysdy = ss(c2d(syscy,Ts));
sysdz = ss(c2d(syscz,Ts));

% create G matrix
g_x = impulse(sysdx,T);
%g_x(1) = []; % remove the diagonal 
g_x(end) = [];
G_x = toeplitz(g_x,g_x*0); % transfer to matrix with diagonal = CB

g_y = impulse(sysdy,T);
g_y(1) = []; % remove the diagonal 0
G_y = toeplitz(g_y,g_y*0); % transfer to matrix with diagonal = CB

g_z = impulse(sysdz,T);
g_z(1) = []; % remove the diagonal 0
G_z = toeplitz(g_z,g_z*0); % transfer to matrix with diagonal = CB

% ILC weights 
R = eye(size(G_x))*0.01; 
Q = eye(size(G_x));


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

    if(1) %run uing lsim
        yx = lsim(sysc,u_x,t);
        yy = lsim(syscy,u_y,t);
        yz = lsim(syscz,u_z,t);
    else
       % model = sim()
       
    end

    e_x = refx - yx;
    e_y = refy - yy;
    e_z = refz - yz;

    enorm_x(i) = norm(e_x);
    enorm_y(i) = norm(e_y);
    enorm_z(i) = norm(e_z);
    
    u_x  = u_x + inv(R+Q*(G_x'*G_x))*Q*G_x'*e_x;
    u_y  = u_y + inv(R+Q*(G_y'*G_y))*Q*G_y'*e_y;
    u_z  = u_z + inv(R+Q*(G_z'*G_z))*Q*G_z'*e_z;

   % u = u + noise;
    

%     e_x = refx - G_x*u_x;
%     e_y = refy - G_y*u_y;
%     e_z = refz - G_z*u_z;
    

end

final_enorm_x = norm(e_x)
final_enorm_y = norm(e_y)
final_enorm_z = norm(e_z)

subplot(3,2,1)
plot(1:N,enorm_x); xlabel('Trial, k'); ylabel('Error Norm');
subplot(3,2,2)
plot(1:N,enorm_y); xlabel('Trial, k'); ylabel('Error Norm');
subplot(3,2,3)
plot(1:N,enorm_z); xlabel('Trial, k'); ylabel('Error Norm');
subplot(3,2,4)
plot(t,u_z); xlabel('Time (s)'); ylabel('u');
% subplot(3,2,2)
% semilogy(1:N,enorm_x); xlabel('Trial, k'); ylabel('Error Norm');
% subplot(3,2,3)
% plot(t,u_x); xlabel('Time (s)'); ylabel('u');
% subplot(3,2,4)
% plot(t,e_x); xlabel('Time (s)'); ylabel('e');
% subplot(3,2,5)
% plot(t,refx); xlabel('Time (s)'); ylabel('refx');
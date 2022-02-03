clear all
close all
clc
%% 

% Open model in script
%model = sim('Gantry_Model_V1.slx')
%% 

% Tracking reference for model
load trajectories_30upm_100Hz_no_offset.mat;
%% 

% Load 'y' val of tracking reference to individual var
refx = x_profile_30upm(:,2);
refy = y_profile_30upm(:,2);
refz = z_profile_30upm(:,2);
%% 

% Set time constant and sampling time
T = 2;
Ts = 0.01;

N = 100;

t = 0:Ts:T;
t(1) = []; % remove t = 0
%% 

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
%TFz = tf([kdz kpz], [mz*1680.672269 kdz*1680.672269 kpz*1680.672269]);
TFz = tf([1 592.5 5.636e4], [1 1522 6.224e5 9.473e7 0]);
%% 

% discretise system
sysdx = ss(c2d(TFx,Ts));
sysdy = ss(c2d(TFy,Ts));
sysdz = ss(c2d(TFz,Ts));
%% 

% create G matrix
gx = impulse(sysdx,T);
gx(1) = []; % remove the diagonal 
%g_x(end) = [];
Gx = toeplitz(gx,gx*0); % transfer to matrix with diagonal = CB

gy = impulse(sysdy,T);
gy(1) = []; % remove the diagonal 0
Gy = toeplitz(gy,gy*0); % transfer to matrix with diagonal = CB

gz = impulse(sysdz,T);
gz(1) = []; % remove the diagonal 0
Gz = toeplitz(gz,gz*0); % transfer to matrix with diagonal = CB
%% 

% ILC weights
Rx = eye(size(Gx))*0.01; 
Qx = eye(size(Gx));

Ry = eye(size(Gy))*0.01; 
Qy = eye(size(Gy));

Rz = eye(size(Gz))*0.01; 
Qz = eye(size(Gz));
%% 

% Vectors to store results
ux = 0*t'; % start from zero. Could start from ref, but results worse
inputx = timeseries(ux,t);
uy = 0*t'; % start from zero. Could start from ref, but results worse
inputy = timeseries(uy,t);
uz = 0*t'; % start from zero. Could start from ref, but results worse
inputz = timeseries(uz,t);

% Open model in script
%model = sim('Gantry_Model_V1.slx')
yx = lsim(TFx,ux,t)
yy = lsim(TFy,uy,t)
yz = lsim(TFz,uz,t)

%yx = model.outputx.Data();
%yx(end) = [];
ex = refx - yx;
%yy = model.outputy.Data();
%yy(end) = [];
ey = refz - yy;
%yz = model.outputz.Data();
%yz(end) = [];
ez = -refz - yz;
enorm0x = norm(ex) %initial error
enorm0y = norm(ey) %initial error
enorm0z = norm(ez) %initial error
enormx = zeros(N,1); %create N-by-1 matrix of all 0
enormy = zeros(N,1); %create N-by-1 matrix of all 0
enormz = zeros(N,1); %create N-by-1 matrix of all 0


%% 

for i=1:N
    ux  = ux + inv(Rx+Qx*(Gx'*Gx))*Qx*Gx'*ex;
    inputx = timeseries(ux,t);
    uy  = uy + inv(Ry+Qy*(Gy'*Gy))*Qy*Gy'*ey;
    inputy = timeseries(uy,t);
    uz  = uz + inv(Rz+Qz*(Gz'*Gz))*Qz*Gz'*ez;
    inputz = timeseries(uz,t);
    %u_y  = u_y + inv(R+Q*(G_y'*G_y))*Q*G_y'*e_y;
    %u_z  = u_z + inv(R+Q*(G_z'*G_z))*Q*G_z'*e_z;

   % u = u + noise;
   %model = sim('Gantry_Model_V1.slx')
   yx = lsim(TFx,ux,t)
   yy = lsim(TFy,uy,t)
   yz = lsim(TFz,uz,t)

    %yx = model.outputx.Data();
    %yx(end) = [];
    ex = refx - yx;
   % yy = model.outputy.Data();
   % yy(end) = [];
    ey = refy - yy;
   % yz = model.outputz.Data();
   % yz(end) = [];
    ez = refz - yz;
    %e_y = refy - G_y*u_y;
    %e_z = refz - G_z*u_z;
    
    enormx(i) = norm(ez);
    enormy(i) = norm(ey);
    enormz(i) = norm(ez);
    %enorm_y(i) = norm(e_y);
    %enorm_z(i) = norm(e_z);
    
end

final_enorm_x = norm(ex)
final_enorm_y = norm(ey)
final_enorm_z = norm(ez)

subplot(3,2,1)
plot(1:N,enormx); xlabel('Trial, k'); ylabel('Error Norm');
subplot(3,2,2)
plot(1:N,enormy); xlabel('Trial, k'); ylabel('Error Norm');
subplot(3,2,3)
plot(1:N,enormz); xlabel('Trial, k'); ylabel('Error Norm');
subplot(3,2,4)
plot(t,uz); xlabel('Time (s)'); ylabel('u');


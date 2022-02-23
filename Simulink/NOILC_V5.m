close all;
clear;
clc;

% load x-axis_7th_order_model.mat;
% load y_axis_model.mat;
% load z_axis_model.mat;
% load trajectories_30upm_100Hz_no_offset.mat;
load Profile_grip.mat;
load Profile_x.mat;
load Profile_y.mat;
load Profile_z.mat;

% tracking reference
refx = x_profile_V2(:,2);
refy = y_profile_V2(:,2);
refz = z_profile_V2(:,2);
refg1 = grip_profile_V2(:,2);
refg2 = grip_profile_V2(:,2);

T = 4;
Ts = 0.01;

N = 10;

t = 0:Ts:T;
t(1) = []; % remove t = 0

% object size
obj_x = 0.025;
obj_y = 0.025;
obj_z = 0.03;

% x axis trajectory values
T_x = 4;
D1_x = 0.4;
D2_x = 0.4;
d1_x = 0.8;
d2_x = 0.8;
d3_x = 0.8;
d4_x = 0.8;
A1_x = 0.5;
A2_x = 0.3;

% Y axis trajectory values
T_y = 4;
D1_y = 0.4;
D2_y = 0.4;
d1_y = 0.8;
d2_y = 0.8;
d3_y = 0.8;
d4_y = 0.8;
A1_y = 0.3;
A2_y = 0.1;

% Z axis trajectory values
T_z = 4;
D1_z = 0.2;
D2_z = 0.2;
d1_z = 0.1;
d2_z = 0.1;
d3_z = 0.1;
d4_z = 0.1;
d5_z = 0.1;
A1_z = 0.038;
A2_z = A1_z*0.6;
A3_z = 0.028;

% Enviroment
conveyer1 = 0.01;
conveyer2 = 0.02;

% Z axis clear of object
z_clear = 0;

% gripper clear to grab
grip1_clear = 0;
grip2_clear = 0;

% Mass for each axis independently
mx = 0.64*0.05*0.05*2700;
my = 0.05*0.52*0.05*2700;
mz = 0.025*0.025*0.1*2700;
mg = 0.05*0.01*0.03*2700;
%% 

% PD val for each axis to mimic actual TF
kpx = 27146.6747627399;
kdx = 1020.02839756073;
kpy = 131139.225952649;
kdy = 1480.57986826149;
kpz = 33734.3806;
kdz = 163.3421;
kpg = 33734.3806;
kdg = 163.3421;
%% 

% Define model tf
 TFx = tf([kdx kpx], [(mx+my+mz)*12.345679 kdx*12.345679 kpx*12.345679]);
 TFy = tf([kdy kpy], [(my+mz)*487.8 kdy*487.8 kpy*487.8]);
 TFz = tf([kdz kpz], [mz*1680.672269 kdz*1680.672269 kpz*1680.672269]);
 TFg = tf([kdg kpg], [mg*1680.672269 kdg*1680.672269 kpg*1680.672269]);
%% 

% discretise system
 sysdx = ss(c2d(TFx,Ts));
 sysdy = ss(c2d(TFy,Ts));
 sysdz = ss(c2d(TFz,Ts));
 sysdg1 = ss(c2d(TFg,Ts));
 sysdg2 = ss(c2d(TFg,Ts));

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

g_g1 = impulse(sysdg1,T);
g_g1(1) = []; % remove the diagonal 0
g_g1 = g_g1*Ts;
G_g1 = toeplitz(g_g1,g_g1*0); % transfer to matrix with diagonal = CB

g_g2 = impulse(sysdg2,T);
g_g2(1) = []; % remove the diagonal 0
g_g2 = g_g2*Ts;
G_g2 = toeplitz(g_g2,g_g2*0); % transfer to matrix with diagonal = CB

% ILC weights 
Rx = eye(size(G_x))*0.1; 
Qx = eye(size(G_x))*30;

Ry = eye(size(G_y))*0.001; 
Qy = eye(size(G_y))*400;

Rz = eye(size(G_z))*0.0001; 
Qz = eye(size(G_z))*1500;

Rg1 = eye(size(G_g1))*0.0001; 
Qg1 = eye(size(G_g1))*1500;

Rg2 = eye(size(G_g2))*0.0001; 
Qg2 = eye(size(G_g2))*1500;


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

u_g1 = 0*t';
enorm_g1 = zeros(N,1);

u_g2 = 0*t';
enorm_g2 = zeros(N,1);

%noise = 0.0001*sin(2*pi*0.5*t);
%noise = 0.0001;

x_pos_plt = [];
y_pos_plt = [];
z_pos_plt = [];
grip1_pos_plt = [];
grip2_pos_plt = [];

for i=1:N
    i

    % Find the position where deadtime is at
    [row_x1, col_x1] = find(t>=d1_x & t<=d1_x+D1_x);
    [row_x2, col_x2] = find(t>=d1_x+D1_x+d2_x+d3_x & t<=d1_x+D1_x+d2_x+d3_x+D2_x);

    [row_y1, col_y1] = find(t>=d1_y & t<=d1_y+D1_y);
    [row_y2, col_y2] = find(t>=d1_y+D1_y+d2_y+d3_y & t<=d1_y+D1_y+d2_y+d3_y+D2_y);

    % Find when z axis lowered to object
    [row_g11, col_g11] = find(t>=0.8+d1_z & t<=0.8+d1_z+D1_z);
    [row_g12, col_g12] = find(t>=0.8+d1_z+D1_z+d2_z+d5_z+d3_z+1.5 & t<=0.8+d1_z+D1_z+d2_z+d5_z+d3_z+1.5+D2_z);
    [row_g21, col_g21] = find(t>=0.8+d1_z & t<=0.8+d1_z+D1_z);
    [row_g22, col_g22] = find(t>=0.8+d1_z+D1_z+d2_z+d5_z+d3_z+1.5 & t<=0.8+d1_z+D1_z+d2_z+d5_z+d3_z+1.5+D2_z);

    if(0) %run uing lsim
        yx = lsim(TFx,u_x,t);
        yy = lsim(TFy,u_y,t);
        if (z_clear)
            yz = lsim(TFz,u_z,t);
            if (grip1_clear)
                yg1 = lsim(TFg,u_g1,t);
            end
            if (grip2_clear)
                yg2 = lsim(TFg,u_g2,t);
            end
        end
    elseif(0)
        yx = G_x*u_x;
        yy = G_y*u_y;
        if (z_clear)
            yz = G_z*u_z;
            if (grip1_clear)
                yg1 = G_g1*u_g1;
            else 
                yg1 = G_g1*0;
            end
            if (grip2_clear)
                yg2 = G_g2*u_g1;
            else 
                yg2 = G_g2*0;
            end
        else
            yz = G_z*0;
            yg1 = G_g1*0;
            yg2 = G_g2*0;
        end
        
    else
       % model = sim()
        inputx = timeseries(u_x,t);
        inputy = timeseries(u_y,t);
%         if (z_clear)
%             inputz = timeseries(u_z,t);
%             if (grip1_clear)
%                 inputg1 = timeseries(u_g1,t);
%             else
%                 inputg1 = timeseries(u_g1,t);
%             end
%             if (grip2_clear)
%                 inputg2 = timeseries(u_g2,t);
%             else
%                 inputg2 = timeseries(u_g2,t);
%             end
%         else 
        inputz = timeseries(u_z,t);
        inputg1 = timeseries(u_g1,t);
        inputg2 = timeseries(u_g2,t);
%         end
        

        model = sim('Gantry_Model_V9.slx')

        yx = model.outputx.Data();
        yx(1) = [];
        yy = model.outputy.Data();
        yy(1) = [];
        yz = model.outputz.Data();
        yz(1) = [];
        yg1 = model.outputg1.Data();
        yg1(1) = [];
        yg2 = model.outputg2.Data();
        yg2(1) = [];

        gripperforce = model.Gripper_Force.Data();
        gripperforce(1) = [];

        xinputforce = model.x_input_force.Data();
        xinputforce(1) = [];
        yinputforce = model.y_input_force.Data();
        yinputforce(1) = [];
        xsenseforce = model.x_sense_force.Data();
        xsenseforce(1) = [];
        ysenseforce = model.y_sense_force.Data();
        ysenseforce(1) = [];
       
    end

    e_x = refx - yx;
    e_y = refy - yy;
    e_z = -refz - yz;
    e_g1 = refg1 - yg1;
    e_g2 = refg2 - yg2;

    enorm_x(i) = norm(e_x);
    enorm_y(i) = norm(e_y);
    enorm_z(i) = norm(e_z);
    enorm_g1(i) = norm(e_g1);
    enorm_g2(i) = norm(e_g2);


    % Decide if z should move
    if (mean(yx(col_x1))>=A1_x-obj_x && mean(yx(col_x2))>=A2_x-obj_x)
        if (mean(yx(col_y1))>=A1_y-obj_y && mean(yx(col_y2))>=A2_y-obj_y)
            z_clear = 1;
        end
    end

    % Decide if gripper should grip
    if (abs(mean(yz(col_g11)))>=0.055 - conveyer1 - obj_z + 0.005 && abs(mean(yz(col_g12)))>=0.055 - conveyer2 - obj_z + 0.005)
        grip1_clear = 1;
    end
    if (abs(mean(yz(col_g21)))>=0.055 - conveyer1 - obj_z + 0.005 && abs(mean(yz(col_g22)))>=0.055 - conveyer2 - obj_z + 0.005)
        grip2_clear = 1;
    end

    
    u_x  = u_x + inv(Rx+Qx*(G_x'*G_x))*Qx*G_x'*e_x;
    u_y  = u_y + inv(Ry+Qy*(G_y'*G_y))*Qy*G_y'*e_y;
    %u_y  = u_y + inv(Ry+G_y'*Qy*G_y)*G_y'* Qy*e_y;
    if (z_clear)
        u_z  = u_z + inv(Rz+Qz*(G_z'*G_z))*Qz*G_z'*e_z;
        if (1)
            u_g1  = u_g1 + inv(Rg1+Qg1*(G_g1'*G_g1))*Qg1*G_g1'*e_g1;
        else
            u_g1 = u_g2 + 0*t';
        end
        if (1)
            u_g2  = u_g2 + inv(Rg2+Qg2*(G_g2'*G_g2))*Qg2*G_g2'*e_g2;
        else 
            u_g2 = u_g2 + 0*t';
        end
    else
        u_z = 0*t';
        u_g1 = 0*t';
        u_g2 = 0*t';
    end



   % u = u + noise;
    

%     e_x = refx - G_x*u_x;
%     e_y = refy - G_y*u_y;
%     e_z = refz - G_z*u_z;

    x_pos_plt = [x_pos_plt yx];
    y_pos_plt = [y_pos_plt yy];
    z_pos_plt = [z_pos_plt yz];
    grip1_pos_plt = [grip1_pos_plt yg1];
    grip2_pos_plt = [grip2_pos_plt yg2];
    

end

final_enorm_x = norm(e_x)
enorm_x(1)
% accuracy_x = enorm_x(25)/enorm_x(1)*100
final_enorm_y = norm(e_y)
enorm_y(1)
% accuracy_y = enorm_y(25)/enorm_y(1)*100
final_enorm_z = norm(e_z)
enorm_z(1)
% accuracy_z = enorm_z(25)/enorm_z(1)*100
final_enorm_g1 = norm(e_g1)
enorm_g1(1)
%accuracy_g1 = enorm_g1(25)/enorm_g1(1)*100
final_enorm_g2 = norm(e_g2)
enorm_g2(1)
%accuracy_g2 = enorm_g2(25)/enorm_g2(1)*100

subplot(3,3,1)
plot(1:N,enorm_g1); xlabel('Trial, k'); ylabel('Error Norm'); title('gripper 1');
subplot(3,3,2)
plot(1:N,enorm_g2); xlabel('Trial, k'); ylabel('Error Norm'); title('gripper 2');
subplot(3,3,6)
plot(1:N,enorm_z); xlabel('Trial, k'); ylabel('Error Norm'); title('z axis');
subplot(3,3,5)
plot(1:N,enorm_y); xlabel('Time (s)'); ylabel('Error Norm'); title('y axis');
subplot(3,3,4)
plot(1:N,enorm_x); xlabel('Trial, k'); ylabel('Error Norm');title('x axis');
% subplot(3,2,3)
% plot(t,u_x); xlabel('Time (s)'); ylabel('u');
% subplot(3,2,4)
% plot(t,e_x); xlabel('Time (s)'); ylabel('e');
% subplot(3,2,5)
% plot(t,refx); xlabel('Time (s)'); ylabel('refx');
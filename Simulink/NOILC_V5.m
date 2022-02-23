close all;
clear;
clc;

points = 400;
T = 5;
Ts = 0.0125;
time_x = [0.8 1.4 0.8 1.2 0.8];
A_x = [0.5 0.3];
time_y = [0.8 1.4 0.8 1.2 0.8];
A_y = [0.3 0.1];
time_z1 = [0.8 0.3 0.6 0.3 0.2 0.8 0.3 0.6 0.3 0.8];
A_z1 = [0.038 0.028];
time_z2 = [0.8 0.3 0.6 0.3 0.2 0.8 0.3 0.6 0.3 0.8];
A_z2 = [0.038 0.028];
time_g1 = [0.8 0.4 0.2 0.3 2.1 0.4 0.8];
A_g1 = [0.0115+0.003 0.0115-0.0002];
time_g2 = [0.8 0.4 2.6 0.4 0.8];
A_g2 =[0.0115+0.003];

% tracking reference
refx = X_axis_V3(time_x, T, points, A_x);
refx = refx(:,2);
refy = Y_axis_V3(time_y, T, points, A_y);
refy = refy(:,2);
refz1 = Z1_axis_V3(time_z1, T, points, A_z1);
refz1 = refz1(:,2);
refz2 = Z2_axis_V3(time_z2, T, points, A_z2);
refz2 = refz2(:,2);
refLg1 = G1_axis_V3(time_g1, T, points, A_g1);
refLg1 = refLg1(:,2);
refLg2 = G2_axis_V3(time_g2, T, points, A_g2);
refLg2 = refLg2(:,2);
refRg1 = G1_axis_V3(time_g1, T, points, A_g1);
refRg1 = refRg1(:,2);
refRg2 = G2_axis_V3(time_g2, T, points, A_g2);
refRg2 = refRg2(:,2);

N = 10;

t = 0:Ts:T;
t(1) = []; % remove t = 0

% object size
obj_x = 0.025;
obj_y = 0.025;
obj_z = 0.03;
obj_density = 100000;
obj_contactpt = 8;
obj_coeff = 0.5;

Fmin = obj_x*obj_y*obj_z*obj_density*9.81/obj_coeff/obj_contactpt

% Enviroment
conveyer1 = 0.01;
conveyer2 = 0.02;

% Z axis clear of object
z_clear = 0;

% gripper clear to grab
gripL_clear = 0;
gripR_clear = 0;

firm_grip = 0;

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
 sysdgL = ss(c2d(TFg,Ts));
 sysdgR = ss(c2d(TFg,Ts));

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

g_gL = impulse(sysdgL,T);
g_gL(1) = []; % remove the diagonal 0
g_gL = g_gL*Ts;
G_gL = toeplitz(g_gL,g_gL*0); % transfer to matrix with diagonal = CB

g_gR = impulse(sysdgR,T);
g_gR(1) = []; % remove the diagonal 0
g_gR = g_gR*Ts;
G_gR = toeplitz(g_gR,g_gR*0); % transfer to matrix with diagonal = CB

% ILC weights 
Rx = eye(size(G_x))*0.1; 
Qx = eye(size(G_x))*30;

Ry = eye(size(G_y))*0.001; 
Qy = eye(size(G_y))*400;

Rz = eye(size(G_z))*0.0001; 
Qz = eye(size(G_z))*1500;

RgL = eye(size(G_gL))*0.0001; 
QgL = eye(size(G_gL))*1500;

RgR = eye(size(G_gR))*0.0001; 
QgR = eye(size(G_gR))*1500;


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

u_gL = 0*t';
enorm_gL = zeros(N,1);

u_gR = 0*t';
enorm_gR = zeros(N,1);

%noise = 0.0001*sin(2*pi*0.5*t);
%noise = 0.0001;

x_pos_plt = [];
y_pos_plt = [];
z_pos_plt = [];
gripL_pos_plt = [];
gripR_pos_plt = [];
gripL_FNormal_plt = [];
gripR_FNormal_plt = [];

for i=1:N
    i

    % Find the position where deadtime is at
    [row_x1, col_x1] = find(t>=time_x(1) & t<=time_x(1)+time_x(2));
    [row_x2, col_x2] = find(t>=time_x(1)+time_x(2)+time_x(3) & t<=time_x(1)+time_x(2)+time_x(3)+time_x(4));

    [row_y1, col_y1] = find(t>=time_y(1) & t<=time_y(1)+time_y(2));
    [row_y2, col_y2] = find(t>=time_y(1)+time_y(2)+time_y(3) & t<=time_y(1)+time_y(2)+time_y(3)+time_y(4));

    % Find when z axis lowered to object
    [row_gL1, col_gL1] = find(t>=time_z2(1)+time_z2(2) & t<=time_z2(1)+time_z2(2)+time_z2(3));
    [row_gL2, col_gL2] = find(t>=time_z2(1)+time_z2(2)+time_z2(3)+time_z2(4)+time_z2(5)+time_z2(6)+time_z2(7) & t<=time_z2(1)+time_z2(2)+time_z2(3)+time_z2(4)+time_z2(5)+time_z2(6)+time_z2(7)+time_z2(8));
    [row_gR1, col_gR1] = find(t>=time_z2(1)+time_z2(2) & t<=time_z2(1)+time_z2(2)+time_z2(3));
    [row_gR2, col_gR2] = find(t>=time_z2(1)+time_z2(2)+time_z2(3)+time_z2(4)+time_z2(5)+time_z2(6)+time_z2(7) & t<=time_z2(1)+time_z2(2)+time_z2(3)+time_z2(4)+time_z2(5)+time_z2(6)+time_z2(7)+time_z2(8));

    if(0) %run uing lsim
        yx = lsim(TFx,u_x,t);
        yy = lsim(TFy,u_y,t);
        if (z_clear)
            yz = lsim(TFz,u_z,t);
            if (gripL_clear)
                ygL = lsim(TFg,u_gL,t);
            end
            if (gripR_clear)
                ygR = lsim(TFg,u_gR,t);
            end
        end
    elseif(0)
        yx = G_x*u_x;
        yy = G_y*u_y;
        if (z_clear)
            yz = G_z*u_z;
            if (gripL_clear)
                ygL = G_gL*u_gL;
            else 
                ygL = G_gL*0;
            end
            if (gripR_clear)
                ygR = G_gR*u_gR;
            else 
                ygR = G_gR*0;
            end
        else
            yz = G_z*0;
            ygL = G_gL*0;
            ygR = G_gR*0;
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
        inputgL = timeseries(u_gL,t);
        inputgR = timeseries(u_gR,t);
%         end
        

        model = sim('Gantry_Model_V10.slx')

        yx = model.outputx.Data();
        yx(1) = [];
        yy = model.outputy.Data();
        yy(1) = [];
        yz = model.outputz.Data();
        yz(1) = [];
        ygL = model.outputgL.Data();
        ygL(1) = [];
        ygR = model.outputgR.Data();
        ygR(1) = [];

        LNormalF = model.LNormalForce.Data();
        LNormalF(1) = [];
        RNormalF = model.RNormalForce.Data();
        RNormalF(1) = [];

       
    end

    e_x = refx - yx;
    e_y = refy - yy;

    [row_g, col_g] = find(t>=time_g1(1)+time_g1(2) & t<=time_g1(1)+time_g1(2)+time_g1(3));
    if (mean(LNormalF(col_g)) >= Fmin * 1.05 && mean(RNormalF(col_g)) >= Fmin * 1.05)
        firm_grip = 1;
    else
        firm_grip = 0;
    end
    

    if (firm_grip)
        e_z = -refz2 - yz;
        e_gL = refLg2 - ygL;
        e_gR = refRg2 - ygR;
    else 
        e_z = -refz1 - yz;
        e_gL = refLg1 - ygL;
        e_gR = refRg1 - ygR;
    end

    enorm_x(i) = norm(e_x);
    enorm_y(i) = norm(e_y);
    enorm_z(i) = norm(e_z);
    enorm_gL(i) = norm(e_gL);
    enorm_gR(i) = norm(e_gR);


    % Decide if z should move
    if (mean(yx(col_x1))>=A_x(1)-obj_x && mean(yx(col_x2))>=A_x(2)-obj_x)
        if (mean(yx(col_y1))>=A_y(2)-obj_y && mean(yx(col_y2))>=A_y(2)-obj_y)
            z_clear = 1;
        end
    end

    % Decide if gripper should grip
    if (abs(mean(yz(col_gL1)))>=0.055 - conveyer1 - obj_z + 0.005 && abs(mean(yz(col_gL2)))>=0.055 - conveyer2 - obj_z + 0.005)
        gripL_clear = 1;
    end
    if (abs(mean(yz(col_gR1)))>=0.055 - conveyer1 - obj_z + 0.005 && abs(mean(yz(col_gR2)))>=0.055 - conveyer2 - obj_z + 0.005)
        gripR_clear = 1;
    end

    
    u_x  = u_x + inv(Rx+Qx*(G_x'*G_x))*Qx*G_x'*e_x;
    u_y  = u_y + inv(Ry+Qy*(G_y'*G_y))*Qy*G_y'*e_y;
    %u_y  = u_y + inv(Ry+G_y'*Qy*G_y)*G_y'* Qy*e_y;
    if (z_clear)
        u_z  = u_z + inv(Rz+Qz*(G_z'*G_z))*Qz*G_z'*e_z;
        if (1)
            u_gL  = u_gL + inv(RgL+QgL*(G_gL'*G_gL))*QgL*G_gL'*e_gL;
        else
            u_gL = u_gL + 0*t';
        end
        if (1)
            u_gR  = u_gR + inv(RgR+QgR*(G_gR'*G_gR))*QgR*G_gR'*e_gR;
        else 
            u_gR = u_gR + 0*t';
        end
    else
        u_z = 0*t';
        u_gL = 0*t';
        u_gR = 0*t';
    end



   % u = u + noise;
    

%     e_x = refx - G_x*u_x;
%     e_y = refy - G_y*u_y;
%     e_z = refz - G_z*u_z;

    x_pos_plt = [x_pos_plt yx];
    y_pos_plt = [y_pos_plt yy];
    z_pos_plt = [z_pos_plt yz];
    gripL_pos_plt = [gripL_pos_plt ygL];
    gripR_pos_plt = [gripR_pos_plt ygR];
    gripL_FNormal_plt = [gripL_FNormal_plt LNormalF];
    gripR_FNormal_plt = [gripR_FNormal_plt RNormalF];
    

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
final_enorm_gL = norm(e_gL)
enorm_gL(1)
%accuracy_g1 = enorm_g1(25)/enorm_g1(1)*100
final_enorm_gR = norm(e_gR)
enorm_gR(1)
%accuracy_g2 = enorm_g2(25)/enorm_g2(1)*100

subplot(3,3,1)
plot(1:N,enorm_gL); xlabel('Trial, k'); ylabel('Error Norm'); title('gripper 1');
subplot(3,3,2)
plot(1:N,enorm_gR); xlabel('Trial, k'); ylabel('Error Norm'); title('gripper 2');
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
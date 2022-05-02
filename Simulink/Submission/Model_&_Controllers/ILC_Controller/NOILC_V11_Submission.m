
close all;
clear;
clc;

% General simulation parameters
points = 400; % Number of samples
T = 5; % Total trajectory duration
Ts = 0.0125; % Sampling period
N = 40; % number of iterations
t = 0:Ts:T; % Discretised time
t(1) = []; % remove t = 0


%% TRAJECTORY PROFILE GENERATION PARAMETERS

% Amplitude
A_x = [0.5 0.3];
A_y = [0.3 0.1];
A_z1 = [0.037 0.027];   
A_z2 = [0.037 0.027];
A_g1 = [0.0115+0.003 0.0115-0.0002];
A_g2 =[0.0115+0.003 0.0115+0.003];

% Duration for each part of the profile
time_x = [0.8 1.4 0.8 1.2 0.8];
time_y = [0.8 1.4 0.8 1.2 0.8];
time_z1 = [0.8 0.3 0.6 0.3 0.2 0.8 0.3 0.6 0.3 0.8];
time_z2 = [0.8 0.3 0.6 0.3 0.2 0.8 0.3 0.6 0.3 0.8];
time_g1 = [0.8 0.4 0.2 0.3 2.1 0.4 0.8];
time_g2 = [0.8 0.4 2.6 0.4 0.8];

% Profile generation
refx = X_axis_V3(time_x, T, points, A_x, Ts);
refx = refx(:,2);
refy = Y_axis_V3(time_y, T, points, A_y, Ts);
refy = refy(:,2);
refz1 = Z1_axis_V3(time_z1, T, points, A_z1, Ts);
refz1 = refz1(:,2);
refz2 = Z2_axis_V3(time_z2, T, points, A_z2, Ts);
refz2 = refz2(:,2);
refLg1 = G1_axis_V3(time_g1, T, points, A_g1, Ts);
refLg1 = refLg1(:,2);
refLg2 = G2_axis_V3(time_g2, T, points, A_g2, Ts);
refLg2 = refLg2(:,2);
refRg1 = G1_axis_V3(time_g1, T, points, A_g1, Ts);
refRg1 = refRg1(:,2);
refRg2 = G2_axis_V3(time_g2, T, points, A_g2, Ts);
refRg2 = refRg2(:,2);


%% DISTURBANCE SIGNAL GENERATION

% Iteration of when disturbance occurs
error_start = 0;

% Amplitude
A_disturbance_x = [0.625/2];
A_disturbance_y = [15];
A_disturbance_z = [-5.74];
A_disturbance_g = [0.384];

% Duration
time_disturbance_x = [2.3 0.2 0.2 0.2 2.1];
time_disturbance_y = [2.3 0.2 0.2 0.2 2.1];
time_disturbance_z = [2.2 0.2 0.2 0.2 2.2];
time_disturbance_g = [2.3 0.1 0.2 0.1 2.3];

% Disturbance generation
% disturbance_x = A_disturbance_x;
% disturbance_x = impulse_disturbance(time_disturbance_x,T,points,A_disturbance_x,Ts);
% disturbance_x = disturbance_x(:,2);

% disturbance_y = A_disturbance_y;
% disturbance_y = impulse_disturbance(time_disturbance_y,T,points,A_disturbance_y,Ts);
% disturbance_y = disturbance_y(:,2);

% disturbance_z = A_disturbance_z;
% disturbance_z = impulse_disturbance(time_disturbance_z,T,points,A_disturbance_z,Ts);
% disturbance_z = disturbance_z(:,2);

% disturbance_g = A_disturbance_g;
% disturbance_g = impulse_disturbance(time_disturbance_g,T,points,A_disturbance_g,Ts);
% disturbance_g = disturbance_g(:,2);

%% Details of object to be grasped

% Dimension
obj_x = 0.025;
obj_y = 0.025;
obj_z = 0.03;

% Density
obj_density = 106666.6667;

% Mass
mo = obj_x*obj_y*obj_z*obj_density;

% Number of grasping points
obj_contactpt = 8;

% Friction coefficient
obj_coeff = 0.5;

% Minimum force to grasp the object
Fmin = obj_x*obj_y*obj_z*obj_density*9.81/obj_coeff/obj_contactpt

%% Simulation environment details

% Height of conveyer
conveyer1 = 0.01;
conveyer2 = 0.02;

% stiffness of contact point modelling
stiffness = 1e4;

%% Logic flags

% X and Y axis are within grasping area
z_clear = 0;

% X, Y and Z axis are within grasping volume
gripL_clear = 0;
gripR_clear = 0;

% There is sufficient force to grasp the object securely
firm_grip = 0;

% Grip condition for previous iteration
prev_firm_grip = 0;

% Did the gripper drop the object?
objDrop = 0;

%% Gantry model in Simulink

% Mass for each part of the gantry 
mx = 0.64*0.05*0.05*2700;
my = 0.05*0.52*0.05*2700;
mz = 0.025*0.025*0.1*2700;
mg = 0.05*0.01*0.03*2700;

% PD val for each axis to mimic actual TF
kpx = 27146.6747627399;
kdx = 1020.02839756073;
kpy = 131139.225952649;
kdy = 1480.57986826149;
kpz = 33734.3806;
kdz = 163.3421;
kpg = 33734.3806;
kdg = 163.3421;

% Simulink model TF
TFx = tf([kdx kpx], [(mx+my+mz+2*mg+mo)*12.345679 kdx*12.345679 kpx*12.345679]);
TFy = tf([kdy kpy], [(my+mz+2*mg+mo)*487.8 kdy*487.8 kpy*487.8]);
TFz = tf([kdz kpz], [(mz+2*mg+mo)*1680.672269 kdz*1680.672269 kpz*1680.672269]);
TFg = tf([kdg kpg], [(mg+mo)*1625.672269 kdg*1625.672269 kpg*1625.672269]);

% Discretise TF
sysdx = ss(c2d(TFx,Ts));
sysdy = ss(c2d(TFy,Ts));
sysdz = ss(c2d(TFz,Ts));
sysdgL = ss(c2d(TFg,Ts));
sysdgR = ss(c2d(TFg,Ts));

%% Values for Norm-Opmital ILC controller

% create G matrix
g_x = impulse(sysdx,T);
g_x(1) = []; % remove the diagonal 
g_x = g_x*Ts;
G_x = toeplitz(g_x,g_x*0); % transfer to matrix with diagonal = CB

g_y = impulse(sysdy,T);
g_y(1) = []; % remove the diagonal
g_y = g_y*Ts;
G_y = toeplitz(g_y,g_y*0); % transfer to matrix with diagonal = CB

g_z = impulse(sysdz,T);
g_z(1) = []; % remove the diagonal
g_z = g_z*Ts;
G_z = toeplitz(g_z,g_z*0); % transfer to matrix with diagonal = CB

g_gL = impulse(sysdgL,T);
g_gL(1) = []; % remove the diagonal
g_gL = g_gL*Ts;
G_gL = toeplitz(g_gL,g_gL*0); % transfer to matrix with diagonal = CB

g_gR = impulse(sysdgR,T);
g_gR(1) = []; % remove the diagonal
g_gR = g_gR*Ts;
G_gR = toeplitz(g_gR,g_gR*0); % transfer to matrix with diagonal = CB

% ILC weights 
Rx = eye(size(G_x))*0.1; 
Qx = eye(size(G_x))*30;

Ry = eye(size(G_y))*0.001; 
Qy = eye(size(G_y))*400;

Rz = eye(size(G_z))*0.0001; 
Qz = eye(size(G_z))*1500;

RgL = eye(size(G_gL))*0.00005; 
QgL = eye(size(G_gL))*2000;

RgR = eye(size(G_gR))*0.00005; 
QgR = eye(size(G_gR))*2000;

%% ILC Input vector initial generation

% Input vector
u_x = 0*t'; 
% Normalised position error
enorm_x = zeros(N,1);

u_y = 0*t';
enorm_y = zeros(N,1);

u_z = 0*t';
enorm_z = zeros(N,1);

u_gL = 0*t';
enorm_gL = zeros(N,1);
% Normalised force error
enorm_FgL = zeros(N,1);

u_gR = 0*t';
enorm_gR = zeros(N,1);
enorm_FgR = zeros(N,1);

%% Parameters for storing values of iterations

x_pos_plt = []; % Position feedback
x_input_plt = []; % Input vector

y_pos_plt = [];
y_input_plt = [];

z_pos_plt = [];
z_input_plt = [];

gripL_pos_plt = [];
gL_input_plt = [];
gripL_FNormal_plt = []; % Normal force
tracjecotryGL = []; % Adjusted reference profile to reduce force

gripR_pos_plt = [];
gR_input_plt = [];
gripR_FNormal_plt = [];
% gripR_FNormal_smooth_plt = [];
tracjecotryGR = [];


%% Simulation loop
for i=1:N

    % Display values for visual information
    i
    firm_grip

    % Store input vector used for current iteration
    x_input_plt = [x_input_plt u_x];
    y_input_plt = [y_input_plt u_y];
    z_input_plt = [z_input_plt u_z];
    gL_input_plt = [gL_input_plt u_gL];
    gR_input_plt = [gR_input_plt u_gR];
    

    % Samples where X and Y axis deatime are located
    [row_x1, col_x1] = find(t>=time_x(1) & t<=time_x(1)+time_x(2));
    [row_x2, col_x2] = find(t>=time_x(1)+time_x(2)+time_x(3) & t<=time_x(1)+time_x(2)+time_x(3)+time_x(4));

    [row_y1, col_y1] = find(t>=time_y(1) & t<=time_y(1)+time_y(2));
    [row_y2, col_y2] = find(t>=time_y(1)+time_y(2)+time_y(3) & t<=time_y(1)+time_y(2)+time_y(3)+time_y(4));

    % Samples where Z axis deatime are located
    [row_gL1, col_gL1] = find(t>=time_z2(1)+time_z2(2) & t<=time_z2(1)+time_z2(2)+time_z2(3));
    [row_gL2, col_gL2] = find(t>=time_z2(1)+time_z2(2)+time_z2(3)+time_z2(4)+time_z2(5)+time_z2(6)+time_z2(7) & t<=time_z2(1)+time_z2(2)+time_z2(3)+time_z2(4)+time_z2(5)+time_z2(6)+time_z2(7)+time_z2(8));
    [row_gR1, col_gR1] = find(t>=time_z2(1)+time_z2(2) & t<=time_z2(1)+time_z2(2)+time_z2(3));
    [row_gR2, col_gR2] = find(t>=time_z2(1)+time_z2(2)+time_z2(3)+time_z2(4)+time_z2(5)+time_z2(6)+time_z2(7) & t<=time_z2(1)+time_z2(2)+time_z2(3)+time_z2(4)+time_z2(5)+time_z2(6)+time_z2(7)+time_z2(8));

    % Samples where Gripper deatime are located
    [row_gf, col_gf] = find(t>=time_g2(1)+time_g2(2) & t<=time_g2(1)+time_g2(2)+time_g2(3));

    % Change to '1' to run simulation via lsim
    if(0) %run uing lsim
        yx = lsim(TFx,u_x,t);
        yy = lsim(TFy,u_y,t);
        if (z_clear)
            yz = lsim(TFz,u_z,t);
            ygL = lsim(TFg,u_gL,t);
            ygR = lsim(TFg,u_gR,t);
        end
    else

        % Uncomment for disturbance in X axis
    %     if (i >= error_start)
    %         u_x_d = u_x+disturbance_x;
    %     else
    %         u_x_d = u_x;
    %     end
    %     inputx = timeseries(u_x_d,t);
    
        % Comment for disturbance in X axis
        inputx = timeseries(u_x,t);
    
        % Uncomment for disturbance in Y axis
    %     if (i >= error_start)
    %         u_y_d = u_y+disturbance_y;
    %     else
    %         u_y_d = u_y;
    %     end
    %     inputy = timeseries(u_y_d,t);
    
        % Comment for disturbance in Y axis
        inputy = timeseries(u_y,t);
    
    
        % Uncomment for disturbance in Z axis
    %     if (i >= error_start)
    %         u_z_d = u_z+disturbance_z;
    %     else
    %         u_z_d = u_z;
    %     end
    %     inputz = timeseries(u_z_d,t);
    
        % Comment for disturbance in Z axis
        inputz = timeseries(u_z,t);
    
            
        % Uncomment for disturbance in Gripper
    %     if (i >= error_start)
    %         u_gL_d = u_gL + disturbance_g;
    %         u_gR_d = u_gR + disturbance_g;
    %     else
    %         u_gL_d = u_gL;
    %         u_gR_d = u_gR;
    %     end
    %     inputgL = timeseries(u_gL_d,t);
    %     inputgR = timeseries(u_gR_d,t);
    
        % Comment for disturbance in Gripper
        inputgL = timeseries(u_gL,t);
        inputgR = timeseries(u_gR,t);
    
        tIn = timeseries(t,t);
        model = sim('Gantry_Model_V14_Submission.slx')
    
        % Output data from simulation
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

    if (firm_grip)

        %%% LEFT SIDE

        % Get the force when it is gripped
        LForceMeasured = LNormalF(col_gf(1:end));

        % Find the diff between actual grip force and ideal with margin
        LFdiff = Fmin*1.2 - LForceMeasured;

        % Generate 0 values for period where gripper trajectory is 0
        Fzeros1 = zeros(int16(points/T*(time_g2(1)+time_g2(2)))-1);
        Fzeros1 = Fzeros1(:,1);
        Fzeros2 = zeros(int16(points/T*(time_g2(4)+time_g2(5))));
        Fzeros2 = Fzeros2(:,1);

        % Net change of force to whole trajectory
        FLChange = [Fzeros1; LFdiff; Fzeros2];

        % Net profile change to whole trajectory
        distLChange = FLChange/stiffness;
        
        % New profile ILC update
        refLg2 = ygL + 0.5*distLChange;

        % Regenerate the gripping profile to ensure smooth profile
        temp = G2_axis_V3(time_g2, T, points, [refLg2(col_gf(2)) refLg2(col_gf(end))], Ts);
        temp = temp(:,2);
        refLg2 = [temp(1:col_gf-1);smoothdata(refLg2(col_gf(1:end)));temp(col_gf(end)+1:end)];


        %%% Right SIDE

        % Get the force when it is gripped
        RForceMeasured = RNormalF(col_gf(1:end));

        % Find the diff between actual grip force and ideal with margin
        RFdiff = Fmin*1.2 - RForceMeasured;

        % Generate 0 values for period where gripper trajectory is 0
        Fzeros1 = zeros(int16(points/T*(time_g2(1)+time_g2(2)))-1);
        Fzeros1 = Fzeros1(:,1);
        Fzeros2 = zeros(int16(points/T*(time_g2(4)+time_g2(5))));
        Fzeros2 = Fzeros2(:,1);

        % Net change of force to whole trajectory
        FRChange = [Fzeros1; RFdiff; Fzeros2];

        % Net profile change to whole trajectory
        distRChange = FRChange/stiffness;

        % New profile ILC update
        refRg2 = ygR + 0.5*distRChange;
    
        % Regenerate the gripping profile to ensure smooth profile
        temp = G2_axis_V3(time_g2, T, points, [refRg2(col_gf(2)) refRg2(col_gf(end))], Ts);
        temp = temp(:,2);
        refRg2 = [temp(1:col_gf-1);smoothdata(refRg2(col_gf(1:end)));temp(col_gf(end)+1:end)];
            
        % Normalised force error
        enorm_FgL(i) = norm(LFdiff);
        enorm_FgR(i) = norm(RFdiff);
    end

    % Decide if X and Y axis are within grasping area
    if (mean(yx(col_x1))>=A_x(1)-A_x(1)*0.03 && mean(yx(col_x2))>=A_x(2)-A_x(2)*0.03)
        if (mean(yx(col_y1))>=A_y(2)-A_y(2)*0.03 && mean(yx(col_y2))>=A_y(2)-A_y(2)*0.03)
            z_clear = 1;
        end
    end

    % Decide if X, Y and Z axis are within grasping volume
    if (abs(mean(yz(col_gL1)))>=0.055 - conveyer1 - obj_z + 0.005 && abs(mean(yz(col_gL2)))>=0.055 - conveyer2 - obj_z + 0.005)
        gripL_clear = 1;
    end
    if (abs(mean(yz(col_gR1)))>=0.055 - conveyer1 - obj_z + 0.005 && abs(mean(yz(col_gR2)))>=0.055 - conveyer2 - obj_z + 0.005)
        gripR_clear = 1;
    end
    
    prev_firm_grip = firm_grip

    % decide which trajectory to use 
    [row_g, col_g] = find(t>=time_g1(1)+time_g1(2) & t<=time_g1(1)+time_g1(2)+time_g1(3));
    % Decide if 'test' grasp has reach minimum force
    if (mean(LNormalF(col_g)) >= Fmin * 1.05 && mean(RNormalF(col_g)) >= Fmin * 1.05)
        if (mean(LNormalF(col_g)) >= Fmin * 1.2 && mean(RNormalF(col_g)) >= Fmin * 1.2) 
            % Regenerate grasping profile 1 with minimum force grasping distance
            refLg1 = G1_axis_V3(time_g1, T, points, [refLg2(col_g(8)) 0.0115-0.0002], Ts);
            refLg1 = refLg1(:,2);
            refRg1 = G1_axis_V3(time_g1, T, points, [refRg2(col_g(8)) 0.0115-0.0002], Ts);
            refRg1 = refRg1(:,2);
        end
        firm_grip = 1;
        objDrop = 1;
    elseif (mean(LNormalF(col_g)) >= Fmin && mean(RNormalF(col_g)) >= Fmin && firm_grip ==1)
        % Still able to grasp but on the edge of dropping
        firm_grip = 1;
        objDrop = 1;
    elseif ((prev_firm_grip == 1 && firm_grip == 0) || objDrop == 1)
        % Object dropped
        objDrop = 1;
    else
        firm_grip = 0;
        objDrop = 0;
    end


    % Calculate tracking error
    e_x = refx - yx;
    e_y = refy - yy;
   
    if (firm_grip)
        e_z = -refz1 - yz;
        e_gL = refLg2 - ygL;
        e_gR = refRg2 - ygR;
    else 
        e_z = -refz1 - yz;
        e_gL = refLg1 - ygL;
        e_gR = refRg1 - ygR;
    end

    % Normalised each error
    enorm_x(i) = norm(e_x);
    enorm_y(i) = norm(e_y);
    enorm_z(i) = norm(e_z);
    enorm_gL(i) = norm(e_gL);
    enorm_gR(i) = norm(e_gR);

    % Norm-Optimal ILC update input vector
    u_x  = u_x + inv(Rx+Qx*(G_x'*G_x))*Qx*G_x'*e_x;
    u_y  = u_y + inv(Ry+Qy*(G_y'*G_y))*Qy*G_y'*e_y;
    if (z_clear)
        u_z  = u_z + inv(Rz+Qz*(G_z'*G_z))*Qz*G_z'*e_z;
        if (gripL_clear)
            u_gL  = u_gL + inv(RgL+QgL*(G_gL'*G_gL))*QgL*G_gL'*e_gL;
        else
            u_gL = u_gL + 0*t';
        end
        if (gripR_clear)
            u_gR  = u_gR + inv(RgR+QgR*(G_gR'*G_gR))*QgR*G_gR'*e_gR;
        else 
            u_gR = u_gR + 0*t';
        end
    else
        u_z = 0*t';
        u_gL = 0*t';
        u_gR = 0*t';
    end

    % Store feedback generated from current iteration
    x_pos_plt = [x_pos_plt yx];
    y_pos_plt = [y_pos_plt yy];
    z_pos_plt = [z_pos_plt yz];
    gripL_pos_plt = [gripL_pos_plt ygL];
    gripR_pos_plt = [gripR_pos_plt ygR];
    gripL_FNormal_plt = [gripL_FNormal_plt LNormalF];
    gripR_FNormal_plt = [gripR_FNormal_plt RNormalF];
    tracjecotryGL = [tracjecotryGL refLg2];
    tracjecotryGR = [tracjecotryGR refRg2];


end

%% Post simulation plots and display

% Show initial and final normalised error value
final_enorm_x = norm(e_x)
enorm_x(1)
final_enorm_y = norm(e_y)
enorm_y(1)
final_enorm_z = norm(e_z)
enorm_z(1)
final_enorm_gL = norm(e_gL)
enorm_gL(1)
final_enorm_gR = norm(e_gR)
enorm_gR(1)
final_enorm_gFL = norm(LFdiff)
enorm_FgL(1)
final_enorm_gFR = norm(RFdiff)
enorm_FgR(1)

% Initial plot of data
subplot(3,3,1)
plot(1:N,enorm_gL); xlabel('Trial, k'); ylabel('Error Norm'); title('gripper 1');
subplot(3,3,2)
plot(1:N,enorm_gR); xlabel('Trial, k'); ylabel('Error Norm'); title('gripper 2');
subplot(3,3,6)
plot(1:N,enorm_z); xlabel('Trial, k'); ylabel('Error Norm'); title('z axis');
subplot(3,3,5)
plot(1:N,enorm_y); xlabel('Trial, k'); ylabel('Error Norm'); title('y axis');
subplot(3,3,4)
plot(1:N,enorm_x); xlabel('Trial, k'); ylabel('Error Norm');title('x axis');
subplot(3,3,7)
plot(1:N,enorm_FgL); xlabel('Trial, k'); ylabel('Error Norm');title('Left Gripper Force');
subplot(3,3,8)
plot(1:N,enorm_FgR); xlabel('Trial, k'); ylabel('Error Norm');title('Right Gripper Force');

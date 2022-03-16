% for i=1:6
% subplot(2,1,1)
% plot(t, x_pos_plt(:,i));
% title("X axis relative position");
% xlabel("Time (s)");
% ylabel("Displacement (m)");
% hold on
% legend({'Trial 1','Trial 2','Trial 3','Trial 4','Trial 5','Trial 6'},'Location','northeast','FontSize',7)
% subplot(2,1,2)
% plot(t, x_input_plt(:,i));
% title("X axis input signal (10% disturbance)");
% xlabel("Time (s)");
% ylabel("Displacement (m)");
% hold on
% legend({'Trial 1','Trial 2','Trial 3','Trial 4','Trial 5','Trial 6'},'Location','northeast','FontSize',7)
% end

% close all;
% clear;
% clc;

%% Error from the beginning
% save('x_axis_0','enorm_x_0')
% save('x_axis_0_0_5','enorm_x_0_0_5')
% save('x_axis_0_1','enorm_x_0_1')
% save('x_axis_0_1_5','enorm_x_0_1_5')
% save('x_axis_0_2','enorm_x_0_2')

% load('x_axis_0.mat');
% load('x_axis_0_0_5.mat');
% load('x_axis_0_1.mat');
% load('x_axis_0_1_5.mat');
% load('x_axis_0_2.mat');
% 
% plot(enorm_x_0);
% hold on
% plot(enorm_x_0_0_5);
% plot(enorm_x_0_1);
% plot(enorm_x_0_1_5);
% plot(enorm_x_0_2);
% xlabel("Iteration (k)");
% ylabel("Error Norm");
% title('Normalised error for different iteration under different magnitude of disturbance')
% 
% legend({'0% Disturbance','5% Disturbance','10% Disturbance','15% Disturbance','20% Disturbance',},'Location','northeast','FontSize',7)


%% Error comes midway, after 5 iterations
% save('x_axis_0_mid','enorm_x_0_mid')
% save('x_axis_0_0_5_mid','enorm_x_0_0_5_mid')
% save('x_axis_0_1_mid','enorm_x_0_1_mid')
% save('x_axis_0_1_5_mid','enorm_x_0_1_5_mid')
% save('x_axis_0_2_mid','enorm_x_0_2_mid')

% load('x_axis_0_mid.mat');
% load('x_axis_0_0_5_mid.mat');
% load('x_axis_0_1_mid.mat');
% load('x_axis_0_1_5_mid.mat');
% load('x_axis_0_2_mid.mat');
% 
% plot(enorm_x_0_mid);
% hold on
% plot(enorm_x_0_0_5_mid);
% plot(enorm_x_0_1_mid);
% plot(enorm_x_0_1_5_mid);
% plot(enorm_x_0_2_mid);
% xlabel("Iteration (k)");
% ylabel("Error Norm");
% title('Normalised error for different iteration under different magnitude of disturbance (disturbance occurs at 5th itertaion)')
% 
% legend({'0% Disturbance','5% Disturbance','10% Disturbance','15% Disturbance','20% Disturbance',},'Location','northeast','FontSize',7)

%% AC disturbance at 10th iteration.

for i=8:13
    plot(t,x_pos_plt(:,i));
    hold on

end
xlabel("Time (s)");
ylabel("Displacement (m)");
ylim([0.28, 0.52]);
title('Position feedback of X axis (20Hz disturbance at 10th iteration)')
legend({'8th Iteration','9th Iteration','10th Iteration','11th Iteration','12th Iteration','13th Iteration'},'Location','northeast','FontSize',7)
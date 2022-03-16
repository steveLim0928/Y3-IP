% for i=1:6
% subplot(2,1,1)
% plot(t, y_pos_plt(:,i));
% title("Y axis relative position");
% xlabel("Time (s)");
% ylabel("Displacement (m)");
% hold on
% legend({'Trial 1','Trial 2','Trial 3','Trial 4','Trial 5','Trial 6'},'Location','northeast','FontSize',7)
% subplot(2,1,2)
% plot(t, y_input_plt(:,i));
% title("Y axis input signal (No disturbance)");
% xlabel("Time (s)");
% ylabel("Displacement (m)");
% hold on
% legend({'Trial 1','Trial 2','Trial 3','Trial 4','Trial 5','Trial 6'},'Location','northeast','FontSize',7)
% end

% close all;
% clear;
% clc;

%% Error from the beginning
% save('y_axis_0','enorm_y_0')
% save('y_axis_0_0_5','enorm_y_0_0_5')
% save('y_axis_0_1','enorm_y_0_1')


% load('y_axis_0.mat');
% load('y_axis_0_0_5.mat');
% load('y_axis_0_1.mat');
% 
% 
% plot(enorm_y_0);
% hold on
% plot(enorm_y_0_0_5);
% plot(enorm_y_0_1);
% xlabel("Iteration (k)");
% ylabel("Error Norm");
% title('Normalised error for different iteration under different magnitude of disturbance')
% 
% legend({'0% Disturbance','5% Disturbance','10% Disturbance'},'Location','northeast','FontSize',7)


%% Error comes midway, after 5 iterations
% save('y_axis_0_mid','enorm_y_0_mid')
% save('y_axis_0_0_5_mid','enorm_y_0_0_5_mid')
% save('y_axis_0_1_mid','enorm_y_0_1_mid')

% load('y_axis_0_mid.mat');
% load('y_axis_0_0_5_mid.mat');
% load('y_axis_0_1_mid.mat');
% 
% 
% plot(enorm_y_0_mid);
% hold on
% plot(enorm_y_0_0_5_mid);
% plot(enorm_y_0_1_mid);
% 
% xlabel("Iteration (k)");
% ylabel("Error Norm");
% title('Normalised error for different iteration under different magnitude of disturbance (disturbance occurs at 5th itertaion)')
% 
% legend({'0% Disturbance','5% Disturbance','10% Disturbance'},'Location','northeast','FontSize',7)

%% DC disturbance at 5th iteration

% plot(1:N,enorm_y); xlabel('Time (s)'); ylabel('Error Norm'); title('y axis');
% xticks([1:N]);

% for i=8:13
%     plot(t,y_pos_plt(:,i));
%     hold on
% 
% end
% xlabel("Time (s)");
% ylabel("Displacement (m)");
% xlim([2,3])
% title('Position feedback of Y axis (10% disturbance at 10th iteration)')
% legend({'8th Iteration','9th Iteration','10th Iteration','11th Iteration','12th Iteration','13th Iteration'},'Location','northeast','FontSize',7)


%% AC disturbance at 10th iteration.

% for i=8:13
%     plot(t,y_pos_plt(:,i));
%     hold on
% 
% end
% xlabel("Time (s)");
% ylabel("Displacement (m)");
% ylim([0.08, 0.32]);
% title('Position feedback of Y axis (20Hz disturbance at 10th iteration)')
% legend({'8th Iteration','9th Iteration','10th Iteration','11th Iteration','12th Iteration','13th Iteration'},'Location','northeast','FontSize',7)

% for i=8:13
%     subplot(1,2,1)
%     plot(t,gripL_FNormal_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Force (N)");
%     title('Force feedback of Left Gripper (20Hz disturbance at 10th iteration)')
%     legend({'8th Iteration','9th Iteration','10th Iteration','11th Iteration','12th Iteration','13th Iteration'},'Location','northeast','FontSize',7)
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_FNormal_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Force (N)");
%     title('Force feedback of Right Gripper (20Hz disturbance at 10th iteration)')
%     legend({'8th Iteration','9th Iteration','10th Iteration','11th Iteration','12th Iteration','13th Iteration'},'Location','northeast','FontSize',7)
%     hold on
% 
% end

i = 8;
subplot(1,2,1)
plot(t,gripL_FNormal_plt(:,i));
xlabel("Time (s)");
ylabel("Force (N)");
title('Force feedback of Left Gripper (20Hz disturbance at 10th iteration)')
legend({'8th Iteration','10th Iteration','12th Iteration','14th Iteration'},'Location','northeast','FontSize',7)
hold on
subplot(1,2,2)
plot(t,gripR_FNormal_plt(:,i));
xlabel("Time (s)");
ylabel("Force (N)");
title('Force feedback of Right Gripper (20Hz disturbance at 10th iteration)')
legend({'8th Iteration','10th Iteration','12th Iteration','14th Iteration'},'Location','northeast','FontSize',7)
hold on

i = 10;
subplot(1,2,1)
plot(t,gripL_FNormal_plt(:,i));
xlabel("Time (s)");
ylabel("Force (N)");
title('Force feedback of Left Gripper (20Hz disturbance at 10th iteration)')
legend({'8th Iteration','10th Iteration','12th Iteration','14th Iteration'},'Location','northeast','FontSize',7)
subplot(1,2,2)
plot(t,gripR_FNormal_plt(:,i));
xlabel("Time (s)");
ylabel("Force (N)");
title('Force feedback of Right Gripper (20Hz disturbance at 10th iteration)')
legend({'8th Iteration','10th Iteration','12th Iteration','14th Iteration'},'Location','northeast','FontSize',7)

i = 12;
subplot(1,2,1)
plot(t,gripL_FNormal_plt(:,i));
xlabel("Time (s)");
ylabel("Force (N)");
title('Force feedback of Left Gripper (20Hz disturbance at 10th iteration)')
legend({'8th Iteration','10th Iteration','12th Iteration','14th Iteration'},'Location','northeast','FontSize',7)
subplot(1,2,2)
plot(t,gripR_FNormal_plt(:,i));
xlabel("Time (s)");
ylabel("Force (N)");
title('Force feedback of Right Gripper (20Hz disturbance at 10th iteration)')
legend({'8th Iteration','10th Iteration','12th Iteration','14th Iteration'},'Location','northeast','FontSize',7)

i = 14;
subplot(1,2,1)
plot(t,gripL_FNormal_plt(:,i));
xlabel("Time (s)");
ylabel("Force (N)");
title('Force feedback of Left Gripper (20Hz disturbance at 10th iteration)')
legend({'8th Iteration','10th Iteration','12th Iteration','14th Iteration'},'Location','northeast','FontSize',7)
subplot(1,2,2)
plot(t,gripR_FNormal_plt(:,i));
xlabel("Time (s)");
ylabel("Force (N)");
title('Force feedback of Right Gripper (20Hz disturbance at 10th iteration)')
legend({'8th Iteration','10th Iteration','12th Iteration','14th Iteration'},'Location','northeast','FontSize',7)


for i=30:35
subplot(2,1,1)
plot(t, gripL_pos_plt(:,i));
title("Left Gripper relative position");
xlabel("Time (s)");
ylabel("Displacement (m)");
hold on
legend({'Trial 30','Trial 31','Trial 32','Trial 33','Trial 34','Trail 35'},'Location','northeast','FontSize',7)
subplot(2,1,2)
plot(t, gL_input_plt(:,i));
title("Left Gripper input signal (No disturbance)");
xlabel("Time (s)");
ylabel("Input signal");
hold on
legend({'Trial 30','Trial 31','Trial 32','Trial 33','Trial 34','Trail 35'},'Location','northeast','FontSize',7)
end

% close all;
% clear;
% clc;

%% Error from the beginning
% save('gL_axis_0','enorm_gL_0')
% save('gL_axis_0_0_5','enorm_gL_0_0_5')
% save('gL_axis_0_1','enorm_gL_0_1')

% 
% load('gL_axis_0.mat');
% load('gL_axis_0_0_5.mat');
% load('gL_axis_0_1.mat');
% 
% 
% plot(enorm_gL_0);
% hold on
% plot(enorm_gL_0_0_5);
% plot(enorm_gL_0_1);
% xlabel("Iteration (k)");
% ylabel("Error Norm");
% xticks([1:N]);
% title('Normalised error for different iteration under different magnitude of disturbance')
% 
% legend({'0% Disturbance','5% Disturbance','10% Disturbance'},'Location','northeast','FontSize',7)

% save('gR_axis_0','enorm_gR_0')
% save('gR_axis_0_0_5','enorm_gR_0_0_5')
% save('gR_axis_0_1','enorm_gR_0_1')


%% Error comes midway, after 5 iterations
% save('z_axis_0_mid','enorm_z_0_mid')
% save('z_axis_0_0_5_mid','enorm_z_0_0_5_mid')
% save('z_axis_0_1_mid','enorm_z_0_1_mid')

% load('z_axis_0_mid.mat');
% load('z_axis_0_0_5_mid.mat');
% load('z_axis_0_1_mid.mat');
% 
% 
% plot(enorm_z_0_mid);
% hold on
% plot(enorm_z_0_0_5_mid);
% plot(enorm_z_0_1_mid);
% 
% xlabel("Iteration (k)");
% ylabel("Error Norm");
% xticks(linspace(1,i,15))
% title('Normalised error for different iteration under different magnitude of disturbance (disturbance occurs at 10th itertaion)')
% 
% legend({'0% Disturbance','5% Disturbance','10% Disturbance'},'Location','northeast','FontSize',7)

%% Impulse disturbance at 10th iteration

% plot(1:N,enorm_z); xlabel('Time (s)'); ylabel('Error Norm'); title('z axis');
% xlim([25,35])
% xticks([1:N]);

% for i=29:35
%     plot(t,z_pos_plt(:,i));
%     hold on
% 
% end
% xlabel("Time (s)");
% ylabel("Displacement (m)");
% xlim([2,3])
% title('Position feedback of Y axis (5% disturbance at 10th iteration)')
% legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)

% for i=29:35
%     subplot(1,2,1)
%     plot(t,gripL_FNormal_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Force (N)");
%     title('Left Gripper Force feedback (10% disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([5,8]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_FNormal_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Force (N)");
%     title('Right Gripper Force feedback (5% disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     hold on
%     ylim([5,8]);
% 
% end

% xlim([2,3])



%% AC disturbance at 10th iteration.

% for i=29:35
%     plot(t,z_pos_plt(:,i));
%     hold on
% 
% end
% xlabel("Time (s)");
% ylabel("Displacement (m)");
% % xlim([2, 3]);
% title('Position feedback of z axis (10Hz disturbance at 30th iteration)')
% legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)

% for i=32:35
%     subplot(1,2,1)
%     plot(t,gripL_FNormal_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Force (N)");
%     title('Force feedback of Left Gripper (20Hz disturbance at 30th iteration)')
%     legend({'32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([4.5,8.5]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_FNormal_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Force (N)");
%     title('Force feedback of Right Gripper (20Hz disturbance at 30th iteration)')
%     legend({'32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([4.5,8.5]);
%     hold on
% 
% end

% for i=33:35
%     subplot(1,2,1)
%     plot(t,gripL_pos_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Displacement (m)");
%     title('Position feedback of Left Gripper (20Hz disturbance at 30th iteration)')
%     legend({'33th Iteration','34th Iteration','35th Iteration','29 iteration'},'Location','northeast','FontSize',7)
%     ylim([0.0115,0.0129]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_pos_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Displacement (m)");
%     title('Position feedback of Right Gripper (20Hz disturbance at 30th iteration)')
%     legend({'33th Iteration','34th Iteration','35th Iteration','29 iteration'},'Location','northeast','FontSize',7)
%     ylim([0.0115,0.0129]);
%     hold on
% 
% end
% 
% subplot(1,2,1)
% plot(t,gripL_pos_plt(:,29));
% legend({'33th Iteration','34th Iteration','35th Iteration','29th iteration'},'Location','northeast','FontSize',7)
% subplot(1,2,2)
% plot(t,gripR_pos_plt(:,29));
% legend({'33th Iteration','34th Iteration','35th Iteration','29th iteration'},'Location','northeast','FontSize',7)



%% Ideal case
close all;

% subplot(1,2,1)
% plot(1:N,enorm_gL); xlabel('Iteration (k)'); ylabel('Error Norm'); title('Left Gripper Position Error');
% % xticks([1:N]);
% subplot(1,2,2)
% plot(1:N,enorm_gR); xlabel('Iteration (k)'); ylabel('Error Norm'); title('Right Gripper Position Error');
% % xticks([1:N]);

% subplot(1,2,1)
% plot(1:N,enorm_FgL); xlabel('Iteration (k)'); ylabel('Error Norm'); title('Left Gripper Force Error');
% % xticks([1:N]);
% subplot(1,2,2)
% plot(1:N,enorm_FgR); xlabel('Iteration (k)'); ylabel('Error Norm'); title('Right Gripper Force Error');
% % xticks([1:N]);

% subplot(2,1,1);
% plot(t,u_gL_d);
% xlabel('Time (s)');
% ylabel('Amplitude');
% title("Left Gripper Input Signal");
% subplot(2,1,2);
% plot(t,u_gR_d);
% xlabel('Time (s)');
% ylabel('Amplitude');
% title("Right Gripper Input Signal");

% subplot(2,1,1);
% plot(t,gripL_FNormal_plt(:,30));
% xlabel('Time (s)');
% ylabel('Force (s)');
% title("Left Gripper Force Feedback (30th iteration)");
% subplot(2,1,2);
% plot(t,gripR_FNormal_plt(:,30));
% xlabel('Time (s)');
% ylabel('Force (s)');
% title("Left Gripper Force Feedback (30th iteration)");

% subplot(2,1,1);
% plot(t,gripL_FNormal_plt(:,35));
% xlabel('Time (s)');
% ylabel('Force (s)');
% title("Left Gripper Force Feedback (35th iteration)");
% subplot(2,1,2);
% plot(t,gripR_FNormal_plt(:,35));
% xlabel('Time (s)');
% ylabel('Force (s)');
% title("Left Gripper Force Feedback (35th iteration)");

plot(t,ygL(:,N));
xlabel('Time (s)');
ylabel('Displacement (m)');
title("Left Gripper Position Feedback");



% for i=30:35
% subplot(2,1,1)
% plot(t, gripL_pos_plt(:,i));
% title("Left Gripper relative position");
% xlabel("Time (s)");
% ylabel("Displacement (m)");
% hold on
% legend({'Trial 30','Trial 31','Trial 32','Trial 33','Trial 34','Trail 35'},'Location','northeast','FontSize',7)
% subplot(2,1,2)
% plot(t, gL_input_plt(:,i));
% title("Left Gripper input signal (No disturbance)");
% xlabel("Time (s)");
% ylabel("Input signal");
% hold on
% legend({'Trial 30','Trial 31','Trial 32','Trial 33','Trial 34','Trail 35'},'Location','northeast','FontSize',7)
% end

% close all;
% clear;
% clc;

%% Error from the beginning
% save('gL_axis_0','enorm_gL_0')
% save('gR_axis_0','enorm_gR_0')
% save('FgL_axis_0','enorm_FgL_0')
% save('FgR_axis_0','enorm_FgR_0')

% save('gL_axis_0_0_5','enorm_gL_0_0_5')
% save('gR_axis_0_0_5','enorm_gR_0_0_5')
% save('FgL_axis_0_0_5','enorm_FgL_0_0_5')
% save('FgR_axis_0_0_5','enorm_FgR_0_0_5')

% save('gL_axis_0_1','enorm_gL_0_1')
% save('gR_axis_0_1','enorm_gR_0_1')
% save('FgL_axis_0_1','enorm_FgL_0_1')
% save('FgR_axis_0_1','enorm_FgR_0_1')

% 
% load('gL_axis_0.mat');
% load('gL_axis_0_0_5.mat');
% load('gL_axis_0_1.mat');
% load('gR_axis_0.mat');
% load('gR_axis_0_0_5.mat');
% load('gR_axis_0_1.mat');
% 
% subplot(1,2,1)
% plot(enorm_gL_0);
% hold on
% plot(enorm_gL_0_0_5);
% plot(enorm_gL_0_1);
% xlabel("Iteration (k)");
% ylabel("Error Norm");
% xticks([1:N]);
% title('Left gripper position error (constant disturbance at 1st iteration)');
% 
% legend({'0% Disturbance','5% Disturbance','10% Disturbance'},'Location','northeast','FontSize',7);
% 
% subplot(1,2,2)
% plot(enorm_gR_0);
% hold on
% plot(enorm_gR_0_0_5);
% plot(enorm_gR_0_1);
% xlabel("Iteration (k)");
% ylabel("Error Norm");
% xticks([1:N]);
% title('Right gripper position error (constant disturbance at 1st iteration)');
% 
% legend({'0% Disturbance','5% Disturbance','10% Disturbance'},'Location','northeast','FontSize',7);


%

% load('FgL_axis_0.mat');
% load('FgL_axis_0_0_5.mat');
% load('FgL_axis_0_1.mat');
% load('FgR_axis_0.mat');
% load('FgR_axis_0_0_5.mat');
% load('FgR_axis_0_1.mat');
% 
% subplot(1,2,1)
% plot(enorm_FgL_0);
% hold on
% plot(enorm_FgL_0_0_5);
% plot(enorm_FgL_0_1);
% xlabel("Iteration (k)");
% ylabel("Error Norm");
% xticks([1:N]);
% title('Left gripper Force error (constant disturbance at 1st iteration)')
% 
% legend({'0% Disturbance','5% Disturbance','10% Disturbance'},'Location','northeast','FontSize',7)
% 
% subplot(1,2,2)
% plot(enorm_FgR_0);
% hold on
% plot(enorm_FgR_0_0_5);
% plot(enorm_FgR_0_1);
% xlabel("Iteration (k)");
% ylabel("Error Norm");
% xticks([1:N]);
% title('Right gripper Force error (constant disturbance at 1st iteration)')
% 
% legend({'0% Disturbance','5% Disturbance','10% Disturbance'},'Location','northeast','FontSize',7)



%% Error comes midway, after 5 iterations
% save('gL_axis_0_mid','enorm_gL_0_mid')
% save('gR_axis_0_mid','enorm_gR_0_mid')
% save('FgL_axis_0_mid','enorm_FgL_0_mid')
% save('FgR_axis_0_mid','enorm_FgR_0_mid')

% save('gL_axis_0_0_5_mid','enorm_gL_0_0_5_mid')
% save('gR_axis_0_0_5_mid','enorm_gR_0_0_5_mid')
% save('FgL_axis_0_0_5_mid','enorm_FgL_0_0_5_mid')
% save('FgR_axis_0_0_5_mid','enorm_FgR_0_0_5_mid')

% save('gL_axis_0_1_mid','enorm_gL_0_1_mid')
% save('gR_axis_0_1_mid','enorm_gR_0_1_mid')
% save('FgL_axis_0_1_mid','enorm_FgL_0_1_mid')
% save('FgR_axis_0_1_mid','enorm_FgR_0_1_mid')


% 
% load('gL_axis_0_mid.mat');
% load('gL_axis_0_0_5_mid.mat');
% load('gL_axis_0_1_mid.mat');
% load('gR_axis_0_mid.mat');
% load('gR_axis_0_0_5_mid.mat');
% load('gR_axis_0_1_mid.mat');
% 
% subplot(1,2,1)
% plot(enorm_gL_0_mid);
% hold on
% plot(enorm_gL_0_0_5_mid);
% plot(enorm_gL_0_1_mid);
% xlabel("Iteration (k)");
% ylabel("Error Norm");
% xticks([1:N]);
% title('Left gripper Position error (constant disturbance at 30th iteration)');
% 
% legend({'0% Disturbance','5% Disturbance','10% Disturbance'},'Location','northeast','FontSize',7);
% 
% subplot(1,2,2)
% plot(enorm_gR_0_mid);
% hold on
% plot(enorm_gR_0_0_5_mid);
% plot(enorm_gR_0_1_mid);
% xlabel("Iteration (k)");
% ylabel("Error Norm");
% xticks([1:N]);
% title('Right gripper Position error (constant disturbance at 30th iteration)');
% 
% legend({'0% Disturbance','5% Disturbance','10% Disturbance'},'Location','northeast','FontSize',7);


%

% load('FgL_axis_0_mid.mat');
% load('FgL_axis_0_0_5_mid.mat');
% load('FgL_axis_0_1_mid.mat');
% load('FgR_axis_0_mid.mat');
% load('FgR_axis_0_0_5_mid.mat');
% load('FgR_axis_0_1_mid.mat');
% 
% subplot(1,2,1)
% plot(enorm_FgL_0_mid);
% hold on
% plot(enorm_FgL_0_0_5_mid);
% plot(enorm_FgL_0_1_mid);
% xlabel("Iteration (k)");
% ylabel("Error Norm");
% xticks([1:N]);
% title('Left gripper Force error (constant disturbance at 30th iteration)');
% legend({'0% Disturbance','5% Disturbance','10% Disturbance'},'Location','northeast','FontSize',7);
% 
% subplot(1,2,2)
% plot(enorm_FgR_0_mid);
% hold on
% plot(enorm_FgR_0_0_5_mid);
% plot(enorm_FgR_0_1_mid);
% xlabel("Iteration (k)");
% ylabel("Error Norm");
% xticks([1:N]);
% title('Right gripper Force error (constant disturbance at 30th iteration)');
% legend({'0% Disturbance','5% Disturbance','10% Disturbance'},'Location','northeast','FontSize',7);

%  for i=30:35
%     subplot(1,2,1)
%     plot(t,gripL_FNormal_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Force (N)");
%     title('Left Gripper Force feedback (5% disturbance at 30th iteration)')
%     legend({'30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration','36th Iteration','37th Iteration'},'Location','northeast','FontSize',7)
% 
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_FNormal_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Force (N)");
%     title('Right Gripper Force feedback (5% disturbance at 30th iteration)')
%     legend({'30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration','36th Iteration','37th Iteration'},'Location','northeast','FontSize',7)
%     hold on
% 
% 
% end

%% Impulse disturbance at 30th iteration

% close all;

% subplot(1,2,1)
% plot(1:N,enorm_gL); xlabel('Iteration (k)'); ylabel('Error Norm'); title('Left Gripper Position Feedback');
% xlim([25,40])
% xticks([1:N]);
% subplot(1,2,2)
% plot(1:N,enorm_gR); xlabel('Iteration (k)'); ylabel('Error Norm'); title('Right Gripper Position Feedback');
% xlim([25,40])
% xticks([1:N]);

% subplot(1,2,1)
% plot(1:N,enorm_FgL); xlabel('Iteration (k)'); ylabel('Error Norm'); title('Left Gripper Force Feedback');
% xlim([25,40])
% xticks([1:N]);
% subplot(1,2,2)
% plot(1:N,enorm_FgR); xlabel('Iteration (k)'); ylabel('Error Norm'); title('Right Gripper Force Feedback');
% xlim([25,40])
% xticks([1:N]);

% for i=29:35
%     plot(t,gripL_pos_plt(:,i));
%     hold on
% 
% end
% xlabel("Time (s)");
% ylabel("Displacement (m)");
% xlim([1,4])
% ylim([0.0115,0.0135])
% title('Position feedback of Left gripper (10% disturbance at 30th iteration)')
% legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)

% for i=29:35
%     plot(t,gripR_pos_plt(:,i));
%     hold on
% 
% end
% xlabel("Time (s)");
% ylabel("Displacement (m)");
% xlim([1,4])
% ylim([0.011,0.0135])
% title('Position feedback of Right gripper (10% disturbance at 30th iteration)')
% legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)

% for i=29:35
%     subplot(1,2,1)
%     plot(t,gripL_FNormal_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Force (N)");
%     title('Left Gripper Force feedback (5% disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
% %     ylim([5,8]);
%     xlim([1,4]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_FNormal_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Force (N)");
%     title('Right Gripper Force feedback (5% disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     xlim([1,4]);
%     hold on
% %     ylim([5,8]);
% 
% end

% xlim([2,3])



%% AC disturbance at 10th iteration.

% close all;

% subplot(1,2,1)
% plot(1:N,enorm_gL); xlabel('Iteration (k)'); ylabel('Error Norm'); title('Left Gripper Position Feedback');
% xlim([25,40])
% xticks([1:N]);
% subplot(1,2,2)
% plot(1:N,enorm_gR); xlabel('Iteration (k)'); ylabel('Error Norm'); title('Right Gripper Position Feedback');
% xlim([25,40])
% xticks([1:N]);

% subplot(1,2,1)
% plot(1:N,enorm_FgL); xlabel('Iteration (k)'); ylabel('Error Norm'); title('Left Gripper Force Feedback');
% xlim([25,40])
% xticks([1:N]);
% subplot(1,2,2)
% plot(1:N,enorm_FgR); xlabel('Iteration (k)'); ylabel('Error Norm'); title('Right Gripper Force Feedback');
% xlim([25,40])
% xticks([1:N]);

% for i=29:35
%     subplot(1,2,1)
%     plot(t,gripL_pos_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Displacement (m)");
%     title('Position feedback of Left Gripper (20Hz disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([0.012,0.0125]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_pos_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Displacement (m)");
%     title('Position feedback of Right Gripper (20Hz disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([0.0117,0.0121]);
%     hold on
% 
% end

% for i=29:35
%     subplot(1,2,1)
%     plot(t,gripL_FNormal_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Force (N)");
%     title('Force feedback of Left Gripper (20Hz disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([3.5,8.5]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_FNormal_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Force (N)");
%     title('Force feedback of Right Gripper (20Hz disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([3.5,8.5]);
%     hold on
% 
% end

% for i=29:35
%     subplot(1,2,1)
%     plot(t,gL_input_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Amplitude");
%     title('Left Gripper Input Signal (20Hz disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([19,21]);
%     xlim([1,4]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gR_input_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Amplitude");
%     title('Right Gripper Input Signal (20Hz disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([19,21]);
%     xlim([1,4]);
%     hold on
% 
% end



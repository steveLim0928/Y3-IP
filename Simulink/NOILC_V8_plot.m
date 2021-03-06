%% Ideal case
close all;

% plot(1:N,enorm_x); xlabel('Iteration (k)'); ylabel('Error Norm');title('X Axis Position Error');

% plot(t,u_x); xlabel('Time (s)'); ylabel('Amplitude');title('X Axis Input Signal');
% plot(t,yx); xlabel('Time (s)'); ylabel('Displacement (m)');title('X Axis Position Feedback');

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


% load('x_axis_0.mat');
% load('x_axis_0_0_5.mat');
% load('x_axis_0_1.mat');
% 
% plot(enorm_x_0(1:10));
% hold on
% plot(enorm_x_0_0_5(1:10));
% plot(enorm_x_0_1(1:10));
% xlabel("Iteration (k)");
% ylabel("Error Norm");
% title('X axis position error (constant disturbance at 1st iteration)');
% xticks([1:15]);
% 
% legend({'0% Disturbance','5% Disturbance','10% Disturbance'},'Location','northeast','FontSize',7)


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


%% Error comes midway, after 30 iterations
close all;

% save('x_axis_0_mid','enorm_x_0_mid')
% save('x_axis_0_0_5_mid','enorm_x_0_0_5_mid')
% save('x_axis_0_1_mid','enorm_x_0_1_mid')

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

% load('x_axis_0_mid.mat');
% load('x_axis_0_0_5_mid.mat');
% load('x_axis_0_1_mid.mat');
% 
% plot(enorm_x_0_mid);
% hold on
% plot(enorm_x_0_0_5_mid);
% plot(enorm_x_0_1_mid);
% xlabel("Iteration (k)");
% ylabel("Error Norm");
% title('X axis position error (constant disturbance at 30th itertaion)')
% 
% legend({'0% Disturbance','5% Disturbance','10% Disturbance'},'Location','northeast','FontSize',7)

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
% ylim([0,0.02]);
% title('Left gripper position error (constant disturbance at 30th iteration)');
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
% ylim([0,0.02]);
% title('Right gripper position error (constant disturbance at 30th iteration)');
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
% title('Left gripper Force error (constant disturbance at 30th iteration)')
% 
% legend({'0% Disturbance','5% Disturbance','10% Disturbance'},'Location','northeast','FontSize',7)
% 
% subplot(1,2,2)
% plot(enorm_FgR_0_mid);
% hold on
% plot(enorm_FgR_0_0_5_mid);
% plot(enorm_FgR_0_1_mid);
% xlabel("Iteration (k)");
% ylabel("Error Norm");
% xticks([1:N]);
% title('Right gripper Force error (constant disturbance at 30th iteration)')
% 
% legend({'0% Disturbance','5% Disturbance','10% Disturbance'},'Location','northeast','FontSize',7)

% x axis

% for i=29:35
% 
%     plot(t,x_pos_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Displacement (m)");
%     title('X axis Position Feedback (10% at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([0.29,0.58]);
% %     xlim([1,4]);
%     hold on
% 
% end

% for i=29:35
% 
%     plot(t,x_input_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Amplitude");
%     title('X axis Input Signal (10% at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([3,6.5]);
% %     xlim([1,4]);
%     hold on
% 
% end

% Gripper

% for i=29:35
%     subplot(1,2,1)
%     plot(t,gripL_pos_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Displacement (m)");
%     title('Position feedback of Left Gripper (10% disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([0.0119,0.0135]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_pos_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Displacement (m)");
%     title('Position feedback of Right Gripper (10% disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([0.011,0.013]);
%     hold on
% 
% end

% for i=29:35
%     subplot(1,2,1)
%     plot(t,gripL_FNormal_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Force (N)");
%     title('Force feedback of Left Gripper (10% disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([3.5,9]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_FNormal_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Force (N)");
%     title('Force feedback of Right Gripper (10% disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([3.5,9.2]);
%     hold on
% 
% end

% for i=29:35
%     subplot(1,2,1)
%     plot(t,gL_input_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Amplitude");
%     title('Left Gripper Input Signal (10% disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([20,21]);
%     xlim([1,4]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gR_input_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Amplitude");
%     title('Right Gripper Input Signal (10% disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([18.5,20.5]);
%     xlim([1,4]);
%     hold on
% 
% end

%% Impulse at 30th 
close all;

% x axis

% plot(1:N,enorm_x); xlabel('Iteration (k)'); ylabel('Error Norm');title('X Axis Position Error');

% for i=29:35
% 
%     plot(t,x_pos_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Displacement (m)");
%     title('X axis Position Feedback (5% at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([0.29,0.58]);
% %     xlim([1,4]);
%     hold on
% 
% end

% for i=29:35
% 
%     plot(t,x_input_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Amplitude");
%     title('X axis Input Signal (5% at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([3,6.5]);
% %     xlim([1,4]);
%     hold on
% 
% end

% Gripper

% subplot(1,2,1);
% plot(1:N,enorm_gL); xlabel('Iteration (k)'); ylabel('Error Norm');title('Left Gripper Position Error');
% subplot(1,2,2);
% plot(1:N,enorm_gR); xlabel('Iteration (k)'); ylabel('Error Norm');title('Right Gripper Position Error');

% subplot(1,2,1);
% plot(1:N,enorm_FgL); xlabel('Iteration (k)'); ylabel('Error Norm');title('Left Gripper Force Error');
% subplot(1,2,2);
% plot(1:N,enorm_FgR); xlabel('Iteration (k)'); ylabel('Error Norm');title('Right Gripper Force Error');

% for i=29:35
%     subplot(1,2,1)
%     plot(t,gripL_pos_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Displacement (m)");
%     title('Position feedback of Left Gripper (5% impulse disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([0.0119,0.0135]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_pos_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Displacement (m)");
%     title('Position feedback of Right Gripper (5% impulse disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([0.011,0.013]);
%     hold on
% 
% end

% for i=29:35
%     subplot(1,2,1)
%     plot(t,gripL_FNormal_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Force (N)");
%     title('Force feedback of Left Gripper (5% impulse disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([3.5,9]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_FNormal_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Force (N)");
%     title('Force feedback of Right Gripper (5% impulse disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([3.5,9.2]);
%     hold on
% 
% end

% for i=29:35
%     subplot(1,2,1)
%     plot(t,gL_input_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Amplitude");
%     title('Left Gripper Input Signal (5% impulse disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([19.5,21]);
%     xlim([1,4]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gR_input_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Amplitude");
%     title('Right Gripper Input Signal (5% impulse disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([18.5,20.5]);
%     xlim([1,4]);
%     hold on
% 
% end



%% AC disturbance at 30th iteration.

close all;

% x axis

% plot(1:N,enorm_x); xlabel('Iteration (k)'); ylabel('Error Norm');title('X Axis Position Error');

% for i=29:35
% 
%     plot(t,x_pos_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Displacement (m)");
%     title('X axis Position Feedback (20Hz at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([0.29,0.58]);
% %     xlim([1,4]);
%     hold on
% 
% end

% for i=29:35
% 
%     plot(t,x_input_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Amplitude");
%     title('X axis Input Signal (20Hz at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([3,6.5]);
% %     xlim([1,4]);
%     hold on
% 
% end

% Gripper

% subplot(1,2,1);
% plot(1:N,enorm_gL); xlabel('Iteration (k)'); ylabel('Error Norm');title('Left Gripper Position Error');
% subplot(1,2,2);
% plot(1:N,enorm_gR); xlabel('Iteration (k)'); ylabel('Error Norm');title('Right Gripper Position Error');

% subplot(1,2,1);
% plot(1:N,enorm_FgL); xlabel('Iteration (k)'); ylabel('Error Norm');title('Left Gripper Force Error');
% subplot(1,2,2);
% plot(1:N,enorm_FgR); xlabel('Iteration (k)'); ylabel('Error Norm');title('Right Gripper Force Error');

% for i=29:35
%     subplot(1,2,1)
%     plot(t,gripL_pos_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Displacement (m)");
%     title('Position feedback of Left Gripper (20Hz disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([0.0119,0.014]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_pos_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Displacement (m)");
%     title('Position feedback of Right Gripper (20Hz disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([0.011,0.014]);
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
%     ylim([3.5,10.5]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_FNormal_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Force (N)");
%     title('Force feedback of Right Gripper (20Hz disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([3.5,10.7]);
%     hold on
% 
% end

for i=29:35
    subplot(1,2,1)
    plot(t,gL_input_plt(:,i));
    xlabel("Time (s)");
    ylabel("Amplitude");
    title('Left Gripper Input Signal (20Hz disturbance at 30th iteration)')
    legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
    ylim([20,22]);
    xlim([1,4]);
    hold on
    subplot(1,2,2)
    plot(t,gR_input_plt(:,i));
    xlabel("Time (s)");
    ylabel("Amplitude");
    title('Right Gripper Input Signal (20Hz disturbance at 30th iteration)')
    legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
    ylim([19,21]);
    xlim([1,4]);
    hold on

end


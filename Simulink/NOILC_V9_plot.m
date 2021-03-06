%% Ideal case
close all;

% plot(1:N,enorm_y); xlabel('Iteration (k)'); ylabel('Error Norm');title('Y Axis Position Error');

% plot(t,u_y); xlabel('Time (s)'); ylabel('Amplitude');title('Y Axis Input Signal');
% plot(t,yy); xlabel('Time (s)'); ylabel('Displacement (m)');title('Y Axis Position Feedback');


%% Error from the beginning
% save('y_axis_0','enorm_y_0')
% save('y_axis_0_0_5','enorm_y_0_0_5')
% save('y_axis_0_1','enorm_y_0_1')

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


% load('y_axis_0.mat');
% load('y_axis_0_0_5.mat');
% load('y_axis_0_1.mat');
% 
% plot(enorm_y_0(1:10));
% hold on
% plot(enorm_y_0_0_5(1:10));
% plot(enorm_y_0_1(1:10));
% xlabel("Iteration (k)");
% ylabel("Error Norm");
% title('Y axis position error (constant disturbance at 1st iteration)');
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

% save('y_axis_0_mid','enorm_y_0_mid')
% save('y_axis_0_0_5_mid','enorm_y_0_0_5_mid')
% save('y_axis_0_1_mid','enorm_y_0_1_mid')

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

% load('y_axis_0_mid.mat');
% load('y_axis_0_0_5_mid.mat');
% load('y_axis_0_1_mid.mat');
% 
% plot(enorm_y_0_mid);
% hold on
% plot(enorm_y_0_0_5_mid);
% plot(enorm_y_0_1_mid);
% xlabel("Iteration (k)");
% ylabel("Error Norm");
% title('Y axis position error (constant disturbance at 30th itertaion)')
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

% y axis

% for i=29:35
% 
%     plot(t,y_pos_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Displacement (m)");
%     title('Y axis Position Feedback (10% at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([0.09,0.35]);
% %     xlim([1,4]);
%     hold on
% 
% end

% for i=29:35
% 
%     plot(t,y_input_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Amplitude");
%     title('Y axis Input Signal (10% at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([30,160]);
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
% close all;

% y axis

% plot(1:N,enorm_y); xlabel('Iteration (k)'); ylabel('Error Norm');title('Y Axis Position Error');

% for i=29:35
% 
%     plot(t,y_pos_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Displacement (m)");
%     title('Y axis Position Feedback (10% at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([0.09,0.35]);
% %     xlim([1,4]);
%     hold on
% 
% end

% for i=29:35
% 
%     plot(t,y_input_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Amplitude");
%     title('Y axis Input Signal (10% at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([30,160]);
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
%     title('Position feedback of Left Gripper (10% impulse disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([0.0119,0.0135]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_pos_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Displacement (m)");
%     title('Position feedback of Right Gripper (10% impulse disturbance at 30th iteration)')
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
%     title('Force feedback of Left Gripper (10% impulse disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([3.5,9]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_FNormal_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Force (N)");
%     title('Force feedback of Right Gripper (10% impulse disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([3.5,9.2]);
%     hold on
% 
% end
% 
% for i=29:35
%     subplot(1,2,1)
%     plot(t,gL_input_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Amplitude");
%     title('Left Gripper Input Signal (10% impulse disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([20,21.5]);
%     xlim([1,4]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gR_input_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Amplitude");
%     title('Right Gripper Input Signal (10% impulse disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([18.5,20.5]);
%     xlim([1,4]);
%     hold on
% 
% end



%% AC disturbance at 30th iteration.

% close all;

% y axis

% plot(1:N,enorm_y); xlabel('Iteration (k)'); ylabel('Error Norm');title('Y Axis Position Error');

% for i=29:35
% 
%     plot(t,y_pos_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Displacement (m)");
%     title('Y axis Position Feedback (20Hz at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([0.09,0.35]);
% %     xlim([1,4]);
%     hold on
% 
% end

% for i=29:35
% 
%     plot(t,y_input_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Amplitude");
%     title('Y axis Input Signal (20Hz at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([30,160]);
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
%     ylim([0.0115,0.014]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_pos_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Displacement (m)");
%     title('Position feedback of Right Gripper (20Hz disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([0.0105,0.0135]);
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
%     ylim([5,14]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_FNormal_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Force (N)");
%     title('Force feedback of Right Gripper (20Hz disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([5,14]);
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
%     ylim([20,22]);
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


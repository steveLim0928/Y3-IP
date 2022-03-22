%% Save data

if (0)
    save('inputx.mat','inputx');
    save('inputy.mat','inputy');
    save('inputz.mat','inputz');
    save('tIn.mat','tIn');
    
    save('LGripRef.mat','LGripRef');
    save('RGripRef.mat','RGripRef');
    
    save('yx.mat','yx');
    save('yy.mat','yy');
    save('yz.mat','yz');
    
    save('ygL.mat','ygL');
    save('ygR.mat','ygR');
    
    save('LFNormal.mat','LFNormal');
    save('RFNormal.mat','RFNormal');
    save('RFFriction.mat','RFFriction');
    
    save('refL.mat','refL');
    save('refR.mat','refR');
    
    save('u_gL.mat','u_gL');
    save('u_gR.mat','u_gR');

    save('x_pos_plt.mat','x_pos_plt');
    save('y_pos_plt.mat','y_pos_plt');
    save('z_pos_plt.mat','z_pos_plt');
    save('gripL_pos_plt.mat','gripL_pos_plt');
    save('gripR_pos_plt.mat','gripR_pos_plt');
    save('gripL_FNormal_plt.mat','gripL_FNormal_plt');
    save('gripR_FNormal_plt.mat','gripR_FNormal_plt');

    save('x_input_plt.mat','x_input_plt');
    save('y_input_plt.mat','y_input_plt');
    save('z_input_plt.mat','z_input_plt');
    save('gL_input_plt.mat','gL_input_plt');
    save('gR_input_plt.mat','gR_input_plt');

    save('enorm_FgL.mat','enorm_FgL');
    save('enorm_FgR.mat','enorm_FgR');
    save('enorm_x.mat','enorm_x');
    save('enorm_y.mat','enorm_y');
    save('enorm_z.mat','enorm_z');
    save('enorm_gL.mat','enorm_gL');
    save('enorm_gR.mat','enorm_gR');
end

if (1)
    load('inputx.mat');
    load('inputy.mat');
    load('inputz.mat');
    load('tIn.mat');
    
    load('LGripRef.mat');
    load('RGripRef.mat');
    
    load('yx.mat');
    load('yy.mat');
    load('yz.mat');
    
    load('ygL.mat');
    load('ygR.mat');
    
    load('LFNormal.mat');
    load('RFNormal.mat');
    load('RFFriction.mat');
    
    load('refL.mat');
    load('refR.mat');
    
    load('u_gL.mat');
    load('u_gR.mat');

    load('x_pos_plt.mat');
    load('y_pos_plt.mat');
    load('z_pos_plt.mat');
    load('gripL_pos_plt.mat');
    load('gripR_pos_plt.mat');
    load('gripL_FNormal_plt.mat');
    load('gripR_FNormal_plt.mat');

    load('x_input_plt.mat');
    load('y_input_plt.mat');
    load('z_input_plt.mat');
    load('gL_input_plt.mat');
    load('gR_input_plt.mat');

    load('enorm_FgL.mat');
    load('enorm_FgR.mat');
    load('enorm_x.mat');
    load('enorm_y.mat');
    load('enorm_z.mat');
    load('enorm_gL.mat');
    load('enorm_gR.mat');

    N = 40;
    t = 0:0.0125:5;
    t(1) = []; % remove t = 0
end



%% Error from the beginning

close all

N = 40;
t = 0:0.0125:5;
t(1) = []; % remove t = 0

% save('z_axis_0','enorm_z_0')
% save('z_axis_0_0_5','enorm_z_0_0_5')
% save('z_axis_0_1','enorm_z_0_1')

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


% load('z_axis_0.mat');
% load('z_axis_0_0_5.mat');
% load('z_axis_0_1.mat');
% 
% plot(enorm_z_0(1:25));
% hold on
% plot(enorm_z_0_0_5(1:25));
% plot(enorm_z_0_1(1:25));
% xlabel("Iteration (k)");
% ylabel("Error Norm");
% title('Z axis position error (constant disturbance at 1st iteration)');
% xticks([1:25]);
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
% close all;

% save('z_axis_0_mid','enorm_z_0_mid')
% save('z_axis_0_0_5_mid','enorm_z_0_0_5_mid')
% save('z_axis_0_1_mid','enorm_z_0_1_mid')

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

% load('z_axis_0.mat');
% load('z_axis_0_0_5_mid.mat');
% load('z_axis_0_1_mid.mat');
% 
% plot(enorm_z_0);
% hold on
% plot(enorm_z_0_0_5_mid);
% plot(enorm_z_0_1_mid);
% xlabel("Iteration (k)");
% ylabel("Error Norm");
% title('Z axis position error (constant disturbance at 30th itertaion)')
% 
% legend({'0% Disturbance','5% Disturbance','10% Disturbance'},'Location','northeast','FontSize',7)

% load('gL_axis_0.mat');
% load('gL_axis_0_0_5_mid.mat');
% load('gL_axis_0_1_mid.mat');
% load('gR_axis_0.mat');
% load('gR_axis_0_0_5_mid.mat');
% load('gR_axis_0_1_mid.mat');
% 
% subplot(1,2,1)
% plot(enorm_gL_0);
% hold on
% plot(enorm_gL_0_0_5_mid);
% plot(enorm_gL_0_1_mid);
% xlabel("Iteration (k)");
% ylabel("Error Norm");
% xticks([1:N]);
% ylim([0,0.02]);
% title('Left gripper position error (constant disturbance at 30th iteration)');
% legend({'0% Disturbance','5% Disturbance','10% Disturbance'},'Location','northeast','FontSize',7);
% 
% subplot(1,2,2)
% plot(enorm_gR_0);
% hold on
% plot(enorm_gR_0_0_5_mid);
% plot(enorm_gR_0_1_mid);
% xlabel("Iteration (k)");
% ylabel("Error Norm");
% xticks([1:N]);
% ylim([0,0.02]);
% title('Right gripper position error (constant disturbance at 30th iteration)');
% legend({'0% Disturbance','5% Disturbance','10% Disturbance'},'Location','northeast','FontSize',7);

% 
% load('FgL_axis_0.mat');
% load('FgL_axis_0_0_5_mid.mat');
% load('FgL_axis_0_1_mid.mat');
% load('FgR_axis_0.mat');
% load('FgR_axis_0_0_5_mid.mat');
% load('FgR_axis_0_1_mid.mat');
% 
% subplot(1,2,1)
% plot(enorm_FgL_0);
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
% plot(enorm_FgR_0);
% hold on
% plot(enorm_FgR_0_0_5_mid);
% plot(enorm_FgR_0_1_mid);
% xlabel("Iteration (k)");
% ylabel("Error Norm");
% xticks([1:N]);
% title('Right gripper Force error (constant disturbance at 30th iteration)')
% 
% legend({'0% Disturbance','5% Disturbance','10% Disturbance'},'Location','northeast','FontSize',7)

% z axis

% for i=29:35
% 
%     plot(t,z_pos_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Displacement (m)");
%     title('Z axis Position Feedback (10% at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([-0.045,-0.02]);
% %     xlim([1,4]);
%     hold on
% 
% end

% for i=29:35
% 
%     plot(t,z_input_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Amplitude");
%     title('Z axis Input Signal (10% at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([-60,-35]);
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
%     ylim([0.011,0.0135]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_pos_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Displacement (m)");
%     title('Position feedback of Right Gripper (10% disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([0.0105,0.013]);
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
%     ylim([4.5,7.5]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_FNormal_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Force (N)");
%     title('Force feedback of Right Gripper (10% disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([5,7]);
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
%     ylim([19,22]);
%     xlim([1,4]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gR_input_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Amplitude");
%     title('Right Gripper Input Signal (10% disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([18,21]);
%     xlim([1,4]);
%     hold on
% 
% end

%% Impulse at 30th 
% close all;

% z axis

% plot(1:N,enorm_z); xlabel('Iteration (k)'); ylabel('Error Norm');title('Z Axis Position Error');

% for i=29:35
% 
%     plot(t,z_pos_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Displacement (m)");
%     title('Z axis Position Feedback (10% at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     hold on
% 
% end

% for i=29:35
% 
%     plot(t,z_input_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Amplitude");
%     title('Z axis Input Signal (10% at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
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
%     title('Position feedback of Left Gripper (10% disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([0.011,0.0140]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_pos_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Displacement (m)");
%     title('Position feedback of Right Gripper (10% disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([0.0105,0.013]);
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
%     ylim([4.5,7.5]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_FNormal_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Force (N)");
%     title('Force feedback of Right Gripper (10% disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([5,7]);
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
%     ylim([19,22]);
%     xlim([1,4]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gR_input_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Amplitude");
%     title('Right Gripper Input Signal (10% disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([18,21]);
%     xlim([1,4]);
%     hold on
% 
% end



%% AC disturbance at 30th iteration.

% close all;

% z axis

% plot(1:N,enorm_z); xlabel('Iteration (k)'); ylabel('Error Norm');title('Z Axis Position Error');

% for i=29:35
% 
%     plot(t,z_pos_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Displacement (m)");
%     title('Z axis Position Feedback (20Hz at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
% %     xlim([1,4]);
%     hold on
% 
% end

% for i=29:35
% 
%     plot(t,z_input_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Amplitude");
%     title('Z axis Input Signal (20Hz at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
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
%     ylim([0.011,0.0135]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_pos_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Displacement (m)");
%     title('Position feedback of Right Gripper (20Hz disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([0.0105,0.013]);
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
%     ylim([4.5,8]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_FNormal_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Force (N)");
%     title('Force feedback of Right Gripper (20Hz disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([5,8]);
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
    ylim([18,22]);
    xlim([1,4]);
    hold on
    subplot(1,2,2)
    plot(t,gR_input_plt(:,i));
    xlabel("Time (s)");
    ylabel("Amplitude");
    title('Right Gripper Input Signal (20Hz disturbance at 30th iteration)')
    legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
    ylim([17,21]);
    xlim([1,4]);
    hold on

end


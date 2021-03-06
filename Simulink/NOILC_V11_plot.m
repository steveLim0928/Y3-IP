%% Save data
close all;


if (0)
    save('inputx.mat','inputx');
    save('inputy.mat','inputy');
    save('inputz.mat','inputz');
    save('inputgL.mat','inputgL');
    save('inputgR.mat','inputgR');
    save('tIn.mat','tIn');
    
    save('yx.mat','yx');
    save('yy.mat','yy');
    save('yz.mat','yz');
    
    save('ygL.mat','ygL');
    save('ygR.mat','ygR');
    
    save('LNormalF.mat','LNormalF');
    save('RNormalF.mat','RNormalF');
    
    save('refLg2.mat','refLg2');
    save('refRg2.mat','refRg2');

    save('refLg1.mat','refLg1');
    save('refRg1.mat','refRg1');
    
    save('u_gL_d.mat','u_gL_d');
    save('u_gR_d.mat','u_gR_d');
    
    save('u_gL.mat','u_gL');
    save('u_gR.mat','u_gR');

    save('x_pos_plt.mat','x_pos_plt');
    save('y_pos_plt.mat','y_pos_plt');
    save('z_pos_plt.mat','z_pos_plt');
    save('gripL_pos_plt.mat','gripL_pos_plt');
    save('gripR_pos_plt.mat','gripR_pos_plt');
    save('gripL_FNormal_plt.mat','gripL_FNormal_plt');
    save('gripR_FNormal_plt.mat','gripR_FNormal_plt');
    save('tracjecotryGL.mat','tracjecotryGL');
    save('tracjecotryGR.mat','tracjecotryGR');

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

if (0)
    load('inputx.mat');
    load('inputy.mat');
    load('inputz.mat');
    load('inputgL.mat');
    load('inputgR.mat');
    load('tIn.mat');
    
    load('yx.mat');
    load('yy.mat');
    load('yz.mat');
    
    load('ygL.mat');
    load('ygR.mat');
    
    load('LNormalF.mat');
    load('RNormalF.mat');
    
    load('refLg2.mat');
    load('refRg2.mat');

    load('refLg1.mat');
    load('refRg1.mat');
    
    load('u_gL_d.mat');
    load('u_gR_d.mat');
    
    load('u_gL.mat');
    load('u_gR.mat');

    load('x_pos_plt.mat');
    load('y_pos_plt.mat');
    load('z_pos_plt.mat');
    load('gripL_pos_plt.mat');
    load('gripR_pos_plt.mat');
    load('gripL_FNormal_plt.mat');
    load('gripR_FNormal_plt.mat');
    load('tracjecotryGL.mat');
    load('tracjecotryGR.mat');

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

%% Ideal case
% close all;

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

% plot(t,ygL(:,N));
% xlabel('Time (s)');
% ylabel('Displacement (m)');
% title("Left Gripper Position Feedback");



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

% N = 40;
% t = 0:0.0125:5;
% t(1) = []; % remove t = 0
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
% xlim([25,40]);
% title('Left gripper Position error (constant disturbance at 30th iteration)');
% 
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
% xlim([25,40]);
% title('Right gripper Position error (constant disturbance at 30th iteration)');
% 
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
% xlim([25,40]);
% title('Left gripper Force error (constant disturbance at 30th iteration)');
% legend({'0% Disturbance','5% Disturbance','10% Disturbance'},'Location','northeast','FontSize',7);
% 
% subplot(1,2,2)
% plot(enorm_FgR_0);
% hold on
% plot(enorm_FgR_0_0_5_mid);
% plot(enorm_FgR_0_1_mid);
% xlabel("Iteration (k)");
% ylabel("Error Norm");
% xticks([1:N]);
% xlim([25,40]);
% title('Right gripper Force error (constant disturbance at 30th iteration)');
% legend({'0% Disturbance','5% Disturbance','10% Disturbance'},'Location','northeast','FontSize',7);

%  for i=29:37
%     subplot(1,2,1)
%     plot(t,gripL_FNormal_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Force (N)");
%     title('Left Gripper Force feedback (10% disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration','36th Iteration','37th Iteration'},'Location','northeast','FontSize',7)
% 
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_FNormal_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Force (N)");
%     title('Right Gripper Force feedback (10% disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration','36th Iteration','37th Iteration'},'Location','northeast','FontSize',7)
%     hold on
% 
% 
%  end
% 
% for i=29:37
%     subplot(1,2,1)
%     plot(t,gripL_pos_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Force (N)");
%     title('Left Gripper Position feedback (10% disturbance at 30th iteration)')
%     xlim([1,4]);
%     ylim([0.011,0.013]);
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration','36th Iteration','37th Iteration'},'Location','northeast','FontSize',7)
% 
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_pos_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Force (N)");
%     title('Right Gripper Position feedback (10% disturbance at 30th iteration)')
%     xlim([1,4]);
%     ylim([0.011,0.013]);
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration','36th Iteration','37th Iteration'},'Location','northeast','FontSize',7)
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
% ylim([0.0110,0.0134])
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
%     ylim([5,8]);
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
%     ylim([5,8]);
% 
% end

% xlim([2,3])

for i=29:35
    subplot(1,2,1)
    plot(t,x_pos_plt(:,i));
    xlabel("Time (s)");
    ylabel("Displacement (m)");
    title('X axis Position Feedback (5% disturbance at 30th iteration)')
    legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
    xlim([2.3,2.9]);
    ylim([0.3,0.5]);
    hold on
    subplot(1,2,2)
    plot(t,x_input_plt(:,i));
    xlabel("Time (s)");
    ylabel("Displacement (m)");
    title('X axis Input Signal (5% disturbance at 30th iteration)')
    legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
    xlim([2.3,2.9]);
    hold on
    ylim([4,6.5]);

end



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
%     xlim([2,3]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_pos_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Displacement (m)");
%     title('Position feedback of Right Gripper (20Hz disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([0.0117,0.0121]);
%     xlim([2,3]);
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
%     xlim([2,3]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gripR_FNormal_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Force (N)");
%     title('Force feedback of Right Gripper (20Hz disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([3.5,8.5]);
%     xlim([2,3]);
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
%     xlim([2,3]);
%     hold on
%     subplot(1,2,2)
%     plot(t,gR_input_plt(:,i));
%     xlabel("Time (s)");
%     ylabel("Amplitude");
%     title('Right Gripper Input Signal (20Hz disturbance at 30th iteration)')
%     legend({'29th Iteration','30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration'},'Location','northeast','FontSize',7)
%     ylim([18.5,20.5]);
%     xlim([2,3]);
%     hold on
% 
% end



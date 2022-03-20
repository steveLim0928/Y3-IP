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
% legend({'Trial 30','Trail 35'},'Location','northeast','FontSize',7);
% hold on;
% subplot(2,1,2);
% plot(t,gripR_FNormal_plt(:,30));
% xlabel('Time (s)');
% ylabel('Force (s)');
% title("Right Gripper Force Feedback (30th iteration)");
% legend({'Trial 30','Trail 35'},'Location','northeast','FontSize',7);

% hold on;

% subplot(2,1,1);
% plot(t,gripL_FNormal_plt(:,35));
% xlabel('Time (s)');
% ylabel('Force (s)');
% title("Left Gripper Force Feedback (35th iteration)");
% legend({'Trial 30','Trail 35'},'Location','northeast','FontSize',7);
% subplot(2,1,2);
% plot(t,gripR_FNormal_plt(:,35));
% xlabel('Time (s)');
% ylabel('Force (s)');
% title("Right Gripper Force Feedback (35th iteration)");
% legend({'Trial 30','Trail 35'},'Location','northeast','FontSize',7);

%% Error from the beginning



%% Error comes midway, after 5 iterations

%%
if (1)
    save('inputx.mat','inputx');
    save('inputy.mat','inputy');
    save('inputz.mat','inputz');
    save('tIn.mat','tIn');
    
    save('LGripRef.mat','LGripRef');
    save('LGripRef.mat','RGripRef');
    
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
    
    save('u_gL_d.mat','u_gL_d');
    save('u_gR_d.mat','u_gR_d');
    
    save('u_gL.mat','u_gL');
    save('u_gR.mat','u_gR');

    save('gripL_FNormal_plt.mat','gripL_FNormal_plt');
    save('gripR_FNormal_plt.mat','gripR_FNormal_plt');

    save('gL_input_plt.mat','gL_input_plt');
    save('gR_input_plt.mat','gR_input_plt');
end

%%
if (0)
    inputx = timeseries(u_x,t);
    inputy = timeseries(u_y,t);
    inputz = timeseries(u_z,t);
    tIn = timeseries(t,t);
    LGripRef = timeseries(refRg2,t);
    RGripRef = timeseries(refRg2,t);
    %         end
    model = sim('Impedance_Gantry_Model_V8.slx')
    yx = model.outputx.Data();
    yx(1) = [];
    yy = model.outputy.Data();
    yy(1) = [];
    yz = model.outputz.Data();
    yz(1) = [];
    LFNormal = model.LFNormal.Data();
    LFNormal(1) = [];
    RFNormal = model.RFNormal.Data();
    RFNormal(1) = [];
    RFFriction = model.RFFriction.Data();
    RFFriction(1) = [];
    refL = model.LGPosAdj.Data();
    refL(1) = [];
    refR = model.RGPosAdj.Data();
    refR(1) = [];
    ygL = model.outputgL.Data();
    ygL(1) = [];
    ygR = model.outputgR.Data();
    ygR(1) = [];
    u_gL_d = model.u_gL_d.Data();
    u_gL_d(1) = [];
    u_gR_d = model.u_gR_d.Data();
    u_gR_d(1) = [];
    u_gL = model.u_gL.Data();
    u_gL(1) = [];
    u_gR = model.u_gR.Data();
    u_gR(1) = [];
    if (z_clear)
    enorm_FgL(i) = norm(1.2*Fmin - LFNormal);
    enorm_FgR(i) = norm(1.2*Fmin - RFNormal);
    end
    % Decide if z should move
    if (mean(yx(col_x1))>=A_x(1)-A_x(1)*0.03 && mean(yx(col_x2))>=A_x(2)-A_x(2)*0.03)
    if (mean(yx(col_y1))>=A_y(2)-A_y(2)*0.03 && mean(yx(col_y2))>=A_y(2)-A_y(2)*0.03)
    z_clear = 1;
    end
    end
    %     z_clear = 1;
    % Calculate tracking error
    e_x = refx - yx;
    e_y = refy - yy;
    e_z = -refz1 - yz;
    e_gL = refL - ygL;
    e_gR = refR - ygR;
    enorm_x(i) = norm(e_x);
    enorm_y(i) = norm(e_y);
    enorm_z(i) = norm(e_z);
    enorm_gL(i) = norm(e_gL);
    enorm_gR(i) = norm(e_gR);
    x_pos_plt = [x_pos_plt yx];
    y_pos_plt = [y_pos_plt yy];
    z_pos_plt = [z_pos_plt yz];
    gripL_pos_plt = [gripL_pos_plt ygL];
    gripR_pos_plt = [gripR_pos_plt ygR];
    gripL_FNormal_plt = [gripL_FNormal_plt LFNormal];
    gripR_FNormal_plt = [gripR_FNormal_plt RFNormal];
end

%%


if (0)
    subplot(1,2,1)
    plot(t,gripL_FNormal_plt(:,end));
    xlabel("Time (s)");
    ylabel("Force (N)");
    title('Left Gripper Force feedback (20Hz sine disturbance)')
    % legend({'30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration','36th Iteration','37th Iteration'},'Location','northeast','FontSize',7)
    subplot(1,2,2)
    plot(t,gripR_FNormal_plt(:,end));
    xlabel("Time (s)");
    ylabel("Force (N)");
    title('Right Gripper Force feedback (20Hz sine disturbance)')
    % legend({'30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration','36th Iteration','37th Iteration'},'Location','northeast','FontSize',7)
end

% subplot(1,2,1)
% plot(t,gL_input_plt(:,end));
% xlabel("Time (s)");
% ylabel("Force (N)");
% title('Left Gripper Position feedback (5% disturbance)')
% % legend({'30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration','36th Iteration','37th Iteration'},'Location','northeast','FontSize',7)
% subplot(1,2,2)
% plot(t,gR_input_plt(:,end));
% xlabel("Time (s)");
% ylabel("Force (N)");
% title('Right Gripper Position feedback (5% disturbance)')
% % legend({'30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration','36th Iteration','37th Iteration'},'Location','northeast','FontSize',7)


%% Impulse disturbance at 30th iteration

% close all;

% subplot(1,2,1)
% plot(t,gripL_FNormal_plt(:,end));
% xlabel("Time (s)");
% ylabel("Force (N)");
% title('Left Gripper Force feedback (10% disturbance)')
% % legend({'30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration','36th Iteration','37th Iteration'},'Location','northeast','FontSize',7)
% subplot(1,2,2)
% plot(t,gripR_FNormal_plt(:,end));
% xlabel("Time (s)");
% ylabel("Force (N)");
% title('Right Gripper Force feedback (10% disturbance)')
% % legend({'30th Iteration','31th Iteration','32th Iteration','33th Iteration','34th Iteration','35th Iteration','36th Iteration','37th Iteration'},'Location','northeast','FontSize',7)



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



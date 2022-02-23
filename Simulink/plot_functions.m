
% for i=1:N
% subplot(2,3,1)
% plot(t, x_pos_plt(:,i));
% title("x axis");
% xlabel("time(s)");
% ylabel("position(m)");
% hold on
% legend({'Trial 1','Trial 2','Trial 3','Trial 4','Trial 5','Trial 6','Trial 7','Trial 8','Trial 9','Trial 10'},'Location','northeast','FontSize',7)
% subplot(2,3,2)
% plot(t, y_pos_plt(:,i));
% title("y axis");
% xlabel("time(s)");
% ylabel("position(m)");
% hold on
% legend({'Trial 1','Trial 2','Trial 3','Trial 4','Trial 5','Trial 6','Trial 7','Trial 8','Trial 9','Trial 10'},'Location','northeast','FontSize',7)
% subplot(2,3,3)
% plot(t, z_pos_plt(:,i));
% title("z axis");
% xlabel("time(s)");
% ylabel("position(m)");
% hold on
% legend({'Trial 1','Trial 2','Trial 3','Trial 4','Trial 5','Trial 6','Trial 7','Trial 8','Trial 9','Trial 10'},'Location','northeast','FontSize',7)
% subplot(2,3,4)
% plot(t, grip1_pos_plt(:,i));
% title("gripper1 axis");
% xlabel("time(s)");
% ylabel("position(m)");
% hold on
% legend({'Trial 1','Trial 2','Trial 3','Trial 4','Trial 5','Trial 6','Trial 7','Trial 8','Trial 9','Trial 10'},'Location','northeast','FontSize',7)
% subplot(2,3,5)
% plot(t, grip2_pos_plt(:,i));
% title("gripper2 axis");
% xlabel("time(s)");
% ylabel("position(m)");
% hold on
% legend({'Trial 1','Trial 2','Trial 3','Trial 4','Trial 5','Trial 6','Trial 7','Trial 8','Trial 9','Trial 10'},'Location','northeast','FontSize',7)
% end

subplot(3,1,1)
plot(t, xinputforce);
title("x axis input and sense force");
xlabel("Time (s)");
ylabel("Force (N)");
hold on
scatter(t, xsenseforce);
legend({'input force','measured force'})
subplot(3,1,2)
plot(t, yinputforce);
title("y axis input and sense force");
xlabel("Time (s)");
ylabel("Force (N)");
hold on
scatter(t, ysenseforce);
legend({'input force','measured force'})
subplot(3,1,3)
scatter(t,u_x)
title("x axis input signal");
xlabel("Time (s)");
ylabel("Force (N)");
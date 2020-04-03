save_path = 'C:\Files\workspace\MATLAB_workspace\毕设版本存档\UAV-GBS-20191111\Simulation\sim_20191115_3\';
load([save_path,'Optimal_Rate.mat'],'Optimal_Rate')
load([save_path,'Trajectory.mat'],'q')
load ([save_path,'Schedule.mat'],'Schedule_result')
load ([save_path,'Speed.mat'],'speed_final')
load ([save_path,'Power.mat'],'Power')
load ([save_path,'Max-min Rate.mat'],'Optimal_value')
%%
%%绘制信息率
figure(1)
plot(min(Optimal_Rate,[],2),'-.*b','LineWidth',2);
grid on
set(gca,'FontSize',13)
title('Max-min Signal Rate','FontSize',16)
xlabel('Number of Iteration','FontSize',13)
ylabel({'Max-min Signal Rate','(bps/Hz)'},'FontSize',13) 
saveas(gcf,[save_path,'min Signal Rate.jpg'])
saveas(gcf,[save_path,'min Signal Rate.fig'])

figure(2)
total_Rate = sum(Optimal_Rate(:,1:6),2)
plot((total_Rate),'-.*b','LineWidth',2);
grid on
set(gca,'FontSize',13)
title('Total Signal Rate','FontSize',16)
xlabel('Number of Iteration','FontSize',13)
ylabel({'Total Signal Rate','(bps/Hz)'},'FontSize',13) 
saveas(gcf,[save_path,'total Signal Rate.jpg'])
saveas(gcf,[save_path,'total Signal Rate.fig'])
%%
%%绘制飞行轨迹
q = 100*q;
figure(3)
plot(q(1,:,1),q(2,:,1),'-o','color','b','LineWidth',2);
hold on
plot(q(1,:,2),q(2,:,2),'-s','color','r','LineWidth',2);
map()
hold off
axis([0 2000 0 2000])
grid on
set(gca,'FontSize',13)
title("UAV's optimal Trajectory",'FontSize',16)
xlabel('X/m','FontSize',13)
ylabel('Y/m','FontSize',13) 
saveas(gcf,[save_path,'Trajectory.jpg'])
saveas(gcf,[save_path,'Trajectory.fig'])
%%
%%带宽分配
figure(4)
h1 = heatmap(Schedule_result(:,:,1));
set(gca,'FontSize',13)
h1.Title = "Bandwidth Assignment of UAV-1";
h1.XLabel = 'Time Slot Index';
h1.YLabel = 'User Index';
saveas(gcf,[save_path,'Bandwidth Assignment of UAV-1.fig'])
saveas(gcf,[save_path,'Bandwidth Assignment of UAV-1.jpg'])
figure(5)
h2 = heatmap(Schedule_result(:,:,2));
set(gca,'FontSize',13)
h2.Title = "Bandwidth Assignment of UAV-2";
h2.XLabel = 'Time SlotIndex';
h2.YLabel = 'User Index';
saveas(gcf,[save_path,'Bandwidth Assignment of UAV-2.fig'])
saveas(gcf,[save_path,'Bandwidth Assignment of UAV-2.jpg'])
figure(6)
h3 = heatmap(Schedule_result(:,:,3));
set(gca,'FontSize',13)
h3.Title = "Bandwidth Assignment of GBS";
h3.XLabel = 'Time Slot Index';
h3.YLabel = 'User Index';
saveas(gcf,[save_path,'Bandwidth Assignment of GBS.fig'])
saveas(gcf,[save_path,'Bandwidth Assignment of GBS.jpg'])
%%
%%飞行速度
figure(7)
subplot(211)
plot(speed_final(:,1),'-.*b','LineWidth',2);
grid on
set(gca,'FontSize',13)
title('UAV-1 Speed','FontSize',16)
xlabel('Time Slot Index','FontSize',13)
ylabel('UAV-1 Speed','FontSize',13)
subplot(212)
plot(speed_final(:,2),'-.*r','LineWidth',2);
set(gca,'FontSize',13)
grid on
title('UAV-2 Speed','FontSize',16)
xlabel('Time Slot Index','FontSize',13)
ylabel('UAV-2 Speed','FontSize',13)
saveas(gcf,[save_path,'UAV Speed.fig'])
saveas(gcf,[save_path,'UAV Speed.jpg'])
%%
% %%发射功率
% figure(8)
% subplot(2,1,1)
% plot(Power(1,:),'-.*b','LineWidth',2);
% set(gca,'FontSize',13)
% xlabel('Time Slot Index','FontSize',13);ylabel('Transmit Power/mW','FontSize',13)
% title('UAV-1 Transmit Power','FontSize',16)
% grid on
% subplot(2,1,2)
% plot(Power(2,:),'-.*b','LineWidth',2);
% set(gca,'FontSize',13)
% xlabel('Time Slot Index','FontSize',13);ylabel('Transmit Power/mW','FontSize',13)
% title('UAV-2 Transmit Power','FontSize',16)
% grid on
% saveas(gcf,[save_path,'UAV Transmit Power.fig'])
% saveas(gcf,[save_path,'UAV Transmit Power.jpg'])
%%
figure(9)
Index = find(Optimal_value ~= 1);
plot(Optimal_value(Index),'-.*b','LineWidth',2);
grid on
set(gca,'FontSize',13)
title('Optimal Value','FontSize',16)
xlabel('Number of Iteration','FontSize',13)
ylabel('Optimal Value','FontSize',13) 
saveas(gcf,[save_path,'Optimal_value.jpg'])
saveas(gcf,[save_path,'Optimal_value.fig'])
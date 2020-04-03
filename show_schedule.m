function [] = show_schedule()
%show_schedule 用于绘制用户调度热力图，便于直观展示基站与用户在各个时隙的连接状态。
load('Data_const.mat','save_path');              
load Schedule.mat Schedule_result
figure(6)
h1 = heatmap(Schedule_result(:,:,1));
set(gca,'FontSize',13)
h1.Title = "Bandwidth Assignment of UAV-1";
h1.XLabel = 'Time Slot';
h1.YLabel = 'User Index';
saveas(gcf,[save_path,'Bandwidth Assignment of UAV-1.fig'])
saveas(gcf,[save_path,'Bandwidth Assignment of UAV-1.jpg'])
figure(7)
h2 = heatmap(Schedule_result(:,:,2));
set(gca,'FontSize',13)
h2.Title = "Bandwidth Assignment of UAV-2";
h2.XLabel = 'Time Slot';
h2.YLabel = 'User Index';
saveas(gcf,[save_path,'Bandwidth Assignment of UAV-2.fig'])
saveas(gcf,[save_path,'Bandwidth Assignment of UAV-2.jpg'])
figure(8)
h3 = heatmap(Schedule_result(:,:,3));
set(gca,'FontSize',13)
h3.Title = "Bandwidth Assignment of GBS";
h3.XLabel = 'Time Slot';
h3.YLabel = 'User Index';
saveas(gcf,[save_path,'Bandwidth Assignment of GBS.fig'])
saveas(gcf,[save_path,'Bandwidth Assignment of GBS.jpg'])
end


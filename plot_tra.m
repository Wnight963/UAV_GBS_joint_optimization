function [] = plot_tra(Trajectory_result)
%plot_tra �˺����û��������˻��Ĺ켣ͼ
%��������ʹ��plot�����е�MarkerIndices������������ʾ��marker��������ʾָ�������ݵ�
%   �˴���ʾ��ϸ˵��
load('Data_const.mat','NumOfTimeSlot','Height_UAV','width','save_path');
Height_UAV = Height_UAV*10;
figure(2)
plot3(Trajectory_result(1,:,1),Trajectory_result(2,:,1),Height_UAV*ones(1,NumOfTimeSlot),'-o','color','b','LineWidth',2); %����UAV-1�켣
hold on
plot3(Trajectory_result(1,:,2),Trajectory_result(2,:,2),Height_UAV*ones(1,NumOfTimeSlot),'-s','color','r','LineWidth',2); %����UAV-2�켣
axis([0 width 0 width 0 500])
set(gca,'FontSize',13)
title("UAVs' Optimal Trajectory,",'FontSize',16)
xlabel('X/m');ylabel('Y/m')
map()
legend({'UAV-1','UAV-2','GBS','User'},'Location','southeast')
grid on
saveas(gcf,[save_path,'Trajectory.fig'])
saveas(gcf,[save_path,'Trajectory.jpg'])
hold off
end


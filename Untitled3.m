%%%本文件为无人机-基站优化主程序
clear 
clc
Initiallization = 1;   %为0时读取上次寻优结果，为1时重新进行寻优。
NumOfUAV = 2;  %无人机的数目
NumOfNode =7; %终端节点的数目
width = 2000;  %地图的边长
MaxSpeed = 4;  %无人机最大速度 m/s
MinSpeed = 1;   %无人机最小速度 m/s
MinDist = 8;  %无人机之间最小安全距离 m
Height_UAV = 30;  %无人机飞行高度 m
MaxPower = 100;  %无人机最大发射功率 mW
MaxEnergyConsume = 4800; %无人机在T内的最大能耗
Power_GBS = 300; %地面基站发射功率 mW
T =60;  %时间间隔，需要调整其大小 ，比较不同情况下的结果
epsilon = 0.15; %认为无人机在一个时隙内为静止的的阈值
NumOfTimeSlot = ceil(MaxSpeed*T/Height_UAV/epsilon);
SlotInterval = T/NumOfTimeSlot;  %时隙长度 单位s
Gain_UAV = 10^-6;  %单位距离无人机信道增益  -60dB
Gain_GBS = 10^-4; %地面基站信道增益 -40dB
LoseExp = 3;   %地面基站损耗指数
% noise_dBm = -167;  %噪声功率谱密度，dBm/Hz
% noise_mW = 10^(noise_dBm/10);  %噪声功率谱密度 mW/Hz
Bandwidth=10*10^6;
% noise_mW = noise_mW*Bandwidth;
noise_mW = 1e-11; %可以直接设置噪声的功率

BlockPara = ones(1,NumOfNode);   %遮挡因子 有遮挡时为0.01，无遮挡时为1,需要时将对应的用户设为0.01即可。
BlockPara(5) = 0.01;
BlockPara(4) = 0.01;
BlockPara_UAV = ones(NumOfNode,NumOfUAV);   %无人机静止时的遮挡因子 有遮挡时为0.01，无遮挡时为1,需要时将对应的用户设为0.01即可。
% BlockPara_UAV(1,1) = 0.1;
% BlockPara_UAV(3,1) = 0.1;
% BlockPara_UAV(5,1) = 0.1;
% BlockPara_UAV(1,2) = 0.1;
% BlockPara_UAV(2,2) = 0.1;
% BlockPara_UAV(6,2) = 0.1;
% BlockPara(5) = 0.1;
threshold_1 = 1e-6;  %迭代阈值
threshold_2 = 1e-5;  %迭代阈值
threshold_3 = 1e-7;  %迭代阈值

save_path = 'C:\Files\workspace\MATLAB_workspace\毕设版本存档\UAV-GBS-20200108\Simulation\sim_20200108_5\';
% save_path(2) = 'C:\Files\workspace\MATLAB_workspace\毕设版本存档\UAV-GBS-20191110\SImulation\sim_20191111_3\';
save('Data_const','NumOfUAV','NumOfNode','NumOfTimeSlot','MaxSpeed','MinSpeed','MinDist','Height_UAV','Power_GBS',...
                  'MaxPower','MaxEnergyConsume','T','SlotInterval','Gain_UAV','Gain_GBS','LoseExp','BlockPara','BlockPara_UAV','noise_mW','width','Bandwidth','Initiallization','save_path');
save([save_path,'Data_const.mat'],'NumOfUAV','NumOfNode','NumOfTimeSlot','MaxSpeed','MinSpeed','MinDist','Height_UAV','Power_GBS',...
                  'MaxPower','MaxEnergyConsume','T','SlotInterval','Gain_UAV','Gain_GBS','LoseExp','BlockPara','BlockPara_UAV','noise_mW','width','Bandwidth','Initiallization','save_path');
%%
if  Initiallization== 1  %判断是否读取上次存储的数据
    % initial_map_v6;  %无人机固定位置地图
    circle_tra();      %无人机圆形初始路径
    % initial_map_v5;  %无人机直线初始路径
    % load generate_schedule.mat Schedule_result
    % load schedule.mat Schedule_result
    Schedule_result = zeros(NumOfNode,NumOfTimeSlot,NumOfUAV+1);
    Power = ones(NumOfUAV,NumOfTimeSlot)*MaxEnergyConsume/T;
    save Power.mat Power;
    save Schedule.mat Schedule_result;
end

Object_value = 0;  
save Object_value.mat Object_value;
Optimal_Rate = [];   %用于存储真实的信息率
save Optimal_Rate.mat Optimal_Rate;
%%
iteration = 0;
count = 0;    
count_1 = 1; %用于存储迭代过程中无人机飞行距离时计数
travel_dist = zeros(2,100); %预分配内存，用于存储迭代过程中无人机飞行距离
Optimal_value = ones(250,1);   %用于存储目标函数最优值，不一定是真实信息率
Iter_control_1 = 1; %控制迭代运行
Iter_control_2 = 1;
Iter_control_3 = 1;
former_1 = 1; %上一次迭代的目标函数值
latter_1 = 1; %当次迭代的目标函数值
former_2 = 1; %上一次迭代的目标函数值 
latter_2 = 1; %当次迭代的目标函数值
former_3 = 1; %上一次迭代的目标函数值
latter_3 = 1; %当次迭代的目标函数值


%%
% while Iter_control_1 > threshold_1
while iteration < 100
   iteration = iteration+1  %迭代参数加1，取消分号便于监测迭代状态。
   %用户调度优化
   while Iter_control_3 > threshold_3
   disp('用户调度优化')
   [Schedule_result,S_value] = Schedule_optim();
   show_schedule();   %%绘制用户调度热力图
   latter_3 = S_value;
   Iter_control_3 = (latter_3 - former_3)/former_3
   former_3 = latter_3;
   end
   count = count + 1;
   Optimal_value(count) = S_value;
   calcu_rate();
      
   former_3 = 1; %上一次迭代的目标函数值
   latter_3 = 1; %当次迭代的目标函数值
   Iter_control_3 = 1;
   %%
   %无人机路径优化
   while Iter_control_2 > threshold_2
   disp('路径优化')
   [Trajectory_result,T_value] = Trajectory_optim();
   Trajectory_result = 10*Trajectory_result;%恢复原来尺寸
   plot_tra(Trajectory_result); %绘制轨迹
   
   travel_dist(:,count_1) = travel_dis();
   count_1 = count_1+1;
   
   latter_2 = T_value;
   Iter_control_2 = (latter_2 - former_2)/former_2
   former_2 = latter_2;
   end
   count = count + 1;
   Optimal_value(count) = T_value;
   calcu_rate();
   %循环结束后需要重置
%    threshold_2 = threshold_2/2;
   former_2 = 1; %上一次迭代的目标函数值
   latter_2 = 1; %当次迭代的目标函数值
   Iter_control_2 = 1;
   
%   %%
%    %无人机发射功率优化
%     disp('发射功率优化')
%    for PowerIter = 1:5
%    [Power,P_value] = Power_optim();
%    end
%    count = count + 1;
%    Optimal_value(count) = P_value;
%    calcu_rate();
   %%
   %%计算目标函数的增长率，控制外层迭代的运行
   latter_1 = T_value; 
   Iter_control_1 = (latter_1 - former_1)/former_1
   former_1 = latter_1;
   
%    %%
%    %绘制发射功率变化图
%    figure(3)
%    subplot(2,1,1)
%    plot(Power(1,:),'-.*b','LineWidth',2);
%    set(gca,'FontSize',13)
%    xlabel('Time Slot');ylabel('Transmit Power/mW')
%    title('UAV-1 Transmit Power')
%    grid on
%    subplot(2,1,2)
%    plot(Power(2,:),'-.*b','LineWidth',2);
%    set(gca,'FontSize',13)
%    xlabel('Time Slot');ylabel('Transmit Power/mW')
%    title('UAV-2 Transmit Power','FontSize',16)
%    grid on
%    saveas(gcf,[save_path,'UAV Transmit Power.fig'])
%    saveas(gcf,[save_path,'UAV Transmit Power.jpg'])

%%
%绘制速度变化图 
 speed_final = zeros(NumOfTimeSlot,NumOfUAV);  
 for k=1:NumOfUAV
    for n=1:NumOfTimeSlot-1
    speed_final(n,k)=sqrt((Trajectory_result(1,n+1,k)-Trajectory_result(1,n,k))^2+(Trajectory_result(2,n+1,k)-Trajectory_result(2,n,k))^2)/SlotInterval;
    end
 end
 figure(4)
 subplot(211)
 plot(speed_final(:,1),'-.*b','LineWidth',2);
 set(gca,'FontSize',13)
 grid on
 title('UAV-1 Speed','FontSize',16)
 xlabel('Time Slot')
 ylabel('UAV-1 Speed')
 subplot(212)
 plot(speed_final(:,2),'-.*r','LineWidth',2);
 set(gca,'FontSize',13)
 grid on
 title('UAV-2 Speed','FontSize',16)
 xlabel('Time Slot')
 ylabel('UAV-2 Speed')
 save ([save_path,'Speed.mat'], 'speed_final');
 saveas(gcf,[save_path,'UAV Speed.fig'])
  saveas(gcf,[save_path,'UAV Speed.jpg'])

end
%%
%迭代结束后打印出目标函数的值。
%如果中间结束运行，可以在复制下面的代码到命令行中，进行手动打印
figure(5)
plot(Optimal_value(1:count),'-.*b','LineWidth',2);
set(gca,'FontSize',13)
title('Optimal Value','FontSize',16)
xlabel('Number of Iteration')
ylabel('Optimal Value Rate') 
save([save_path,'Optimal Value.mat'], 'Optimal_value')
saveas(gcf,[save_path,'Optimal Value.fig'])
saveas(gcf,[save_path,'Optimal Value.jpg'])
grid on

figure(9)%绘制迭代过程中无人机飞行距离的变化情况
for k=1:NumOfUAV
    plot(1:count_1-1,travel_dist(k,1:count_1-1),'Marker','o','LineWidth',2);
    hold on
end
set(gca,'FontSize',13)
grid on
title('Travel distance against iteration','FontSize',16)
xlabel('Number of Iteration')
ylabel('Travel distance') 
save([save_path,'Travel distance.mat'], 'travel_dist')
saveas(gcf,[save_path,'Travel distance.fig'])
saveas(gcf,[save_path,'Travel distance.jpg'])







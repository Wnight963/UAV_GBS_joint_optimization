%%%���ļ�Ϊ���˻�-��վ�Ż�������
clear 
clc
Initiallization = 1;   %Ϊ0ʱ��ȡ�ϴ�Ѱ�Ž����Ϊ1ʱ���½���Ѱ�š�
NumOfUAV = 2;  %���˻�����Ŀ
NumOfNode =7; %�ն˽ڵ����Ŀ
width = 2000;  %��ͼ�ı߳�
MaxSpeed = 4;  %���˻�����ٶ� m/s
MinSpeed = 1;   %���˻���С�ٶ� m/s
MinDist = 8;  %���˻�֮����С��ȫ���� m
Height_UAV = 30;  %���˻����и߶� m
MaxPower = 100;  %���˻�����书�� mW
MaxEnergyConsume = 4800; %���˻���T�ڵ�����ܺ�
Power_GBS = 300; %�����վ���书�� mW
T =60;  %ʱ��������Ҫ�������С ���Ƚϲ�ͬ����µĽ��
epsilon = 0.15; %��Ϊ���˻���һ��ʱ϶��Ϊ��ֹ�ĵ���ֵ
NumOfTimeSlot = ceil(MaxSpeed*T/Height_UAV/epsilon);
SlotInterval = T/NumOfTimeSlot;  %ʱ϶���� ��λs
Gain_UAV = 10^-6;  %��λ�������˻��ŵ�����  -60dB
Gain_GBS = 10^-4; %�����վ�ŵ����� -40dB
LoseExp = 3;   %�����վ���ָ��
% noise_dBm = -167;  %�����������ܶȣ�dBm/Hz
% noise_mW = 10^(noise_dBm/10);  %�����������ܶ� mW/Hz
Bandwidth=10*10^6;
% noise_mW = noise_mW*Bandwidth;
noise_mW = 1e-11; %����ֱ�����������Ĺ���

BlockPara = ones(1,NumOfNode);   %�ڵ����� ���ڵ�ʱΪ0.01�����ڵ�ʱΪ1,��Ҫʱ����Ӧ���û���Ϊ0.01���ɡ�
BlockPara(5) = 0.01;
BlockPara(4) = 0.01;
BlockPara_UAV = ones(NumOfNode,NumOfUAV);   %���˻���ֹʱ���ڵ����� ���ڵ�ʱΪ0.01�����ڵ�ʱΪ1,��Ҫʱ����Ӧ���û���Ϊ0.01���ɡ�
% BlockPara_UAV(1,1) = 0.1;
% BlockPara_UAV(3,1) = 0.1;
% BlockPara_UAV(5,1) = 0.1;
% BlockPara_UAV(1,2) = 0.1;
% BlockPara_UAV(2,2) = 0.1;
% BlockPara_UAV(6,2) = 0.1;
% BlockPara(5) = 0.1;
threshold_1 = 1e-6;  %������ֵ
threshold_2 = 1e-5;  %������ֵ
threshold_3 = 1e-7;  %������ֵ

save_path = 'C:\Files\workspace\MATLAB_workspace\����汾�浵\UAV-GBS-20200108\Simulation\sim_20200108_5\';
% save_path(2) = 'C:\Files\workspace\MATLAB_workspace\����汾�浵\UAV-GBS-20191110\SImulation\sim_20191111_3\';
save('Data_const','NumOfUAV','NumOfNode','NumOfTimeSlot','MaxSpeed','MinSpeed','MinDist','Height_UAV','Power_GBS',...
                  'MaxPower','MaxEnergyConsume','T','SlotInterval','Gain_UAV','Gain_GBS','LoseExp','BlockPara','BlockPara_UAV','noise_mW','width','Bandwidth','Initiallization','save_path');
save([save_path,'Data_const.mat'],'NumOfUAV','NumOfNode','NumOfTimeSlot','MaxSpeed','MinSpeed','MinDist','Height_UAV','Power_GBS',...
                  'MaxPower','MaxEnergyConsume','T','SlotInterval','Gain_UAV','Gain_GBS','LoseExp','BlockPara','BlockPara_UAV','noise_mW','width','Bandwidth','Initiallization','save_path');
%%
if  Initiallization== 1  %�ж��Ƿ��ȡ�ϴδ洢������
    % initial_map_v6;  %���˻��̶�λ�õ�ͼ
    circle_tra();      %���˻�Բ�γ�ʼ·��
    % initial_map_v5;  %���˻�ֱ�߳�ʼ·��
    % load generate_schedule.mat Schedule_result
    % load schedule.mat Schedule_result
    Schedule_result = zeros(NumOfNode,NumOfTimeSlot,NumOfUAV+1);
    Power = ones(NumOfUAV,NumOfTimeSlot)*MaxEnergyConsume/T;
    save Power.mat Power;
    save Schedule.mat Schedule_result;
end

Object_value = 0;  
save Object_value.mat Object_value;
Optimal_Rate = [];   %���ڴ洢��ʵ����Ϣ��
save Optimal_Rate.mat Optimal_Rate;
%%
iteration = 0;
count = 0;    
count_1 = 1; %���ڴ洢�������������˻����о���ʱ����
travel_dist = zeros(2,100); %Ԥ�����ڴ棬���ڴ洢�������������˻����о���
Optimal_value = ones(250,1);   %���ڴ洢Ŀ�꺯������ֵ����һ������ʵ��Ϣ��
Iter_control_1 = 1; %���Ƶ�������
Iter_control_2 = 1;
Iter_control_3 = 1;
former_1 = 1; %��һ�ε�����Ŀ�꺯��ֵ
latter_1 = 1; %���ε�����Ŀ�꺯��ֵ
former_2 = 1; %��һ�ε�����Ŀ�꺯��ֵ 
latter_2 = 1; %���ε�����Ŀ�꺯��ֵ
former_3 = 1; %��һ�ε�����Ŀ�꺯��ֵ
latter_3 = 1; %���ε�����Ŀ�꺯��ֵ


%%
% while Iter_control_1 > threshold_1
while iteration < 100
   iteration = iteration+1  %����������1��ȡ���ֺű��ڼ�����״̬��
   %�û������Ż�
   while Iter_control_3 > threshold_3
   disp('�û������Ż�')
   [Schedule_result,S_value] = Schedule_optim();
   show_schedule();   %%�����û���������ͼ
   latter_3 = S_value;
   Iter_control_3 = (latter_3 - former_3)/former_3
   former_3 = latter_3;
   end
   count = count + 1;
   Optimal_value(count) = S_value;
   calcu_rate();
      
   former_3 = 1; %��һ�ε�����Ŀ�꺯��ֵ
   latter_3 = 1; %���ε�����Ŀ�꺯��ֵ
   Iter_control_3 = 1;
   %%
   %���˻�·���Ż�
   while Iter_control_2 > threshold_2
   disp('·���Ż�')
   [Trajectory_result,T_value] = Trajectory_optim();
   Trajectory_result = 10*Trajectory_result;%�ָ�ԭ���ߴ�
   plot_tra(Trajectory_result); %���ƹ켣
   
   travel_dist(:,count_1) = travel_dis();
   count_1 = count_1+1;
   
   latter_2 = T_value;
   Iter_control_2 = (latter_2 - former_2)/former_2
   former_2 = latter_2;
   end
   count = count + 1;
   Optimal_value(count) = T_value;
   calcu_rate();
   %ѭ����������Ҫ����
%    threshold_2 = threshold_2/2;
   former_2 = 1; %��һ�ε�����Ŀ�꺯��ֵ
   latter_2 = 1; %���ε�����Ŀ�꺯��ֵ
   Iter_control_2 = 1;
   
%   %%
%    %���˻����书���Ż�
%     disp('���书���Ż�')
%    for PowerIter = 1:5
%    [Power,P_value] = Power_optim();
%    end
%    count = count + 1;
%    Optimal_value(count) = P_value;
%    calcu_rate();
   %%
   %%����Ŀ�꺯���������ʣ�����������������
   latter_1 = T_value; 
   Iter_control_1 = (latter_1 - former_1)/former_1
   former_1 = latter_1;
   
%    %%
%    %���Ʒ��书�ʱ仯ͼ
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
%�����ٶȱ仯ͼ 
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
%�����������ӡ��Ŀ�꺯����ֵ��
%����м�������У������ڸ�������Ĵ��뵽�������У������ֶ���ӡ
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

figure(9)%���Ƶ������������˻����о���ı仯���
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







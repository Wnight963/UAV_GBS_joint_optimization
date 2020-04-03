function [] = calcu_rate()
%此函数用来计算仿真结束后各个用户的平均信息率，可以仿真结果中的目标函数进行对比，验证结果的正确与否
%注意需要修改导入mat文件的地址
load('Data_const.mat')
load('map.mat')
% load('C:\Files\workspace\MATLAB_workspace\毕设版本存档\UAV-GBS-20191110\SImulation\sim_20191110_2\Power.mat', 'Power')
% load('C:\Files\workspace\MATLAB_workspace\毕设版本存档\UAV-GBS-20191110\SImulation\sim_20191110_2\Schedule.mat', 'Schedule_result')
% load('C:\Files\workspace\MATLAB_workspace\毕设版本存档\UAV-GBS-20191110\SImulation\sim_20191110_2\Trajectory.mat', 'q')
load('Schedule.mat', 'Schedule_result')
load('Power.mat', 'Power')
load('Trajectory.mat', 'q')
load Optimal_Rate.mat Optimal_Rate;
biterr
for m= 1:NumOfNode
    for k = 1:NumOfUAV
        UAVtoMT(m,:,k) = 100*((Height_UAV*ones(1,NumOfTimeSlot)).^2+(q(1,:,k)-MTlocation(1,:,m)).^2+...
            (q(2,:,k)-MTlocation(2,:,m)).^2);
        SNR(m,:,k) = (Power(k,:)*Gain_UAV/noise_mW)./UAVtoMT(m,:,k);
    end
    GBStoMT(m) = 10*sqrt((GBSlocation(1,1)-MTlocation(1,1,m))^2+(GBSlocation(2,1)-MTlocation(2,1,m))^2);
    SNR(m,:,NumOfUAV+1) = Power_GBS*Gain_GBS/noise_mW/GBStoMT(m)^LoseExp;
    
    for k = 1:NumOfUAV
     R(m,:,k) = Bandwidth*BlockPara_UAV(m,k)*Schedule_result(m,:,k).*log2(1+SNR(m,:,k));
    end
     R(m,:,NumOfUAV+1) = Bandwidth*Schedule_result(m,:,NumOfUAV+1).*log2(1+SNR(m,:,end))*BlockPara(m);

    total=sum(sum(R(m,:,:)));

    average(m) = total/NumOfTimeSlot;
end
Optimal_min_Rate = min(average);
Optimal_Rate = [Optimal_Rate;[average,Optimal_min_Rate]]
size(Optimal_Rate)
save([save_path,'SNR.mat'],'SNR')
save Optimal_Rate.mat Optimal_Rate;
save([save_path,'Optimal_Rate.mat'],'Optimal_Rate')

figure(10)
plot(min(Optimal_Rate,[],2),'-.*b','LineWidth',2);
grid on
set(gca,'FontSize',13)
title('Max-min Signal Rate(Episode)','FontSize',16)
xlabel('Number of Iteration','FontSize',13)
ylabel({'Max-min Signal Rate','(bps/Hz)'},'FontSize',13) 
saveas(gcf,[save_path,'min Signal Rate(Episode).jpg'])
saveas(gcf,[save_path,'min Signal Rate(Episode).fig'])

figure(11)
plot(Optimal_Rate(2:2:end,end),'-.*b','LineWidth',2);
grid on 
set(gca,'FontSize',13)
title('Max-min Signal Rate(Iteration)','FontSize',16)
xlabel('Number of Iteration','FontSize',13)
ylabel({'Max-min Signal Rate','(bps/Hz)'},'FontSize',13) 
saveas(gcf,[save_path,'min Signal Rate(Iteration).jpg'])
saveas(gcf,[save_path,'min Signal Rate(Iteration).fig'])

figure(12)
total_Rate = sum(Optimal_Rate(:,1:NumOfNode),2)
plot((total_Rate),'-.*b','LineWidth',2);
grid on
set(gca,'FontSize',13)
title('Total Signal Rate','FontSize',16)
xlabel('Number of Iteration','FontSize',13)
ylabel({'Total Signal Rate','(bps/Hz)'},'FontSize',13) 
saveas(gcf,[save_path,'total Signal Rate.jpg'])
saveas(gcf,[save_path,'total Signal Rate.fig'])
end
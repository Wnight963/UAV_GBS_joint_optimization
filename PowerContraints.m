function [c,ceq] = PowerContraints(Variable_P_initial)
%PowerContraints 此函数为发射功率优化子问题的约束条件
%   此处显示详细说明
load('Data_const.mat','NumOfUAV','NumOfNode','NumOfTimeSlot','MaxSpeed','MinDist','Height_UAV','Power_GBS',...
                 'MaxPower','MaxEnergyConsume','T', 'SlotInterval','Gain_UAV','Gain_GBS','LoseExp','BlockPara','BlockPara_UAV','noise_mW','Bandwidth');
load map.mat MTlocation GBSlocation;
load Trajectory.mat q;
load Schedule.mat Schedule_result;
%%
%%计算距离这一部分可以单独写成一个函数
%定义一个三维数组，N*M*K，无人机到终端的距离的平方。预分配内存，提高运算速度。
UAVtoMT = zeros(NumOfNode,NumOfTimeSlot,NumOfUAV);
%定义一个二维数组，N*M，地面基站到终端的距离。预分配内存，提高运算速度。
GBStoMT = zeros(NumOfNode,NumOfTimeSlot);
%%计算无人机到终端的距离的平方
for k = 1:NumOfUAV
    for m = 1:NumOfNode 
        UAVtoMT(m,:,k) = 10000*((Height_UAV*ones(1,NumOfTimeSlot)).^2+(q(1,:,k)-MTlocation(1,:,m)).^2+...
            (q(2,:,k)-MTlocation(2,:,m)).^2);
    end
end
for m = 1:NumOfNode
  GBStoMT(m,:) = 100*sqrt((GBSlocation(1,:)-MTlocation(1,:,m)).^2+(GBSlocation(2,:)-MTlocation(2,:,m)).^2);
end
%%
P_value = Variable_P_initial(end); %恢复引入的辅助变量
Variable_P = reshape(Variable_P_initial(1:end-1),[NumOfUAV,NumOfTimeSlot]);
R = zeros(NumOfNode,NumOfTimeSlot,NumOfUAV+1);  %预分配内存
%%计算信息率
for m = 1 : NumOfNode
  for k = 1 : NumOfUAV
      %%计算无人机k与用户m通信的信息率
      R(m,:,k) = Bandwidth*BlockPara_UAV(m,k)*Schedule_result(m,:,k).*log2(ones(1,NumOfTimeSlot)+(Variable_P(k,:)*Gain_UAV/noise_mW)./UAVtoMT(m,:,k));

  end
     %%计算地面基站与用户m通信的信息率
    R(m,:,NumOfUAV+1) = Bandwidth*Schedule_result(m,:,NumOfUAV+1).*(log2(1+Power_GBS*Gain_GBS/noise_mW/GBStoMT(m)^LoseExp)*BlockPara(m));  
end

%每个用户m的平均信息率的约束
i = 1;
%c = zeros(NumOfTimeSlot*(NumOfUAV+1)*(NumOfNode+1)+(NumOfTimeSlot+1)*NumOfNode);
for m = 1 : NumOfNode 
    sumR_m = sum(sum(R(m,:,:)));
    c(i)=P_value-1/NumOfTimeSlot*sumR_m;
    i = i+1;
end

%   sumR = sum(sum(sum(R)));
%    c(i)=P_value-1/NumOfTimeSlot*sumR;
%     i = i+1; 
   
%总能耗的约束
for k= 1:NumOfUAV
    c(i)=sum(Variable_P(k,:)*SlotInterval)-MaxEnergyConsume;
    i = i+1;
end

ceq = [];


end


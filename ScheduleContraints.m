function [c,ceq] = ScheduleContraints(Variable_S_initial)
%ScheduleContraints 此函数为用户调度优化子问题的约束条件
%   此处显示详细说明

load('Data_const.mat','NumOfUAV','NumOfNode','NumOfTimeSlot','Height_UAV','Power_GBS',...
                 'Gain_UAV','Gain_GBS','LoseExp','BlockPara','BlockPara_UAV','noise_mW','Bandwidth');
load map.mat MTlocation GBSlocation;
load Trajectory.mat q;
load Power.mat Power;
%%
%定义一个三维数组，N*M*K，无人机到终端的距离的平方。预分配内存，提高运算速度。
UAVtoMT = zeros(NumOfNode,NumOfTimeSlot,NumOfUAV);
%定义一个二维数组，N*M，地面基站到终端的距离。预分配内存，提高运算速度。
% GBStoMT = zeros(NumOfNode);
GBStoMT = zeros(NumOfNode,NumOfTimeSlot);
%%计算无人机到终端的距离的平方
for k = 1:NumOfUAV
    for m = 1:NumOfNode 
        UAVtoMT(m,:,k) = 100*((Height_UAV*ones(1,NumOfTimeSlot)).^2+(q(1,:,k)-MTlocation(1,:,m)).^2+...
            (q(2,:,k)-MTlocation(2,:,m)).^2);
    end
end
%%计算地面基站到终端的距离
for m = 1:NumOfNode
    GBStoMT(m,:) = 10*sqrt((GBSlocation(1,:)-MTlocation(1,:,m)).^2+(GBSlocation(2,:)-MTlocation(2,:,m)).^2);
end
%%
S_value = Variable_S_initial(end);  %恢复目标函数
Variable_S_initial = reshape(Variable_S_initial(1:end-1),[NumOfNode,NumOfTimeSlot,NumOfUAV+1]);  %恢复用户调度变量
R = zeros(NumOfNode,NumOfTimeSlot,NumOfUAV+1);  %预分配内存
%%计算无人机k与用户m通信的信息率
for m = 1 : NumOfNode
  for k = 1 : NumOfUAV
      %%需要注意;冒号所处的维度需要保持一致，并且进行加减运算时必须保证矩阵的大小一致
      R(m,:,k) = Bandwidth*BlockPara_UAV(m,k)*Variable_S_initial(m,:,k).*log2(1+(Power(k,:)*Gain_UAV/noise_mW)./UAVtoMT(m,:,k));
  end
     %%计算地面基站与用户m通信的信息率
     R(m,:,NumOfUAV+1) = Bandwidth*Variable_S_initial(m,:,NumOfUAV+1).*(log2(1+Power_GBS*Gain_GBS/noise_mW./GBStoMT(m,:).^LoseExp)*BlockPara(m));  
end
%%
%每个用户m的平均信息率的约束
i = 1;
for m = 1 : NumOfNode
    sumR_m = sum(sum(R(m,:,:)));
    c(i)=S_value-1/NumOfTimeSlot*sumR_m;
    i = i+1;
end
%     sumR = sum(sum(sum(R)));
%     c(i)=S_value-1/NumOfTimeSlot*sumR;
%     i = i+1;

% 
%在任意时隙，每个用户m只能与一个无人机或者地面基站通信
for m = 1 : NumOfNode
    for n = 1:NumOfTimeSlot
        c(i)=sum(Variable_S_initial(m,n,:))-1; 
        i=i+1;
    end 
end

%在任意时隙，每个无人机或者基站只能与一个用户通信
for k = 1 : NumOfUAV+1
    for n = 1:NumOfTimeSlot
        c(i)=sum(Variable_S_initial(:,n,k))-1; 
        i=i+1;
    end
end

%二进制变量约束，数量太多，考虑将其作为惩罚因子放到目标函数中。
% j=1;
% for k = 1:NumOfUAV+1
%    for m = 1 : NumOfNode
%        for n = 1:NumOfTimeSlot
% %            c(i)=Variable_S_initial(m,n,k)-Variable_S_initial(m,n,k)^2;
% %            i = i+1;             
%            ceq(j)=Variable_S_initial(m,n,k)-Variable_S_initial(m,n,k)^2;
%           j=j+1;
%        end
%    end
% end


ceq=[];
end


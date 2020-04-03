function [c,ceq] = PowerContraints(Variable_P_initial)
%PowerContraints �˺���Ϊ���书���Ż��������Լ������
%   �˴���ʾ��ϸ˵��
load('Data_const.mat','NumOfUAV','NumOfNode','NumOfTimeSlot','MaxSpeed','MinDist','Height_UAV','Power_GBS',...
                 'MaxPower','MaxEnergyConsume','T', 'SlotInterval','Gain_UAV','Gain_GBS','LoseExp','BlockPara','BlockPara_UAV','noise_mW','Bandwidth');
load map.mat MTlocation GBSlocation;
load Trajectory.mat q;
load Schedule.mat Schedule_result;
%%
%%���������һ���ֿ��Ե���д��һ������
%����һ����ά���飬N*M*K�����˻����ն˵ľ����ƽ����Ԥ�����ڴ棬��������ٶȡ�
UAVtoMT = zeros(NumOfNode,NumOfTimeSlot,NumOfUAV);
%����һ����ά���飬N*M�������վ���ն˵ľ��롣Ԥ�����ڴ棬��������ٶȡ�
GBStoMT = zeros(NumOfNode,NumOfTimeSlot);
%%�������˻����ն˵ľ����ƽ��
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
P_value = Variable_P_initial(end); %�ָ�����ĸ�������
Variable_P = reshape(Variable_P_initial(1:end-1),[NumOfUAV,NumOfTimeSlot]);
R = zeros(NumOfNode,NumOfTimeSlot,NumOfUAV+1);  %Ԥ�����ڴ�
%%������Ϣ��
for m = 1 : NumOfNode
  for k = 1 : NumOfUAV
      %%�������˻�k���û�mͨ�ŵ���Ϣ��
      R(m,:,k) = Bandwidth*BlockPara_UAV(m,k)*Schedule_result(m,:,k).*log2(ones(1,NumOfTimeSlot)+(Variable_P(k,:)*Gain_UAV/noise_mW)./UAVtoMT(m,:,k));

  end
     %%��������վ���û�mͨ�ŵ���Ϣ��
    R(m,:,NumOfUAV+1) = Bandwidth*Schedule_result(m,:,NumOfUAV+1).*(log2(1+Power_GBS*Gain_GBS/noise_mW/GBStoMT(m)^LoseExp)*BlockPara(m));  
end

%ÿ���û�m��ƽ����Ϣ�ʵ�Լ��
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
   
%���ܺĵ�Լ��
for k= 1:NumOfUAV
    c(i)=sum(Variable_P(k,:)*SlotInterval)-MaxEnergyConsume;
    i = i+1;
end

ceq = [];


end


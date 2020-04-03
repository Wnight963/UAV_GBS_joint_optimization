function [c,ceq] = ScheduleContraints(Variable_S_initial)
%ScheduleContraints �˺���Ϊ�û������Ż��������Լ������
%   �˴���ʾ��ϸ˵��

load('Data_const.mat','NumOfUAV','NumOfNode','NumOfTimeSlot','Height_UAV','Power_GBS',...
                 'Gain_UAV','Gain_GBS','LoseExp','BlockPara','BlockPara_UAV','noise_mW','Bandwidth');
load map.mat MTlocation GBSlocation;
load Trajectory.mat q;
load Power.mat Power;
%%
%����һ����ά���飬N*M*K�����˻����ն˵ľ����ƽ����Ԥ�����ڴ棬��������ٶȡ�
UAVtoMT = zeros(NumOfNode,NumOfTimeSlot,NumOfUAV);
%����һ����ά���飬N*M�������վ���ն˵ľ��롣Ԥ�����ڴ棬��������ٶȡ�
% GBStoMT = zeros(NumOfNode);
GBStoMT = zeros(NumOfNode,NumOfTimeSlot);
%%�������˻����ն˵ľ����ƽ��
for k = 1:NumOfUAV
    for m = 1:NumOfNode 
        UAVtoMT(m,:,k) = 100*((Height_UAV*ones(1,NumOfTimeSlot)).^2+(q(1,:,k)-MTlocation(1,:,m)).^2+...
            (q(2,:,k)-MTlocation(2,:,m)).^2);
    end
end
%%��������վ���ն˵ľ���
for m = 1:NumOfNode
    GBStoMT(m,:) = 10*sqrt((GBSlocation(1,:)-MTlocation(1,:,m)).^2+(GBSlocation(2,:)-MTlocation(2,:,m)).^2);
end
%%
S_value = Variable_S_initial(end);  %�ָ�Ŀ�꺯��
Variable_S_initial = reshape(Variable_S_initial(1:end-1),[NumOfNode,NumOfTimeSlot,NumOfUAV+1]);  %�ָ��û����ȱ���
R = zeros(NumOfNode,NumOfTimeSlot,NumOfUAV+1);  %Ԥ�����ڴ�
%%�������˻�k���û�mͨ�ŵ���Ϣ��
for m = 1 : NumOfNode
  for k = 1 : NumOfUAV
      %%��Ҫע��;ð��������ά����Ҫ����һ�£����ҽ��мӼ�����ʱ���뱣֤����Ĵ�Сһ��
      R(m,:,k) = Bandwidth*BlockPara_UAV(m,k)*Variable_S_initial(m,:,k).*log2(1+(Power(k,:)*Gain_UAV/noise_mW)./UAVtoMT(m,:,k));
  end
     %%��������վ���û�mͨ�ŵ���Ϣ��
     R(m,:,NumOfUAV+1) = Bandwidth*Variable_S_initial(m,:,NumOfUAV+1).*(log2(1+Power_GBS*Gain_GBS/noise_mW./GBStoMT(m,:).^LoseExp)*BlockPara(m));  
end
%%
%ÿ���û�m��ƽ����Ϣ�ʵ�Լ��
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
%������ʱ϶��ÿ���û�mֻ����һ�����˻����ߵ����վͨ��
for m = 1 : NumOfNode
    for n = 1:NumOfTimeSlot
        c(i)=sum(Variable_S_initial(m,n,:))-1; 
        i=i+1;
    end 
end

%������ʱ϶��ÿ�����˻����߻�վֻ����һ���û�ͨ��
for k = 1 : NumOfUAV+1
    for n = 1:NumOfTimeSlot
        c(i)=sum(Variable_S_initial(:,n,k))-1; 
        i=i+1;
    end
end

%�����Ʊ���Լ��������̫�࣬���ǽ�����Ϊ�ͷ����ӷŵ�Ŀ�꺯���С�
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


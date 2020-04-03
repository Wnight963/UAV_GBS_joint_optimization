function [c,ceq] = TrajectoryContraints(Variable_T_initial)
%TrajectoryContraints ·���Ż��������Լ��
%   �˴���ʾ��ϸ˵��
load('Data_const.mat','NumOfUAV','NumOfNode','NumOfTimeSlot','MaxSpeed','MinSpeed','MinDist','Height_UAV','Power_GBS',...
                 'MaxPower','T','SlotInterval','Gain_UAV','Gain_GBS','LoseExp','BlockPara','BlockPara_UAV','noise_mW','Bandwidth');
load map.mat MTlocation GBSlocation;
load Trajectory.mat q;
load Schedule.mat Schedule_result;
load Power.mat Power;
T_value = Variable_T_initial(end);  %�ָ�Ŀ�꺯��
Variable_T_initial = reshape(Variable_T_initial(1:end-1),[2,NumOfTimeSlot,NumOfUAV]);  %�ָ����˻�·������
varphi = Power*Gain_UAV/noise_mW;  %�����м����
%%
UAVtoMT = zeros(NumOfNode,NumOfTimeSlot,NumOfUAV);
%����һ����ά���飬N*M�������վ���ն˵ľ��롣Ԥ�����ڴ棬��������ٶȡ�
% GBStoMT = zeros(NumOfNode);
GBStoMT = zeros(NumOfNode,NumOfTimeSlot);
 %%������һ�ε��������˻����ն˵ľ����ƽ��
for k = 1:NumOfUAV
    for m = 1:NumOfNode 
        UAVtoMT(m,:,k) = 100*((Height_UAV*ones(1,NumOfTimeSlot)).^2+(q(1,:,k)-MTlocation(1,:,m)).^2+...
            (q(2,:,k)-MTlocation(2,:,m)).^2);
    end
end
%���㱾�ε��������˻����ն˵ľ����ƽ��
for k = 1:NumOfUAV
    for m = 1:NumOfNode 
        x(m,:,k) = 100*((Height_UAV*ones(1,NumOfTimeSlot)).^2+(Variable_T_initial(1,:,k)-MTlocation(1,:,m)).^2+...
            (Variable_T_initial(2,:,k)-MTlocation(2,:,m)).^2);
    end
end
%%��������վ���ն˵ľ���
    for m = 1:NumOfNode
          GBStoMT(m,:) = 10*sqrt((GBSlocation(1,:)-MTlocation(1,:,m)).^2+(GBSlocation(2,:)-MTlocation(2,:,m)).^2);
    end
%%
R = zeros(NumOfNode,NumOfTimeSlot,NumOfUAV+1);  %Ԥ�����ڴ�
for m = 1 : NumOfNode
  for k = 1 : NumOfUAV
      %%�������˻�k���û�mͨ�ŵ���Ϣ��
      R(m,:,k) = Bandwidth*BlockPara_UAV(m,k)*Schedule_result(m,:,k).*(log2(1+(Power(k,:)*Gain_UAV/noise_mW)./UAVtoMT(m,:,k))-...
         log2(exp(1))*varphi(k,:).*(x(m,:,k)-UAVtoMT(m,:,k))./UAVtoMT(m,:,k)./(UAVtoMT(m,:,k)+varphi(k,:)));
  end
     %%��������վ���û�mͨ�ŵ���Ϣ��
    R(m,:,NumOfUAV+1) = Bandwidth*Schedule_result(m,:,NumOfUAV+1).*(log2(1+Power_GBS*Gain_GBS/noise_mW/GBStoMT(m)^LoseExp)*BlockPara(m));  
end
%%
%ÿ���û�m��ƽ����Ϣ�ʵ�Լ��
i = 1;

for m = 1 : NumOfNode
    sumR_m = sum(sum(R(m,:,:)));
    c(i)=T_value-1/NumOfTimeSlot*sumR_m;
     i = i+1;
end

%   sumR = sum(sum(sum(R)));
%    c(i)=T_value-1/NumOfTimeSlot*sumR;
%     i = i+1;
    
%���˻���һ��ʱ϶�ڵ������о���Լ��
for  n = 2 : NumOfTimeSlot
    c(i:i+1) = (Variable_T_initial(1,n,:)-Variable_T_initial(1,n-1,:)).^2+(Variable_T_initial(2,n,:)-Variable_T_initial(2,n-1,:)).^2-(MaxSpeed*SlotInterval)^2*ones(1,1,NumOfUAV);
    i = i+2;
end

%���˻���һ��ʱ϶�ڵ���С���о���Լ��
for  n = 2 : NumOfTimeSlot
    c(i:i+1) = (MinSpeed*SlotInterval)^2*ones(1,1,NumOfUAV)-((Variable_T_initial(1,n,:)-Variable_T_initial(1,n-1,:)).^2+(Variable_T_initial(2,n,:)-Variable_T_initial(2,n-1,:)).^2);
    i = i+2;
end


%���˻�֮����С����Լ��

for k=1 : NumOfUAV
   for j =1 : NumOfUAV
      if j~=k
        c(i:i+NumOfTimeSlot-1)= MinDist^2*ones(1,NumOfTimeSlot)+(q(1,:,k)-q(1,:,j)).^2+(q(2,:,k)-q(2,:,j)).^2-...
            2* ((q(1,:,k)-q(1,:,j)).*(Variable_T_initial(1,:,k)-Variable_T_initial(1,:,j))+(q(2,:,k)-q(2,:,j)).*(Variable_T_initial(2,:,k)-Variable_T_initial(2,:,j)));
        i = i+NumOfTimeSlot;
      end
   end
end

% for k=1 : NumOfUAV
%    for j =1 : NumOfUAV
%       if j~=k
%         c(i:i+NumOfTimeSlot-3)= MinDist^2*ones(1,NumOfTimeSlot-2)+(q(1,2:NumOfTimeSlot-1,k)-q(1,2:NumOfTimeSlot-1,j)).^2+(q(2,2:NumOfTimeSlot-1,k)-q(2,2:NumOfTimeSlot-1,j)).^2-...
%             2* ((q(1,2:NumOfTimeSlot-1,k)-q(1,2:NumOfTimeSlot-1,j)).*(Variable_T_initial(1,2:NumOfTimeSlot-1,k)-Variable_T_initial(1,2:NumOfTimeSlot-1,j))+(q(2,2:NumOfTimeSlot-1,k)-q(2,2:NumOfTimeSlot-1,j)).*(Variable_T_initial(2,2:NumOfTimeSlot-1,k)-Variable_T_initial(2,2:NumOfTimeSlot-1,j)));
%         i = i+NumOfTimeSlot-2;
%       end
%    end
% end

%�Ǳպ�·��
%�������յ������ 
% start_point_1=[0;0]; 
% stop_point_1=[10;5];
% start_point_2=[10;10];
% stop_point_2=[0;5];
% 
% start_point_1=[0;0]; 
% stop_point_1=[20;10];
% start_point_2=[20;20];
% stop_point_2=[0;10];
% ceq(1:2) = Variable_T_initial(:,1,1) - start_point_1;
% ceq(3:4) = Variable_T_initial(:,NumOfTimeSlot,1) - stop_point_1; 
% ceq(5:6) = Variable_T_initial(:,1,2) - start_point_2;
% ceq(7:8) = Variable_T_initial(:,NumOfTimeSlot,2) - stop_point_2;

%�պ�·��Լ��
ceq(1:2) = Variable_T_initial(:,1,1) - Variable_T_initial(:,NumOfTimeSlot,1); 
ceq(3:4) = Variable_T_initial(:,1,2) - Variable_T_initial(:,NumOfTimeSlot,2);

% ceq = [];
end


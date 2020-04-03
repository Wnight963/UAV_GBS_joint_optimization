function [Schedule_result,Object_value] = Schedule_optim()
%Schedule_optim �˺������������˻��켣�ͷ��书��ʱ�û����ȵ��Ż����⣬�������⣬ֱ�����
%   �˴���ʾ��ϸ˵��

load('Data_const.mat','NumOfUAV','NumOfNode','NumOfTimeSlot','save_path');
load Schedule.mat Schedule_result 
load Object_value.mat Object_value
%%
%����ά����ת����һά���飬�����ں������ΪĿ�꺯���ı���Object_value��������ͬ��reshape������ԭ
Schedule_temp = reshape(Schedule_result,1,[]); 
%S_value = Object_value; %Ŀ�꺯���еĸ�������
Variable_S_initial = [Schedule_temp,Object_value];  %�Ż�����
ub_S=ones(1,length(Variable_S_initial)-1); %%ֻ���û����ȱ��������Ͻ�
ub_S=[ub_S,+Inf];
lb_S=zeros(1,length(Variable_S_initial)); %�½���Ϊ0
options=optimoptions(@fmincon,'Algorithm','sqp','MaxFunctionEvaluations',40000,'StepTolerance',1.0000e-10,'Display','Iter');
[Variable_S,fval_S]=fmincon(@fun_object_schedule,Variable_S_initial,[],[],[],[],lb_S,ub_S,@ScheduleContraints,options);
Schedule_result = reshape(Variable_S(1:end-1),[NumOfNode,NumOfTimeSlot,NumOfUAV+1]); %���Ż���������ع�
%Schedule_result = Rounding(Schedule_result);
Object_value = -fval_S;  %�������������Ž�
save Object_value.mat Object_value
save('Schedule.mat','Schedule_result');  %�����ļ����켣�Ż��͹����Ż�ʱֱ�Ӽ��ء�
save([save_path,'Schedule.mat'],'Schedule_result');
end













function [Schedule_result,Object_value] = Schedule_optim()
%Schedule_optim 此函数求解给定无人机轨迹和发射功率时用户调度的优化问题，线性问题，直接求解
%   此处显示详细说明

load('Data_const.mat','NumOfUAV','NumOfNode','NumOfTimeSlot','save_path');
load Schedule.mat Schedule_result 
load Object_value.mat Object_value
%%
%将三维数组转化成一维数组，便于在后面接作为目标函数的变量Object_value。可以在同过reshape函数复原
Schedule_temp = reshape(Schedule_result,1,[]); 
%S_value = Object_value; %目标函数中的辅助变量
Variable_S_initial = [Schedule_temp,Object_value];  %优化变量
ub_S=ones(1,length(Variable_S_initial)-1); %%只对用户调度变量设置上界
ub_S=[ub_S,+Inf];
lb_S=zeros(1,length(Variable_S_initial)); %下界设为0
options=optimoptions(@fmincon,'Algorithm','sqp','MaxFunctionEvaluations',40000,'StepTolerance',1.0000e-10,'Display','Iter');
[Variable_S,fval_S]=fmincon(@fun_object_schedule,Variable_S_initial,[],[],[],[],lb_S,ub_S,@ScheduleContraints,options);
Schedule_result = reshape(Variable_S(1:end-1),[NumOfNode,NumOfTimeSlot,NumOfUAV+1]); %将优化结果进行重构
%Schedule_result = Rounding(Schedule_result);
Object_value = -fval_S;  %辅助变量的最优解
save Object_value.mat Object_value
save('Schedule.mat','Schedule_result');  %保存文件，轨迹优化和功率优化时直接加载。
save([save_path,'Schedule.mat'],'Schedule_result');
end













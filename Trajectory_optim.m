function [q,Object_value] = Trajectory_optim()
%Trajectory_optim 无人机轨迹优化子问题
%   此处显示详细说明

load('Data_const.mat','NumOfUAV','NumOfTimeSlot','save_path');
load Trajectory.mat q 
load Object_value.mat Object_value
trajectory_temp = reshape(q,1,[]);   %无人机轨迹初始化
Variable_T_initial = [trajectory_temp,Object_value];  %优化变量
ub_T = 200*ones(1,length(Variable_T_initial)-1); %%只对无人机路径变量设置上界，中心点为（500，500）
ub_T=[ub_T,+Inf];
lb_T = zeros(1,length(Variable_T_initial)-1); %%只对无人机路径变量设置下界，中心点为（500，500）
lb_T=[lb_T,0];
% options=optimoptions(@fmincon,'Algorithm','sqp','OptimalityTolerance',1.000e-8,'StepTolerance',1.0000e-10,'Display','iter');
options=optimoptions(@fmincon,'MaxFunctionEvaluations',40000,'OptimalityTolerance',1.000e-8,'Display','iter');
[Variable_T,fval_T]=fmincon(@fun_object_trajectory,Variable_T_initial,[],[],[],[],lb_T,ub_T,@TrajectoryContraints,options);
q = reshape(Variable_T(1:end-1),[2,NumOfTimeSlot,NumOfUAV]);  %对优化结果进行重构
Object_value = -fval_T;
save Trajectory.mat q
save Object_value.mat Object_value
save([save_path,'Trajectory.mat'],'q');
end


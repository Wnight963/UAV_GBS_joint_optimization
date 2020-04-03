function [Power_result,Object_value] = Power_optim()
%Power_optim 此处显示有关此函数的摘要
%   此处显示详细说明
load('Data_const.mat','NumOfUAV','NumOfNode','NumOfTimeSlot','MaxSpeed','MinDist','Height_UAV','Power_GBS',...
                 'MaxPower','MaxEnergyConsume','T','SlotInterval','Gain_UAV','Gain_GBS','LoseExp','BlockPara','noise_mW','save_path');
load Power.mat Power 
load Object_value.mat Object_value
%Power_temp = reshape(zeros(NumOfUAV,NumOfTimeSlot),1,[]);
Power_temp = reshape(Power,1,[]);
%P_value = Object_value;
Variable_P_initial = [Power_temp,Object_value];
ub_P=MaxPower*ones(1,length(Variable_P_initial)-1);
ub_P=[ub_P,+Inf];
lb_P=zeros(1,length(Variable_P_initial));
options=optimoptions(@fmincon,'Algorithm','interior-point','MaxFunctionEvaluations',10000,'OptimalityTolerance',1.000e-8,'StepTolerance',1.0000e-10,'Display','iter');
%options=optimoptions(@fmincon,'MaxFunctionEvaluations',10000,'OptimalityTolerance',1.000e-8,'StepTolerance',1.0000e-10,'Display','iter');
[Variable_P,fval_P]=fmincon(@fun_object_power,Variable_P_initial,[],[],[],[],lb_P,ub_P,@PowerContraints,options);
Power_result = reshape(Variable_P(1:end-1),[NumOfUAV,NumOfTimeSlot]);
Object_value = -fval_P;
Power = Power_result;
save Power.mat Power
save Object_value.mat Object_value
save([save_path,'Power.mat'],'Power');
end


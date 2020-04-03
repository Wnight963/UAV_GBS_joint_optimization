function [object_P] = fun_object_power(Variable_P_initial)
%fun_object_power 此处显示有关此函数的摘要
%   此处显示详细说明
 object_P = -Variable_P_initial(end);%原问题是求最大值，若要使用fmincon求解器，需要取反求最小值
% load Schedule.mat Schedule_result
% lambda = 1.0e3;
% object_P=-(Variable_P_initial(end)+lambda*sum(sum(sum(Schedule_result.^2-Schedule_result))));
end


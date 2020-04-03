function [object_T] = fun_object_trajectory(Variable_T_initial)
%fun_object_trajectory 路径优化的目标函数
%   此处显示详细说明
% load Data_const.mat;
% i=1;
% q = reshape(Variable_T_initial(1:end-1),[2,NumOfTimeSlot,NumOfUAV]);  %恢复无人机路径变量
% for  n = 1 : NumOfTimeSlot-1
%     c(i:i+1) = (q(1,n+1,:)-q(1,n,:)).^2+(q(2,n+1,:)-q(2,n,:)).^2-(MaxSpeed*SlotInterval)^2*ones(1,1,NumOfUAV)+(MinSpeed*SlotInterval)^2*ones(1,1,NumOfUAV)-((q(1,n+1,:)-q(1,n,:)).^2+(q(2,n+1,:)-q(2,n,:)).^2);
%     i = i+2;
% end
% object_T = -Variable_T_initial(end)+1e3*sum(c);
object_T = -Variable_T_initial(end);
%原问题是求最大值，若要使用fmincon求解器，需要取反求最小值

end


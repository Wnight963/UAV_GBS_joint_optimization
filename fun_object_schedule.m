function object_S = fun_object_schedule(Variable_S_initial)
%fun_object_schedule 用户调度优化子问题的目标函数
%   此处显示详细说明
% load Data_const.mat;
% load Trajectory.mat q;
% i=1;
% for  n = 1 : NumOfTimeSlot-1
%     c(i:i+1) = (q(1,n+1,:)-q(1,n,:)).^2+(q(2,n+1,:)-q(2,n,:)).^2-(MaxSpeed*SlotInterval)^2*ones(1,1,NumOfUAV)+(MinSpeed*SlotInterval)^2*ones(1,1,NumOfUAV)-((q(1,n+1,:)-q(1,n,:)).^2+(q(2,n+1,:)-q(2,n,:)).^2);
%     i = i+2;
% end
% a=sum(c)
% object_S=-Variable_S_initial(end)+1e3*sum(c);
object_S=-Variable_S_initial(end);
end


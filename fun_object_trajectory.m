function [object_T] = fun_object_trajectory(Variable_T_initial)
%fun_object_trajectory ·���Ż���Ŀ�꺯��
%   �˴���ʾ��ϸ˵��
% load Data_const.mat;
% i=1;
% q = reshape(Variable_T_initial(1:end-1),[2,NumOfTimeSlot,NumOfUAV]);  %�ָ����˻�·������
% for  n = 1 : NumOfTimeSlot-1
%     c(i:i+1) = (q(1,n+1,:)-q(1,n,:)).^2+(q(2,n+1,:)-q(2,n,:)).^2-(MaxSpeed*SlotInterval)^2*ones(1,1,NumOfUAV)+(MinSpeed*SlotInterval)^2*ones(1,1,NumOfUAV)-((q(1,n+1,:)-q(1,n,:)).^2+(q(2,n+1,:)-q(2,n,:)).^2);
%     i = i+2;
% end
% object_T = -Variable_T_initial(end)+1e3*sum(c);
object_T = -Variable_T_initial(end);
%ԭ�����������ֵ����Ҫʹ��fmincon���������Ҫȡ������Сֵ

end


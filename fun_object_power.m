function [object_P] = fun_object_power(Variable_P_initial)
%fun_object_power �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
 object_P = -Variable_P_initial(end);%ԭ�����������ֵ����Ҫʹ��fmincon���������Ҫȡ������Сֵ
% load Schedule.mat Schedule_result
% lambda = 1.0e3;
% object_P=-(Variable_P_initial(end)+lambda*sum(sum(sum(Schedule_result.^2-Schedule_result))));
end


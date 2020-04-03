function travel_dist = travel_dis()
%本函数用于计算无人机的飞行距离
load('Data_const.mat','NumOfUAV','NumOfNode','NumOfTimeSlot','MaxSpeed','MinDist','Height_UAV','Power_GBS',...
                 'MaxPower','T','SlotInterval','Gain_UAV','Gain_GBS','LoseExp','BlockPara','noise_mW');
load Trajectory.mat q
travel_dist = zeros(NumOfUAV,1);
for k = 1:NumOfUAV
    for n = 1:NumOfTimeSlot-1 
        travel_dist(k) = travel_dist(k)+100*sqrt((q(1,n+1,k)-q(1,n,k))^2+(q(2,n+1,k)-q(2,n,k))^2);
    end
end

end


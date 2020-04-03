load('Data_const.mat','NumOfUAV','NumOfNode','NumOfTimeSlot','MaxSpeed','MinDist','Height_UAV','Power_GBS',...
                 'MaxPower','MaxEnergyConsume','T', 'SlotInterval','Gain_UAV','Gain_GBS','LoseExp','BlockPara','noise_mW');
Schedule_result=zeros(NumOfNode,NumOfTimeSlot,NumOfUAV+1);

Schedule_result(3,1:20,1) = 1; 
Schedule_result(2,21:48,1) = 1; 
Schedule_result(6,49:72,1) = 1; 

Schedule_result(5,1:24,2) = 1; 
Schedule_result(1,25:55,2) = 1; 
Schedule_result(4,56:72,2) = 1; 

Schedule_result(3,37:72,3) =  1; 
Schedule_result(4,1:36,3) = 1; 


save generate_schedule Schedule_result 
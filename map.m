function [] = map()
load('Data_const.mat','NumOfNode','NumOfTimeSlot','save_path');

 %地面基站的位置坐标，设为原点
GBSlocation=1000*ones(3,NumOfTimeSlot);
GBSlocation(3,:,:) = 0;
plot(GBSlocation(1),GBSlocation(2),'LineStyle','none','Marker','d','MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','k');%绘制地面基站位置
set(gca,'FontSize',12)

MTlocation = ones(3,NumOfTimeSlot,NumOfNode);

% for n = 1 : NumOfTimeSlot
%     MTlocation(:,n,:)=[100,500,700,1400,1600;
%                       1800,600,1450,1700,450;
%                        0,0,0,0,0]; %5个用户
% end
for n = 1 : NumOfTimeSlot
    MTlocation(:,n,:)=[100,330,500,700,1400,1600,1840,;
                      1800,290,600,1450,1700,450,1000;
                       0,0,0,0,0,0,0]; 
end
% for n = 1 : NumOfTimeSlot
% MTlocation(:,n,:)=[100,1200,1400,500,600,1600;
%              1800,800,1700,600,1600,400;
%              0,0,0,0,0,0]; %6个用户,乱序
% end
         
for m=1:NumOfNode
    plot(MTlocation(1,1,m),MTlocation(2,1,m),'LineStyle','none','Marker','d','MarkerSize',10,'MarkerEdgeColor','g','MarkerFaceColor','g');%绘制终端位置
    hold on
end

%将地图尺寸缩小为原来100分之一
MTlocation = MTlocation/10;
GBSlocation = GBSlocation/10;
%%
save('map.mat','MTlocation','GBSlocation'); %保存地图文件，包括无人机路径坐标，终端坐标，地面基站坐标
save([save_path,'map.mat'],'MTlocation','GBSlocation')
end
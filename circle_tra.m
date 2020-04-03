function [] = circle_tra()
load('Data_const.mat','NumOfUAV','NumOfTimeSlot','MaxSpeed','MinDist','Height_UAV','T','width','save_path');      
          
R = width/4;    
x0 = width/2;
y0 = width/2;
Height_UAV = Height_UAV*10;
%%单个无人机轨迹
% for i = 1 : NumOfTimeSlot
% q(1,i,1) = x0 + R*cos(2*pi*i/NumOfTimeSlot);
% q(2,i,1) = y0 + R*sin(2*pi*i/NumOfTimeSlot);
% q(3,i,1) = height;
% end

for k= 1:NumOfUAV
   x(k) =  x0 + R*cos(2*pi*(k-1)/NumOfUAV);
   y(k) =  y0 + R*sin(2*pi*(k-1)/NumOfUAV);
end

if 10*MaxSpeed*T<2*pi*R
    R=floor(10*MaxSpeed*T/2/pi);
end
% 
for k= 1:NumOfUAV
   x(k) =  x0 + R*cos(2*pi*(k-1)/NumOfUAV);
   y(k) =  y0 + R*sin(2*pi*(k-1)/NumOfUAV);
end

q = zeros(2,NumOfTimeSlot,NumOfUAV);
cir_phase = 0;
for k = 1:NumOfUAV
    cir_phase = cir_phase + 2*pi/NumOfUAV;  %加上初始相位，保证轨迹起点靠近地面基站。
    for i = 1 : NumOfTimeSlot
        q(1,i,k) = x(k) + R*cos(2*pi*i/NumOfTimeSlot+cir_phase);
        q(2,i,k) = y(k) + R*sin(2*pi*i/NumOfTimeSlot+cir_phase);
    end
end

for k = 1:NumOfUAV
   for j=1:NumOfUAV
        if k~=j
            for i = 2 : NumOfTimeSlot-1
                if (q(1,i,k)- q(1,i,j))^2+(q(2,i,k)- q(2,i,j))^2<(MinDist*10)^2
                     disp('有碰撞')
                end
            end
        end
    end
end

figure(1)

plot3(q(1,:,1),q(2,:,1),Height_UAV*ones(1,NumOfTimeSlot),'-o','color','b','LineWidth',2); %绘制UAV-1轨迹
hold on
plot3(q(1,:,2),q(2,:,2),Height_UAV*ones(1,NumOfTimeSlot),'-s','color','r','LineWidth',2); %绘制UAV-2轨迹
axis([0 width 0 width 0 500])
grid on
set(gca,'FontSize',13)
map()
plot3(q(1,1,1),q(2,1,1),Height_UAV*ones(1,NumOfTimeSlot),'-s','color','y','LineWidth',2); %绘制UAV-2轨迹
plot3(q(1,1,2),q(2,1,2),Height_UAV*ones(1,NumOfTimeSlot),'-s','color','y','LineWidth',2); %绘制UAV-2轨迹

hold off
legend({'UAV-1','UAV-2','GBS','User'},'Location','southeast')

title("UAVs' Initial Trajectory",'FontSize',16)
xlabel('X/m')
ylabel('Y/m')
zlabel('H/m')
saveas(gcf,[save_path,'Initial Trajectory.fig'])
saveas(gcf,[save_path,'Initial Trajectory.jpg'])
%将地图尺寸缩小为原来100分之一
q=q/10;
save('Trajectory.mat','q');
end
clear;
clc;
disp('A Star Path Planing start!!')
tic;
map.XYMAX=50; %%代表我们要画一个地图的长和宽
map.start=[1,1];  %起始点 注意必须在地图范围内
map.goal=[40,40];  %目标点 注意必须在地图范围内
Environmental_Set=1;  %判断是否随机生成障碍物，为0时会保留上次生成的障碍物信息

obstacle=GetBoundary(map);%得到边界数据
nObstacle=300;%在地图中随机加入XX个障碍物
obstacle=GetObstacle(nObstacle,obstacle,map);%障碍物和边界坐标
obstacle = [obstacle;10,1; 11,20; 11,40; 25,6; 35,7 ;36,19;];%全封死的情况，是没有路的
%obstacle = [obstacle;1,2;2,1;2,2];%此也为全封死的情况，也没有路的
%obstacle = [obstacle;1,3;2,3;3,3;3,2;3,1];%此也为全封死情况，也没有路的
%
if(Environmental_Set)
obstacle=GetObstacle(nObstacle,obstacle,map);%障碍物和边界坐标 %随机生成包含障碍物，起始点，终止点等信息的矩阵
save('obstacle');
else
load('obstacle')
end
%load('obstacle1.mat');
%画出网格线
PlotGrid(map);
hold on;

%画出障碍点
FillPlot(obstacle,'k');

open=[map.start(1),map.start(2),0,0,0,h(map,map.start,map.goal),0,0];
% closelist的格式：jump_point_x|junmp_point_y|g_cost|father_node_x|father_node_x
%初始化closelist
close=[];
[open,close]=AStar(open,close,obstacle,map);%A*算法

%画出路径
%
for pp=1:length(close(:,1))
    x = close(pp,1);
    y = close(pp,2);
    A(pp,1)=x;
    A(pp,2)=y;%这部分是将路径坐标拿出来另外存放
end
plot( A(:,1), A(:,2),'b','linewidth',4)   %绘制路线
    
k=0;%用于存放路径长度的变量
for i=1:length(close(:,1))-1
    b=10*sqrt(((close(i+1,1)-close(i,1))^2)+((close(i+1,2)-close(i,2))^2));%简单的两点间距离公式
    k=k+b;%路径长度值的逐个累加
end
disp(['路径长度为',num2str(k)]);
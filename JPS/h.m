function hcost = h(map, m,goal )
weight=20;
cost1=sqrt((goal(1)-map.start(1))^2+(goal(2)-map.start(2))^2);
cost2=sqrt((m(1)-goal(1))^2+(m(2)-goal(2))^2);
if(cost1/cost2>0.3)
    weight=min(20.8,20+cost1/cost2);
end
%计算启发函数代价值 ，这里采用欧式算法
hcost =weight*sqrt((m(1)-goal(1))^2+(m(2)-goal(2))^2);

end

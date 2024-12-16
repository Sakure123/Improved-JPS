function [open,close,open1,close1]=AStar(open1,close1,open,close,obstacle,map)

%{
Astar算法思路
1.将起始点放在Openlist中
2.重复以下过程：
  
  首先判断是否到达目标点，或无路径
    >>如果终点已加入到Openlist中，则已找到路径（此时起始点就是目标点，无需再找）
    >>Openlist为空，无路径

  a.按照Openlist中的第三列（代价函数F）进行排序，查找F值最小的节点
  b.把这个F值最小的节点移到Closelist中作为 当前节点
  c.对当前节点周围的8个相邻节点：
    >>如果它不可达，忽略它
    >>如果它在Closelist中，忽略它
    >>如果它不在Openlist中，加放Openlist，并把当前节点设置为它的父节点，记录该节点的F值
    >>如果它已经在Openlist中，检查经当前节点到达那里是否更好(用G或F值判断)，
         >如果更好，则将当前节点设置为其父节点，并更新F,G值；如果不好，则不作处理

3.保存路径
%}



%用于存储路径
path=[];
%findFlag用于判断while循环是否结束
findFlag=false;

%================1.将起始点放在Openlist中======================
%open变量每一行  [节点坐标，代价值G,方向X,方向Y,代价值F=G+H,父节点坐标]

%更新状态--下一步的八个点
next=MotionModel();
%从起点的8个方向开始寻找跳点
for i=1:8
    father_node=open(1,1:3);
    dir=next(i,1:2);
    [jump_point,~]=article_jump(father_node,dir,map.start,map.goal,obstacle);
    if ~isequal(jump_point,[])
        successor=[jump_point(1),jump_point(2),jump_point(3),dir,jump_point(3)+...
            h(map,jump_point,map.goal),father_node(1),father_node(2)];
        open=insert_successor(successor,open);
    end
end
close=[open(1,1:3),open(1,7:8);close];
open(1,:)=[];

for i=1:8
    father_node1=open1(1,1:3);
    dir1=next(i,1:2);
    [jump_point1,~]=article_jump(father_node1,dir1,map.start,map.goal,obstacle);
    if ~isequal(jump_point1,[])
        successor1=[jump_point1(1),jump_point1(2),jump_point1(3),dir1,jump_point1(3)+...
            h(map,jump_point1,map.start),father_node1(1),father_node1(2)];
        open1=insert_successor(successor1,open1);
    end
end
close1=[open1(1,1:3),open1(1,7:8);close1];
open1(1,:)=[];
%=======================2.重复以下过程==============================

while ~findFlag
    


%--------------------首先判断是否达到目标点，或无路径-----
    if (isempty(open(:,1))&&isempty(open1(:,1)))
        disp('No path to goal!!');
        return;
    end
    
    %判断目标点是否出现在open列表中
    [common_nodes,idx_A,idx_B]=intersect(open(:,[1,2]),open1(:,[1,2]),'rows');
    if ~isempty(common_nodes)
        disp('Find Goal!!');
        toc;
        %close = insert_closelist(open(idx_A,:),close);
        %close1= insert_closelist(open1(idx_B,:),close1);
        findFlag=true;
        break;
    end


    %------------------a.按照Openlist中的第六列（代价函数F）进行排序，查找F值最小的节点
    [Y,I] = sort(open(:,6)); %对OpenList中第六列排序
    [Y1,I1]=sort(open1(:,6));
    open=open(I,:);%open中第一行节点是F值最小的
    open1=open1(I1,:);
    if length(open(:,1))>1
            %如果排序后的openlist中前两行的f_cost值相等 根据距离目标值的横纵距离排序 距离大的换到openlist第一行 作为选取的节点
            if isequal(open(1,6),open(2,6))
                dist1=sqrt((open(1,1)-map.goal(1))^2+(open(1,2)-map.goal(2))^2);
                dist2=sqrt((open(1,1)-map.goal(1))^2+(open(1,2)-map.goal(2))^2);
                dist_big=dist1;
                if dist1 > dist2  %将与目标点横纵距离大的点换到openlist第一行 作为选取的节点
                    temp=open(2,:);
                    open(2,:)=[];
                    open=[temp;open];
                    dist_big=dist2;
                end
                %比较openlist中其他行是否还有f_cost与第一行相同的点 如果有继续调整
                for j = 3:length(open(:,1))
                    if isequal(open(1,6),open(j,6))
                        dist=sqrt((open(1,1)-map.goal(1))^2+(open(1,2)-map.goal(2))^2);
                        if dist_big > dist
                            temp=open(j,:);
                            open(j,:)=[];
                            open=[temp;open];
                            dist_big=dist;
                        end
                    end
                end
            end      
        end
    if length(open1(:,1))>1
            %如果排序后的openlist中前两行的f_cost值相等 根据距离目标值的横纵距离排序 距离大的换到openlist第一行 作为选取的节点
            if isequal(open1(1,6),open1(2,6))
                dist11=sqrt((open1(1,1)-map.start(1))^2+(open1(1,2)-map.start(2))^2);
                dist22=sqrt((open1(1,1)-map.start(1))^2+(open1(1,2)-map.start(2))^2);
                dist_big1=dist11;
                if dist11 > dist22  %将与目标点横纵距离大的点换到openlist第一行 作为选取的节点
                    temp=open1(2,:);
                    open1(2,:)=[];
                    open1=[temp;open1];
                    dist_big1=dist22;
                end
                %比较openlist中其他行是否还有f_cost与第一行相同的点 如果有继续调整
                for j = 3:length(open1(:,1))
                    if isequal(open1(1,6),open1(j,6))
                        dist0=sqrt((open1(1,1)-map.start(1))^2+(open1(1,2)-map.start(2))^2);
                        if dist_big1 > dist0
                            temp=open1(j,:);
                            open1(j,:)=[];
                            open1=[temp;open1];
                            dist_big1=dist0;
                        end
                    end
                end
            end      
        end
    %------------------b.将F值最小的节点(即open中第一行节点)，放到close第一行(close是不断积压的)，作为当前节点
    dir=open(1,4:5);
    dir1=open1(1,4:5);
    father_node=open(1,1:3);
    father_node1=open1(1,1:3);
    if abs(dir(1))+abs(dir(2)) == 2
            for i=1:2
                if i==1    %竖直方向
                    new_dir=[0,dir(2)];
                 else      %水平方向                
                    new_dir =[dir(1),0];
                 end
                 [new_node,~]=article_jump(father_node,new_dir,map.start,map.goal,obstacle);
                 if ~isequal(new_node ,[])
                     successor=[new_node,new_dir,new_node(3)+h(map,new_node,map.goal),...
                                father_node(1),father_node(2)];
                     open = insert_successor(successor,open);
                 end
            end
        else 
           %从opnelist中弹出的方向是直线，先直线寻找跳点，搜索到了就加入openlist;
           [jump_point,~]=article_jump(father_node,dir,map.start,map.goal,obstacle);
           if ~isequal(jump_point,[])%~isempty(jump_point)
               successor=[jump_point,dir,jump_point(3)+h(map,jump_point,map.goal),...
                          father_node(1),father_node(2)];
                open = insert_successor(successor,open);
           end    
        end
        
        %判断从opnelist中弹出节点，沿其最后被加入的方向是否有强迫邻居 如果有 将其加入到openlist
        [flag,forcedNeigh]=hasForcedNeigh(father_node(1,1:2),father_node(1,1:2)-dir,map.start,map.goal,obstacle);
       if flag
          % 找到的所有的forcedNeigh加入到openlist
          for i=1:length(forcedNeigh(:,1))
              new_dir=forcedNeigh(i,:)-father_node(1,1:2);
              successor=[forcedNeigh(i,:),father_node(3)+14,new_dir,father_node(3)+14+h(map,forcedNeigh,map.goal),...
                         father_node(1),father_node(2)]; 
              open = insert_successor(successor,open);
          end
       end
        %继续沿从opnelist中的方向向前寻找跳点
        [jump_point,~]=article_jump(father_node,dir,map.start,map.goal,obstacle);
        %如果存在则更新从opnelist中弹出的点到closelist中 并将其从openlist中删除 将本次找到的跳点插入到openlist
        if ~isequal(jump_point,[])      %当时忘了可以这样判断--->~isempty(jump_point)  懒得改了 ^_^          
            successor=[jump_point,dir,jump_point(3)+h(map,jump_point,map.goal),...
                       father_node(1),father_node(2)];
            
            close = insert_closelist(open(1,:),close);
            open(1,:)=[];
            open = insert_successor(successor,open);
        else   %如果不存在则更新从opnelist中弹出的点到closelist中 并将其从openlist中删除
            close = insert_closelist(open(1,:),close);
            open(1,:)=[]; 
        end
   
    if abs(dir1(1))+abs(dir1(2)) == 2
            for i=1:2
                if i==1    %竖直方向
                    new_dir=[0,dir1(2)];
                 else      %水平方向                
                    new_dir =[dir1(1),0];
                 end
                 [new_node,~]=article_jump(father_node1,new_dir,map.start,map.goal,obstacle);
                 if ~isequal(new_node ,[])
                     successor=[new_node,new_dir,new_node(3)+h(map,new_node,map.start),...
                                father_node1(1),father_node1(2)];
                     open1 = insert_successor(successor,open1);
                 end
            end
        else 
           %从opnelist中弹出的方向是直线，先直线寻找跳点，搜索到了就加入openlist;
           [jump_point,~]=article_jump(father_node1,dir1,map.start,map.goal,obstacle);
           if ~isequal(jump_point,[])%~isempty(jump_point)
               successor=[jump_point,dir1,jump_point(3)+h(map,jump_point,map.start),...
                          father_node1(1),father_node1(2)];
                open1 = insert_successor(successor,open1);
           end    
        end
        
        %判断从opnelist中弹出节点，沿其最后被加入的方向是否有强迫邻居 如果有 将其加入到openlist
        [flag1,forcedNeigh]=hasForcedNeigh(father_node1(1,1:2),father_node1(1,1:2)-dir1,map.start,map.goal,obstacle);
       if flag1
          % 找到的所有的forcedNeigh加入到openlist
          for i=1:length(forcedNeigh(:,1))
              new_dir=forcedNeigh(i,:)-father_node1(1,1:2);
              successor=[forcedNeigh(i,:),father_node1(3)+14,new_dir,father_node1(3)+14+h(map,forcedNeigh,map.start),...
                         father_node1(1),father_node1(2)]; 
              open1 = insert_successor(successor,open1);
          end
       end
        %继续沿从opnelist中的方向向前寻找跳点
        [jump_point,~]=article_jump(father_node1,dir1,map.start,map.goal,obstacle);
        %如果存在则更新从opnelist中弹出的点到closelist中 并将其从openlist中删除 将本次找到的跳点插入到openlist
        if ~isequal(jump_point,[])      %当时忘了可以这样判断--->~isempty(jump_point)  懒得改了 ^_^          
            successor=[jump_point,dir1,jump_point(3)+h(map,jump_point,map.start),...
                       father_node1(1),father_node1(2)];
            
            close1 = insert_closelist(open1(1,:),close1);
            open1(1,:)=[];
            open1 = insert_successor(successor,open1);
        else   %如果不存在则更新从opnelist中弹出的点到closelist中 并将其从openlist中删除
            close1 = insert_closelist(open1(1,:),close1);
            open1(1,:)=[]; 
        end
   
    
    %=====绘制======
    PlotGrid(map);
    hold on;
    pause(0.000001);
    %绘制节点close和open节点
    FillPlot(close,'r');
    FillPlot(close1,'r');
    hold on;
    FillPlot(open,'g');
    FillPlot(open1,'g');
    hold on;
    

end

%追溯路径


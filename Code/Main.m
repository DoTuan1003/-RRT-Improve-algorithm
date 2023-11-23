clear;
clearvars;
close all;
x_max = 1000;
y_max = 1000;
xplotnode=[];
tplotnode=[];
r2=500;
os=5;
obstacle(1).coord = [250,300,50,400];
obstacle(2).coord = [300,650,400,50];
obstacle(3).coord = [700,300,50,400];
obstacle(4).coord= [300,300,100,50];
obstacle(5).coord = [600,300,100,50];

EPS = 20;
numNodes = 1000; 
g=100;%so lan chia doan thang lan 2
z=200;% so lan chia doan thang lan 1

q_start.coord = [500 500];
q_start.cost = 0;
q_start.parent = 0;
q_goal.coord = [600 900];
q_goal.cost = 0;

nodes(1) = q_start;
figure(2)
axis([0 x_max 0 y_max])
for i=1:1:os
rectangle('Position',obstacle(i).coord,'EdgeColor','none','FaceColor','b');
end

hold on

for i = 1:1:numNodes
    Numbernode=i;
   q_rand = [floor(rand(1)*x_max) floor(rand(1)*y_max)];
    
    % Break if goal node is already reached
    for j = 1:1:length(nodes)
        if nodes(j).coord == q_goal.coord
            break
        end
    end
    
    % Pick the closest node from existing list to branch out from
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = dist(n.coord, q_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near = nodes(idx);
    
    q_new.coord = steer(q_rand, q_near.coord, val, EPS);
    if noCollisionre(q_rand, q_near.coord,os, obstacle)
        
        q_new.cost = dist(q_new.coord, q_near.coord) + q_near.cost;
        
        % Within a radius of r, find all existing nodes
        q_nearest = [];
        r = 70;
        neighbor_count = 1;
        for j = 1:1:length(nodes)
            if noCollisionre(nodes(j).coord, q_new.coord,os, obstacle) && dist(nodes(j).coord, q_new.coord) <= r
                q_nearest(neighbor_count).coord = nodes(j).coord;
                q_nearest(neighbor_count).cost = nodes(j).cost;
                neighbor_count = neighbor_count+1;
            end
        end
        
        % Initialize cost to currently known value
        q_min = q_near;
        C_min = q_new.cost;
        
        % Iterate through all nearest neighbors to find alternate lower
        % cost paths
        
        for k = 1:1:length(q_nearest)
            if noCollisionre(q_nearest(k).coord, q_new.coord,os, obstacle) && q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord) < C_min
                q_min = q_nearest(k);
                C_min = q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord);
                
            end
        end
        line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], 'Color', 'k');                
               drawnow
        hold on
        % Update parent to least cost-from node
        for j = 1:1:length(nodes)
            if nodes(j).coord == q_min.coord
                q_new.parent = j;
            end
        end
        
        % Append to nodes
        nodes = [nodes q_new];
        D = [];
        MD= [];
        % review
         for k = 1:1:length(q_nearest)
            if noCollisionre(q_nearest(k).coord, q_new.coord,os, obstacle) && q_new.cost + dist(q_new.coord,q_nearest(k).coord) < q_nearest(k).cost
                
                q_child=q_nearest(k).coord;
                cost_child=q_new.cost + dist(q_new.coord,q_nearest(k).coord);
                line([q_nearest(k).coord(1), q_new.coord(1)], [q_nearest(k).coord(2), q_new.coord(2)], 'Color', 'y');                
                hold on
                for j = 1:1:length(nodes)
                 if nodes(j).coord == q_child
                     nodes(j).parent=  length(nodes);
                     nodes(j).cost=cost_child;
                 end
                 end
            end
         end

for j = 1:1:length(nodes)
    if noCollisionre(nodes(j).coord, q_goal.coord,os, obstacle)&& dist(nodes(j).coord, q_goal.coord) <= r2
      tmpdist = dist(nodes(j).coord, q_goal.coord);
      D = [D tmpdist];
      MD = [MD j];
    end
end
if length(D)~=0
% Search backwards from goal to start to find the optimal least cost path
      [val, idx] = min(D);
      idx=MD(idx);
      xplotnode=[xplotnode Numbernode];
      costnode=nodes(idx).cost+dist(nodes(idx).coord, q_goal.coord);
      tplotnode=[tplotnode  costnode];             
end
    end
end
if length(D)~=0
q_final = nodes(idx);
q_goal.parent = idx;
q_end = q_goal;
nodes = [nodes q_goal];
T=[];
T=[T length(nodes)];
while q_end.parent ~= 0
    start = q_end.parent;
    T=[ T start];
    q_end = nodes(start);   
end
else
    disp(' run return');
end
% PathOptimization
L = PathOptimization(nodes,T,os,obstacle);
%ve lai
L=[L length(T)];
ford = q_goal;
M=[];
M=[M length(nodes)];
for k = 1:1:length(L)
    back = T(L(k));
    M=[M back];
    line([ford.coord(1), nodes(back ).coord(1)], [ford.coord(2), nodes(back ).coord(2)], 'Color', 'r', 'LineWidth', 2);
     drawnow
    hold on
    ford = nodes(back );
end
% chia kc
renod(1)=q_goal;
for k = 1:1:length(M)-1
xy1 = nodes(M(k)).coord;
xy2 = nodes(M(k+1)).coord;
l=z;% so lan chia doan thang 1
t = linspace(0,1,l+1)';
xy = (1-t)*xy1 + t*xy2;
for i = 1:1:l+1
b=[xy(i,1) xy(i,2)];
renod(length(renod)+1).coord=b;
end
end

%DivisionPathOptimization lan 1
Lmot= DivisionPathOptimization(renod,os,obstacle);
%sua dau duong di
Ldau=[];
dkstop=1;
while dkstop ~=0
for j = (length(renod)-1):-1:1
 if noCollisionre(renod(j).coord, renod(length(renod)).coord,os, obstacle)

 else 
   kqdau= j+1;
   Ldau=[Ldau kqdau];
  dkstop=0;
  break;
 end
end

end

%ve lai lan2 
Lmot(length(Lmot))=Ldau(1);
Lmot=[Lmot length(renod)];
fordmot = q_goal;
Mhai=[];
Mhai=[Mhai 1];
for k = 1:1:length(Lmot)
    backmot = Lmot(k);
    Mhai=[Mhai backmot];
    line([fordmot.coord(1), renod(backmot).coord(1)], [fordmot.coord(2), renod(backmot ).coord(2)], 'Color', 'b', 'LineWidth', 2);
     drawnow
    hold on
    fordmot = renod(backmot);
end
%thuat toan 3
st=0;
n=0;
costmapss=0;
costmaplt=0;
while st~=1
 %chia kc
rerenod = divide_distance(renod,Mhai,q_goal,g);
% smart
Lhai = DivisionPathOptimization(rerenod,os,obstacle);
Lhai=[Lhai length(rerenod)];
% tôi uu
kq = IntelligentPathOptimization(rerenod,Lhai,os,obstacle);
kq=[kq Lhai(length(Lhai))];%luu diem dau
Mhai=[];
fordba = q_goal;
Mhai=[Mhai 1];
   if costmapss-costmaplt<=0 && n>1
       st=1;
   end
  costmapss=costmaplt;
  costmaplt=0;
  
for k = 1:1:length(kq)
    backba = kq(k);
    Mhai=[Mhai backba];
    
    costmaptg=dist(rerenod(backba).coord, fordba.coord);
    costmaplt=costmaplt+costmaptg;

    fordba = rerenod(backba);
end
  
 
renod = struct('coord',[],'cost',[],'parent',[]);
renod=rerenod;
n=n+1
end
%ve lai lan cuoi
fordba = q_goal;
for k = 1:1:length(kq)
    backba = kq(k);
    line([fordba.coord(1), rerenod(backba).coord(1)], [fordba.coord(2), rerenod(backba).coord(2)], 'Color', 'y', 'LineWidth', 2);
     drawnow
    hold on
    fordba = rerenod(backba);
end






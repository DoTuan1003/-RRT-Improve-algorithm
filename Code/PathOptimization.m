function L = PathOptimization(nodes,T,os,obstacle)
L=[];
Tuan=1;
dung=1;
while dung ~=0
for j = Tuan:1:(length(T)-1)
 if noCollisionre(nodes(T(1+j)).coord, nodes(T(Tuan)).coord,os, obstacle)
     count=j+1;
 else 
  Tuan= j;
  L=[L Tuan];
  break;
 end
end
 if count>=(length(T)-1)
     L=[L Tuan];
     dung=0;
  end
end
end
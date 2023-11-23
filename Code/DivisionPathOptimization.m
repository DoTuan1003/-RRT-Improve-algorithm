function Lhai = smart_one(rerenod,os,obstacle)
Lhai=[];
Tuanhai=1;
dunghai=1;

while dunghai ~=0
for j = Tuanhai:1:(length(rerenod)-1)
 if noCollisionre(rerenod(1+j).coord, rerenod(Tuanhai).coord,os, obstacle)
     count=j+1;
 else 
   Tuanhai= j;
  Lhai=[Lhai Tuanhai];
  break;
 end
end
 if count>=length(rerenod)
     dunghai=0;
  end
end
end
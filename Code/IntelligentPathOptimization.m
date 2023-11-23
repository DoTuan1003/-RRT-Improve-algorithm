function kq = path_optimization(rerenod,Lhai,os,obstacle)
kq=[];
count=1;
pointb=1;
pointa=0;
dungdi=1;
while dungdi ~=0
for j = pointb:1:(length(rerenod)-1)
 if noCollisionre(rerenod(j+1).coord, rerenod(pointb).coord,os, obstacle)
    counthai=j+1;
 else 
   tg= j;
   if tg==Lhai(count)
       pointb=pointb+1;
       if pointb+1>=Lhai(count)
           pointb=tg+1;
           kq=[kq tg];
           count=count+1;
           break;
       end
       break;
   else
       pointa=pointb;
       pointb=tg;
       kq=[kq pointa]; 
       kq=[kq pointb];
       count=count+1;
       break;
   end
  break; 
 end
end
if count >=length(Lhai)||counthai>=length(rerenod)
    kq=[kq tg];
    dungdi=0;
end
end
end
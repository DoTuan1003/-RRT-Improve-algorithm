function nc = noCollisionre(n2, n1,n,o)
mkn=0;
for i = 1:1:n
 if noCollision(n2, n1, o(i).coord)
     mkn=mkn+1;
 else
     mkn=0;
 end
if mkn==n
    nc=1;
else
    nc=0;
end
end
end

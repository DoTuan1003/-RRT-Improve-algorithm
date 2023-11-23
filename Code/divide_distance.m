function rerenod = divide_distance(renod,Mhai,q_goal,g)
rerenod = struct('coord',[],'cost',[],'parent',[]);
rerenod(1)=q_goal;
for k = 1:1:length(Mhai)-1
xy1 = renod(Mhai(k)).coord;
xy2 = renod(Mhai(k+1)).coord;
l=g;% so lan chia doan thang 2
t = linspace(0,1,l+1)';
xy = (1-t)*xy1 + t*xy2;
for i = 2:1:l+1
b=[xy(i,1) xy(i,2)];
rerenod(length(rerenod)+1).coord=b;
end
end
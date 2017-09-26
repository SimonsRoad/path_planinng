function DrawAxis(T,l)
% This function draw axis (cartesian).
% Input T= SE3 object / l= length of axis
O=T.t;
pnts=l*eye(3);
pnts=T*pnts;
str={'r','g','b'};
hold on
for i=1:3
quiver3(O(1),O(2),O(3),pnts(1,i)-O(1),pnts(2,i)-O(2),pnts(3,i)-O(3),'Color',str{i},'LineWidth',2);
end

end
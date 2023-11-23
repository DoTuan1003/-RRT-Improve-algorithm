function backba = draw(rerenod,kq,q_goal)
fordba = q_goal;
for k = 1:1:length(kq)
    backba = kq(k);
    line([fordba.coord(1), rerenod(backba).coord(1)], [fordba.coord(2), rerenod(backba).coord(2)], 'Color', 'y', 'LineWidth', 2);
     drawnow
    hold on
    fordba = rerenod(backba);
end
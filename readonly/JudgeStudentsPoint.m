function finalPoint=JudgeStudentsPoint()
load '6formation_results.mat'

[~, num] = size(x0);
pointNum = size(thtraj1.X,2);
k = 0.5;

%% statistics position error, include x, y and z
% if you add the drone num, you should add the additional position error
absPositionErrormean = sum(abs(thtraj1.X-ehtraj1.X)+abs(thtraj1.Y-ehtraj1.Y)+abs(thtraj1.Z-ehtraj1.Z) ...
    +abs(thtraj2.X-ehtraj2.X)+abs(thtraj2.Y-ehtraj2.Y)+abs(thtraj2.Z-ehtraj2.Z) ...
    +abs(thtraj3.X-ehtraj3.X)+abs(thtraj3.Y-ehtraj3.Y)+abs(thtraj3.Z-ehtraj3.Z) ...
    +abs(thtraj4.X-ehtraj4.X)+abs(thtraj4.Y-ehtraj4.Y)+abs(thtraj4.Z-ehtraj4.Z) ...
    +abs(thtraj5.X-ehtraj5.X)+abs(thtraj5.Y-ehtraj5.Y)+abs(thtraj5.Z-ehtraj5.Z) ...
    +abs(thtraj6.X-ehtraj6.X)+abs(thtraj6.Y-ehtraj6.Y)+abs(thtraj6.Z-ehtraj6.Z))/num/pointNum; % 203.1524



finalPoint = absPositionErrormean*100 + time;

disp(finalPoint);    %20.3076
end
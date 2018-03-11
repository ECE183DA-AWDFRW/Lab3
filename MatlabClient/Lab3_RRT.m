[tree, path] = RRT(2000,38, 34, 0, 0, 15,15, 5)


function [tree, path] = RRT(sample_size, box_X, box_Y, cur_x, cur_y, goal_x, goal_y, threshold)
tree.vertex(1).x = cur_x;
tree.vertex(1).y = cur_y;
tree.vertex(1).xPrev = cur_x;
tree.vertex(1).yPrev = cur_y;
tree.vertex(1).dist=0;
tree.vertex(1).ind = 1; tree.vertex(1).indPrev = 0;

xArray=cur_x; yArray = cur_y;

figure(1); hold on; grid on;
plot(cur_x, cur_y, 'ko', 'MarkerSize',10, 'MarkerFaceColor','k');
plot(goal_x, goal_y, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');

for iter = 2:sample_size
    xRand = box_X*rand;
    yRand = box_Y*rand;
    dist = Inf*ones(1,length(tree.vertex));
    for j = 1:length(tree.vertex)
        dist(j) = sqrt( (xRand-tree.vertex(j).x)^2 + (yRand-tree.vertex(j).y)^2 );
    end
    [val, ind] = min(dist);
       
    tree.vertex(iter).x = xRand; tree.vertex(iter).y = yRand;
    tree.vertex(iter).dist = val;
    tree.vertex(iter).xPrev = tree.vertex(ind).x;
    tree.vertex(iter).yPrev = tree.vertex(ind).y;
    tree.vertex(iter).ind = iter; tree.vertex(iter).indPrev = ind;
    
    if sqrt( (xRand-goal_x)^2 + (yRand-goal_y)^2 ) <= threshold
        plot([tree.vertex(iter).x; tree.vertex(ind).x],[tree.vertex(iter).y; tree.vertex(ind).y], 'r');
        break
    end
    
    plot([tree.vertex(iter).x; tree.vertex(ind).x],[tree.vertex(iter).y; tree.vertex(ind).y], 'r');
    pause(0);
end

if iter < sample_size
    path.pos(1).x = goal_x; path.pos(1).y = goal_y;
    path.pos(2).x = tree.vertex(end).x; path.pos(2).y = tree.vertex(end).y;
    pathIndex = tree.vertex(end).indPrev;

    j=0;
    while 1
        path.pos(j+3).x = tree.vertex(pathIndex).x;
        path.pos(j+3).y = tree.vertex(pathIndex).y;
        pathIndex = tree.vertex(pathIndex).indPrev;
        if pathIndex == 1
            break
        end
        j=j+1;
    end

    path.pos(end+1).x = cur_x; path.pos(end).y = cur_y;

    for j = 2:length(path.pos)
        plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 3);
    %     plot([tree.vertex(i).x; tree.vertex(ind).x],[tree.vertex(i).y; tree.vertex(ind).y], 'r');
    %     pause(0);
    end
else
    disp('No path found. Increase number of iterations and retry.');
end
end
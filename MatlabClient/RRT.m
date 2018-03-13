
function [tree, path] = RRT(sample_size, box_X, box_Y, cur_x, cur_y, goal_x, goal_y, objs, step_size, threshold)
    %Initialize tree
    tree.vertex(1).x = cur_x;
    tree.vertex(1).y = cur_y;
    tree.vertex(1).xPrev = cur_x;
    tree.vertex(1).yPrev = cur_y;
    tree.vertex(1).dist=0;
    tree.vertex(1).ind = 1; tree.vertex(1).indPrev = 0;

    figure(1); hold on; grid on;
    plot(cur_x, cur_y, 'ko', 'MarkerSize',10, 'MarkerFaceColor','k');
    plot(goal_x, goal_y, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');
    iter = 2;
    while( iter < sample_size)
        %Give slight probability to go straight to goal(can speed up/slightly optimize path)
        chance = rand;
        if(chance > .1)
            xRand = box_X*rand;
            yRand = box_Y*rand;
        else
            xRand = goal_x;
            yRand = goal_y;
        end
        dist = Inf*ones(1,length(tree.vertex));
        
        %find closest point in current tree to connect it to
        for j = 1:length(tree.vertex)
            dist(j) = sqrt( (xRand-tree.vertex(j).x)^2 + (yRand-tree.vertex(j).y)^2 );
        end
        [val, ind] = min(dist);
        
        %Find the angle, and build straight line until obstacle/point
        x_temp = tree.vertex(ind).x;
        y_temp = tree.vertex(ind).y;
        angle = atan2(yRand-y_temp, xRand-x_temp);
        i_temp = ind;
        x_delta = step_size*cos(angle);
        y_delta = step_size*sin(angle);
        passed_counter = 1;
        passed_flag = false;
        goal_flag = false;
        while( iter < sample_size && pdist([x_temp, y_temp; xRand, yRand], 'Euclidean') > .5)
            collision = false;
            new_x = x_temp + x_delta;
            new_y = y_temp + y_delta;
            [r,c]=size(objs);
            for j = 1:r
                if(new_x > objs(j,1) && new_x < objs(j,2) && new_y > objs(j,3) && new_y < objs(j,4))
                   collision = true;
                    break;
                end
            end
            if (collision == true)
                break;
            end

            %Actually add the new point
            tree.vertex(iter).x = x_temp + x_delta; 
            tree.vertex(iter).y = y_temp + y_delta;
            tree.vertex(iter).dist = step_size;
            tree.vertex(iter).xPrev = tree.vertex(i_temp).x;
            tree.vertex(iter).yPrev = tree.vertex(i_temp).y;
            tree.vertex(iter).ind = iter; 
            tree.vertex(iter).indPrev = i_temp;
            
            i_temp = iter;
            x_temp = tree.vertex(iter).x;
            y_temp = tree.vertex(iter).y;
            plot([tree.vertex(iter).x; tree.vertex(ind).x],[tree.vertex(iter).y; tree.vertex(ind).y], 'r');
            pause(0);
            iter = iter + 1;
            passed_flag = true;
            
            %We found the goal point, so stop
            if(pdist([new_x, new_y; goal_x, goal_y], 'Euclidean') < threshold)
                goal_flag = true;
                break;
            end
            
        end

        %If we never ended up adding a point, increase counter. This means map is too saturated with points and we should try again.
        if(~passed_flag)
            passed_counter = passed_counter + 1;
            if(passed_counter > 100)
                break;
            end
        end
        if(goal_flag)
           break; 
        end
    end

    %We found a path. Time to smooth it.
    if iter < sample_size
        %Initialize path
        path.pos(1).x = goal_x; 
        path.pos(1).y = goal_y;
        path.pos(2).x = tree.vertex(end).x; 
        path.pos(2).y = tree.vertex(end).y;
        pathIndex = tree.vertex(end).indPrev;

        j=0;
        %Backtrack along path to get complete path. We saved the previous index for this purpose.
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
        
        node1_x = path.pos(1).x;
        node1_y = path.pos(1).y;
        node1_pos = 1;
        j=2;
        %With the complete path, time to start cutting out extra points
        while 1
            node2_x = path.pos(j).x;
            node2_y = path.pos(j).y;
            %get max/min x values to check if obstacle is in between points
            if (node1_x > node2_x)
                xmax = node1_x;
                xmin = node2_x;
            else
                xmax = node2_x;
                xmin = node1_x;
            end
            %Now, find line between points to check for obstacles
            %find slope of line
            slope = (node2_y - node1_y)/(node2_x - node1_x);
            %go through obstacles and check for intersection
            intersect = false;
            for i = 1:r
                %check if obstacle xmin is inbetween points
                if (objs(i,1)>=xmin && objs(i,1)<=xmax)
                    %if intersect, change points
                    y = slope*(objs(i,1)-node1_x)+node1_y;
                    if (y >= objs(i,3) && y <= objs(i,4))
                        node1_x = path.pos(j-1).x;
                        node1_y = path.pos(j-1).y;
                        node1_pos = j-1;
                        intersect = true;
                        j = j + 1;
                        break;
                    end
                end
                %check if obstacle xmax is inbetween points
                if (objs(i,2)>=xmin && objs(i,2)<=xmax)
                    %if intersect, change points
                    y = slope*(objs(i,2)-node1_x)+node1_y;
                    if (y >= objs(i,3) && y <= objs(i,4))
                        node1_x = path.pos(j-1).x;
                        node1_y = path.pos(j-1).y;
                        node1_pos = j-1;
                        intersect = true;
                        j = j + 1;
                        break;
                    end
                end
            end
            if (intersect == false)
                %no intersection, remove point
                path.pos(node1_pos+1)=[];
            end
            if j > length(path.pos)
                break;
            end
        end
        
        for j = 2:length(path.pos)
            plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 1);
        %     plot([tree.vertex(i).x; tree.vertex(ind).x],[tree.vertex(i).y; tree.vertex(ind).y], 'r');
        %     pause(0);
        end
    else
        disp('No path found. Increase number of iterations and retry.');
    end
end
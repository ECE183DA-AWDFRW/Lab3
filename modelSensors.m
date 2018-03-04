function [ sen_f, sen_s ] = modelSensors( x_hat, y_hat, theta, dimx, dimy )
    % sen_f is forward sensor, sen_s is side sensor
    % x_hat/y_hat are bot positions, here bottom left-hand corner of box is
    % (0,0)
    % theta is heading from box north
    % dimx/dimy are dimensions of the box
    % Obvious special cases
    if theta == 0 || theta == 360
        sen_f = dimy - y_hat;
        sen_s = dimx - x_hat;
        return
    elseif theta == 90
        sen_f = dimx - x_hat;
        sen_s = y_hat;
        return
    elseif theta == 180
        sen_f = y_hat;
        sen_s = x_hat;
        return
    elseif theta == 270
        sen_f = x_hat;
        sen_s = dimy - y_hat;
        return
    end
    
    % Determine if we need to rotate the box
    if(theta > 0 && theta < 90)
        m1 = tand(90 - theta);
        x_pt = x_hat;
        y_pt = y_hat;
        x_dm = dimx;
        y_dm = dimy;
        
    elseif(theta > 90 && theta < 180)
        m1 = tand(180 - theta);
        x_pt = dimy - y_hat;
        y_pt = x_hat;
        x_dm = dimy;
        y_dm = dimx;
        
    elseif(theta > 180 && theta < 270)
        m1 = tand(270 - theta);
        x_pt = dimx - x_hat;
        y_pt = dimy - y_hat;
        x_dm = dimx;
        y_dm = dimy;
        
    elseif(theta > 270 && theta < 360)
        m1 = tand(360 - theta);
        x_pt = y_hat;
        y_pt = dimx - x_hat;
        x_dm = dimy;
        y_dm = dimx;
    end
    m2 = -1/m1;
    
    % Basically, extend the lines out and found out where they hit the
    % boundaries
    syms x y m;
    func_y = symfun(m*(x-x_pt) + y_pt, [x m]);
    func_x = symfun(((y-y_pt)/m) + x_pt, [y m]);
    
    y1 = eval(func_y(x_dm, m1));
    x1 = eval(func_x(y_dm, m1));    
    y2 = eval(func_y(x_dm, m2));
    x2 = eval(func_x(y_dm, m2));
    
    pt1 = [x1, y_dm];
    pt2 = [x_dm, y1];
    pt3 = [x2, y_dm];
    pt4 = [x_dm, y2];
    
    d1 = pdist([x_pt,y_pt;pt1], 'euclidean');
    d2 = pdist([x_pt,y_pt;pt2], 'euclidean');
    d3 = pdist([x_pt,y_pt;pt3], 'euclidean');
    d4 = pdist([x_pt,y_pt;pt4], 'euclidean');
    
    % Take the lesser of the values, as it hits those edges first
    sen_f = min(d1,d2);
    sen_s = min(d3,d4);

end


function next_state = findNextState(obj, dir, cur_state, d_t )
%Given pwm value, find the next state
%Since we're doing straight lines we can simplify it quite a bit
%    c_t = 90;
%    d_k = 90 - pwm;
%    x = cur_state(1) + (2.0485./pwm.*c_k + 184.09).*d_t.*cosd(cur_state(3));
%    y = cur_state(2) + (2.0485./pwm.*c_k + 184.09).*d_t.*sind(cur_state(3));
%    theta = cur_state(3) + (-1.7956./pwm.*d_k + 157.07).*d_t;

    if(strcmp(dir,'forward')) 
        v = 145; %velocity found experimentally to be 145 mm/s
        x = cur_state(1) + v*d_t*cosd(cur_state(3));
        y = cur_state(2) + v*d_t*sind(cur_state(3));
        theta = cur_state(3);
    elseif(strcmp(dir,'right'))
        w = 150; %angular velocity found to be 150 degrees/s
        x = cur_state(1);
        y = cur_state(2);
        theta = cur_state(3) - w*d_t;
    elseif(strcmp(dir,'left'))
        w = 150; %angular velocity found to be 150 degrees/s 
        x = cur_state(1);
        y = cur_state(2);
        theta = cur_state(3) + w*d_t;
    elseif(strcmp(dir,'reverse'))
        v = 145; %velocity found experimentally to be 145 mm/s
        x = cur_state(1) - v*d_t*cosd(cur_state(3));
        y = cur_state(2) - v*d_t*sind(cur_state(3));
        theta = cur_state(3);
    else
        x = cur_state(1);
        y = cur_state(2);
        theta = cur_state(3);
    end
    if(theta < 0)
        theta = theta + 360;
    elseif(theta > 360)
        theta = theta - 360;
    end
    next_state = [x; y; theta];
end


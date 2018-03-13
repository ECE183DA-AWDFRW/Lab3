function next_state = findNextState(obj, dir, cur_state, d_t )
%Inputs: dir -- direction of movement
%        cur_state -- the current state as [x; y; theta]
%        d_t -- length of movement
%Outputs: next_state -- predicted next state before input into Kalman Filter. Format as [x; y; theta]

%Equivalent to the f(x_k-1, u_k) function in the Kalman filter
%Given pwm value, find the next state
%However, since we're doing straight lines we can simplify it quite a bit.
%We can just set the PWM values to predefined values based off the direction, and use the velocity to solve for next state

%Using the PWM Values, but this is very noisy and inaccurate/inconsistent
%    c_t = 90;
%    d_k = 90 - pwm;
%    x = cur_state(1) + (2.169./pwm.*c_t + 196.988).*d_t.*cosd(cur_state(3));
%    y = cur_state(2) + (2.169./pwm.*c_t + 196.988).*d_t.*sind(cur_state(3));
%    theta = cur_state(3) + (-1.892./pwm.*d_k + 164.286).*d_t;

%dir is direction of movement, d_t is length of time
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


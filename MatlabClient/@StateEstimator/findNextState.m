function next_state = findNextState(obj, pwm, cur_state, d_t )
%Given pwm value, find the next state
%Since we're doing straight lines we can simplify it quite a bit
    c_t = 90;
    d_k = 90 - pwm;
    x = cur_state(1) + (2.169./pwm.*c_t + 196.988).*d_t.*cosd(cur_state(3));
    y = cur_state(2) + (2.169./pwm.*c_t + 196.988).*d_t.*sind(cur_state(3));
    theta = cur_state(3) + (-1.892./pwm.*d_k + 164.286).*d_t;
    next_state = [x; y; theta];
end


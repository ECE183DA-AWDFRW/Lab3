function updateState(obj, pwm, d_t, sen_meas)
    obj.cur_state = obj.extKalmanFilter(pwm, d_t, sen_meas);
    obj.state_array(:,obj.index) = obj.cur_state;
    obj.index = obj.index + 1;
    disp(obj.state_array);
end


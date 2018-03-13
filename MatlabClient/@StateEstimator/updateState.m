function updateState(obj, dir, d_t, sen_meas)
%	Essentially a wrapper function. Estimate the state, and update values
% Inputs: dir -- direction of movement
%		  d_t -- Length of time in that movement
%		  sen_meas -- sensor measurements from Paper Bot
    obj.cur_state = obj.extKalmanFilter(dir, d_t, sen_meas);
    obj.state_array(:,obj.index) = obj.cur_state;
    obj.index = obj.index + 1;
    disp(obj.cur_state);
end


function jacobian_F = createJacobianF(obj, dir, cur_state, d_t)
%Inputs: cur_state -- current state as [x; y; theta]
%        dir -- direction of movement
%		 d_t -- duration of movement
%Outputs: jacobian_H -- jacobian of pre-filtered state estimator at cur_state

%	Create the jacobian for the f function
%    state_b = obj.findNextState(dir, cur_state, d_t);
%    state_dx2 = obj.findNextState(dir, cur_state + [1; 0; 0], d_t);
%    state_dy2 = obj.findNextState(dir, cur_state + [0; 1; 0], d_t);
%    state_da2 = obj.findNextState(dir, cur_state + [0; 0; 1], d_t);
%    disp(state_b);
%    disp(state_dx2);
%    disp(state_da2);
%    state_dx = state_dx2 - state_b;
%    state_dy = state_dy2 - state_b;
%    state_da = state_da2 - state_b;
    
%    jacobian_F = [state_dx, state_dy, state_da];

    jacobian_F = [1, 0, 0; 0, 1, 0; 0, 0, 1];

end


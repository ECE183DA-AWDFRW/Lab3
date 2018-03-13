function jacobian_F = createJacobianF(obj, dir, cur_state, d_t)
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


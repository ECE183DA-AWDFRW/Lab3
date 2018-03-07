function jacobian = createJacobian( prev_state, dimx, dimy )
    %We estimate the jacobian derivatives using our model.
    %Essentially, we take the estimate and two very close points, and find
    %the slope.
    sen_dx1 = modelSensors(prev_state(1),   prev_state(2),   prev_state(3),   dimx, dimy);
    sen_dx2 = modelSensors(prev_state(1)+1, prev_state(2),   prev_state(3),   dimx, dimy);
    sen_dy1 = modelSensors(prev_state(1),   prev_state(2),   prev_state(3),   dimx, dimy);
    sen_dy2 = modelSensors(prev_state(1),   prev_state(2)+1, prev_state(3),   dimx, dimy);
    sen_da1 = modelSensors(prev_state(1),   prev_state(2),   prev_state(3),   dimx, dimy);
    sen_da2 = modelSensors(prev_state(1),   prev_state(2),   prev_state(3)+1, dimx, dimy);
    sen_dx = sen_dx2 - sen_dx1;
    sen_dy = sen_dy2 - sen_dy1;
    sen_da = sen_da2 - sen_da1;
    %theta shouldn't change
    sen_dx(3) = 0;
    sen_dy(3) = 0;
    sen_da(3) = 1;
    jacobian = [sen_dx',sen_dy',sen_da'];
end


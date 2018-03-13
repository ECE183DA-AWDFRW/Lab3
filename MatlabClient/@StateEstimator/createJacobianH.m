function jacobian_H = createJacobianH(obj, cur_state)
    %We estimate the jacobian derivatives using our model.
    %Essentially, we take the estimate and two very close points, and find
    %the slope.
    sen_b = obj.modelSensors(cur_state);
    sen_dx2 = obj.modelSensors(cur_state + [1; 0; 0]);
    sen_dy2 = obj.modelSensors(cur_state + [0; 1; 0]);
    sen_da2 = obj.modelSensors(cur_state + [0; 0; 1]);
    sen_dx = sen_dx2 - sen_b;
    sen_dy = sen_dy2 - sen_b;
    sen_da = sen_da2 - sen_b;
    %theta derivative shouldn't change
    sen_dx(3) = 0;
    sen_dy(3) = 0;
    sen_da(3) = 1;
    jacobian_H = [sen_dx', sen_dy', sen_da'];
    
    
end


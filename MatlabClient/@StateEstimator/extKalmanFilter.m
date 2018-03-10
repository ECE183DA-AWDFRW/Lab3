function next_state_KF = extKalmanFilter( obj, pwm, d_t, sen_meas )
    sen_error = obj.estimateSensors(obj.cur_state)-sen_meas;
    
    %These were found experimentally
    sen_cov = [3.9629, -0.6261 -0.8943; -0.6261, 5.5037 0.8433; -0.8943, 0.8433, 5.7588];
    process_cov = [5, 0, 0; 0, 5, 0; 0, 0, 3];
    
    disp('Finding possible state');
    %Find Possible Next state (x_k|k-1)
    poss_state = obj.findNextState(pwm, obj.cur_state, d_t);
    jacobian_H = obj.createJacobianH(poss_state);
    jacobian_F = obj.createJacobianF(pwm, obj.cur_state, d_t);
    disp(poss_state);
    disp(jacobian_F);
    disp(jacobian_H);
    
    %Update Covariance
    disp('Updating Covariance');
    obj.covariance = jacobian_F*obj.covariance*jacobian_F' + process_cov; 
    disp(obj.covariance);
    
    %Find Kalman Gain
    disp('Finding Kalman Gain');
    G_k = obj.covariance*jacobian_H'*(inv(jacobian_H*obj.covariance*jacobian_H'+sen_cov));
    disp(G_k);
        
    %Take into account Kalman Filter things
    disp('Updating states with Kalman Gain');
    next_state_KF = poss_state + G_k*sen_error;
    obj.covariance = (eye(3) - G_k*sen_cov)*obj.covariance;  
end


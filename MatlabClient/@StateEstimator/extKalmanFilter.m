function next_state_KF = extKalmanFilter( obj, dir, d_t, sen_meas )
    est = obj.estimateSensors(obj.cur_state);
    sen_error = est - sen_meas;
    if ((360 + est(3) - sen_meas(3)) < abs(sen_error(3)))
        sen_error(3) = 360 + est(3) - sen_meas(3);
    elseif ((360 + sen_meas(3) - est(3)) < abs(sen_error(3)))
        sen_error(3) = 360 + sen_meas(3) - est(3);   
    end
    
    %These were found experimentally
    sen_cov = [2.9629, -0.4261 -0.6943; -0.4261, 2.5037 0.5433; -0.6943, 0.5433, 3.7588];
    process_cov = [.5, 0, 0; 0, .5 0; 0, 0, 1];
    
    disp('Finding possible state');
    %Find Possible Next state (x_k|k-1)
    poss_state = obj.findNextState(dir, obj.cur_state, d_t);
    jacobian_H = obj.createJacobianH(poss_state);
    jacobian_F = obj.createJacobianF(dir, obj.cur_state, d_t);
    disp(poss_state);
    
    %Update Covariance
    disp('Updating Covariance');
    obj.covariance = jacobian_F*obj.covariance*jacobian_F' + process_cov; 
    disp(obj.covariance);
    
    %Find Kalman Gain
    disp('Finding Kalman Gain');
    G_k = obj.covariance*jacobian_H'*(inv(jacobian_H*obj.covariance*jacobian_H'+sen_cov));
    disp(G_k);
        
    %Take into account Kalman Filter Gain
    disp('Updating states with Kalman Gain');
    next_state_KF = poss_state + G_k*sen_error;
    if(next_state_KF(3) < 0)
       next_state_KF(3) = next_state_KF(3) + 360;
    elseif(next_state_KF(3) > 360)
       next_state_KF(3) = next_state_KF(3) - 360;
    end
    obj.covariance = (eye(3) - G_k*sen_cov)*obj.covariance;  
end


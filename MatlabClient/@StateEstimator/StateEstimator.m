classdef StateEstimator < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        dmx     %x dimension of the box
        dmy     %y dimension of the box
        a_offset %angle of the box from magnetic north
        
        initial_state %initial state of the paperbot
        state_array   %stores previous estimated states of the paperbot
        cur_state     %the current estimated state
        covariance    %our covariance matrix
        index             %index of state array
        
    end
    
    methods
        function obj = StateEstimator(dmx, dmy, a_offset, initial_state)
            obj.dmx = dmx;
            obj.dmy = dmy;
            obj.a_offset = a_offset;
            obj.initial_state = initial_state;
            obj.index = 2;
            obj.state_array(:,1) = initial_state;
            obj.covariance = eye(3)./10;
            obj.cur_state = initial_state;
        end
       
        jacobian_H = createJacobianH(obj, cur_state)
        jacobian_F = createJacobianF(obj, pwm, cur_state, d_t)
        sensor_estimate = estimateSensors(obj, cur_state)
        sensor_model = modelSensors(obj, cur_state)
        next_state = findNextState(obj, pwm, cur_state, d_t )
        next_state_KF = extKalmanFilter( obj, pwm, d_t, sen_meas )
        updateState(obj, pwm, d_t, sen_meas)
        
    end
    
end


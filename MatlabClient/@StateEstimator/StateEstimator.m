classdef StateEstimator
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        dmx     %x dimension of the box
        dmy     %y dimension of the box
        a_offset %angle of the box from magnetic north
        
        initial_state %initial state of the paperbot
        state_array   %stores previous estimated states of the paperbot
        
    end
    
    methods
        function obj = StateEstimator(dmx, dmy, a_offset, initial_state)
            obj.dmx = dmx;
            obj.dmy = dmy;
            obj.a_offset = a_offset;
            obj.initial_state = initial_state;
        end
        
        jacobian = createJacobian(obj, prev_state)
        sensor_estimate = estimateSensors(obj, cur_state)
        sensor_model = modelSensors(obj, cur_state)
    end
    
end


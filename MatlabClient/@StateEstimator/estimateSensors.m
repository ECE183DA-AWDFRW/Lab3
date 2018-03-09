%These were discovered experimentally
%We modelled the noise as an offset + gaussian random number
function sensor_estimate = estimateSensors(obj, cur_state)
%model the sensors, and add noise
    sen_offset = [17; 5; 30];
    %sen_noise = normrnd([0; 0; 0],[2.18; 2.18; 3]);
    
    sen = obj.modelSensors(cur_state);
    sensor_estimate = [sen(1); sen(2); cur_state(3)] + sen_offset;
end


function sensor_estimate = estimateSensors(obj, cur_state)
%Inputs: cur_state -- current state as [x; y; theta]
%Outputs: sensor_estimate -- sensor estimate values

%We modelled the noise as an offset + gaussian random number
%These were discovered experimentally
    sen_offset = [60; 50; 0];
    %sen_noise = normrnd([0; 0; 0],[2.18; 2.18; 3]);
    
    %convert theta to heading
    heading = -cur_state(3) + 90;
    if(heading < 0)
        heading = heading + 360;
    end
   
    sen = obj.modelSensors(cur_state);
    sensor_estimate = [sen(1); sen(2); heading] - sen_offset;
end


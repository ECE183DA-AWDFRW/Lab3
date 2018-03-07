%These were discovered experimentally
%We modelled the noise as an offset + gaussian random number
function sensor_estimate = estimateSensors(x_hat, y_hat, theta, dimx, dimy)
%model the sensors, and add noise
    sen_offset = [17; 5; 30];
    %sen_noise = normrnd([0; 0; 0],[2.18; 2.18; 3]);
    
    sen = modelSensors(x_hat, y_hat, theta, dimx, dimy);
    sensor_estimate = [sen(1); sen(2); theta] + sen_offset;
end


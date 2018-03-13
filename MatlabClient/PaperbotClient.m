classdef PaperbotClient < WebSocketClient
    %CLIENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        sensor_data;
        state_estimator;
        d_t = 0;  %in milliseconds
        dir = 'stop';
        path;

    end
    
    methods
        function obj = PaperbotClient(x_dm, y_dm, a_offset, initial_state, varargin)
            %Constructor
            obj@WebSocketClient(varargin{:});
            obj.state_estimator = StateEstimator(x_dm, y_dm, a_offset, initial_state);
        end
                %Send PWM values to the paperbot
        function drive(obj, pwm1, pwm2, dir)
            obj.dir = dir;
            s = strcat('$', sprintf('%03d',pwm1));
            s = strcat(s, sprintf(' %03d',pwm2));
            obj.send(s);
            pause(obj.d_t);
            obj.send('#S');
            
        end
        
        function getPath(obj, goal_x, goal_y)
            objs = [150 200 0 230];
            [~, path1] = RRT(10000, obj.state_estimator.dmx, obj.state_estimator.dmy, ...
                                     obj.state_estimator.cur_state(1), obj.state_estimator.cur_state(2), ...
                                     goal_x, goal_y, ...
                                     objs, .5, 1);
                                 
            obj.path = path1;
        end
        
        
        
        function drivePath(obj)
            i = length(obj.path.pos)-1;
            while i > 0
                goal_x = obj.path.pos(i).x;
                goal_y = obj.path.pos(i).y;
                disp('Goal:');
                disp(goal_x);
                disp(goal_y);
                x = obj.state_estimator.cur_state(1);
                y = obj.state_estimator.cur_state(2);
                theta = obj.state_estimator.cur_state(3);
                
                %Align to appropriate angle
                goal_theta = atan2d(goal_y - y,goal_x - x);
                if(goal_theta > theta)
                    if(goal_theta-theta > 180)
                        %goal is actually to right of current theta
                        turn_angle = 360 - goal_theta + theta;
                        obj.dir = 'right';
                        
                    else
                        turn_angle = goal_theta - theta;
                        obj.dir = 'left';
                    end
                    
                elseif(goal_theta < theta)
                    if(theta - goal_theta > 180)
                        turn_angle = 360 - theta + goal_theta;
                        obj.dir = 'left';
                    else
                        turn_angle = theta - goal_theta;
                        obj.dir = 'right';
                    end
                    
                elseif(goal_theta == theta)
                    turn_angle = 0;
                    obj.dir = 'stop';
                end
                
                if(strcmp(obj.dir, 'right'))
                    obj.d_t = turn_angle/150;
                    disp(obj.d_t);
                    obj.drive(180, 180, 'right');
                elseif(strcmp(obj.dir, 'left'))
                    obj.d_t = turn_angle/150;
                    disp(obj.d_t);
                    obj.drive(0, 0, 'left');
                end
                pause;
                %Drive Forward
                
                d = pdist([goal_x, goal_y; x,y]);
                obj.d_t = d/145;
                obj.drive(180, 0, 'forward');
                pause;
                i = i-1;
            end
            
        end
        
        
    end
    
    methods (Access = protected)
        function onTextMessage(obj,message)
            % This function simply displays the message received
            fprintf('Message received:\n%s\n',message);
            temp_message = strsplit(message);
            obj.sensor_data = str2double(temp_message);
            %sensor_data will be front_sensor, side_sensor, heading
            
            if(~isnan(obj.sensor_data))
                % Recieved measurement data! Estimate next state, etc.
                obj.state_estimator.updateState(obj.dir, obj.d_t, obj.sensor_data');
            end
        end
        
        function onOpen(obj,message)
            % This function simply displays the message received
            fprintf('%s\n',message);
        end
        
        function onBinaryMessage(obj,bytearray)
            % This function simply displays the message received
            fprintf('Binary message received:\n');
            fprintf('Array length: %d\n',length(bytearray));
        end
        
        function onError(obj,message)
            % This function simply displays the message received
            fprintf('Error: %s\n',message);
        end
        
        function onClose(obj,message)
            % This function simply displays the message received
            fprintf('%s\n',message);
        end
    end
end


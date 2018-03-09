classdef PaperbotClient < WebSocketClient
    %CLIENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        sensor_data;
        state_estimator;
        cur_state;
    end
    
    methods
        function obj = PaperbotClient(x_dm, y_dm, a_offset, initial_state, varargin)
            %Constructor
            obj@WebSocketClient(varargin{:});
            obj.state_estimator = StateEstimator(x_dm, y_dm, a_offset, initial_state);
        end
    end
    
    methods (Access = protected)
        
        
        
        function onTextMessage(obj,message)
            % This function simply displays the message received
            fprintf('Message received:\n%s\n',message);
            temp_message = strsplit(message);
            obj.sensor_data = str2double(temp_message);
            
            if(~isnan(obj.sensor_data))
                % Recieved measurement data, so find difference
                obj.state_estimator.estimateSensors(obj.cur_state);
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


%client = PaperbotClient(390, 350, 0, [100; 100; 90], 'ws://192.168.4.1:81');
%pause;
%client.getPath(300, 130);
%pause;
%client.drivePath();

[~, path1] = RRT(10000, 390, 350, ...
                         100, 100, ...
                         300, 130, ...
                         [150 200 0 230], 1, 1);
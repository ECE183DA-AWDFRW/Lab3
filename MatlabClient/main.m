client = PaperbotClient(390, 350, 0, [100; 100; 90], 'ws://192.168.4.1:81');
pause;
client.getPath(300, 130);
pause;
client.drivePath();

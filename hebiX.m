


thetaAlpha = .1;

g = HebiLookup.newConnectedGroupFromName('X5-1', 'X-00037')


cmd = CommandStruct;

cmd.velocity = 1;


    fbk = g.getNextFeedback();
    
    x = fbk.accelX;
    y = fbk.accelY;
    
    theta = atan2(-x,y)
thetaFiltered = theta;

% thetaGyro = theta;
tic;
while true
    dt = toc;
    tic;
    fbk = g.getNextFeedback();
    
    x = fbk.accelX;
    y = fbk.accelY;
    
    theta = atan2(-x,y);
    
    thetaFiltered = (1-thetaAlpha)*thetaFiltered + thetaAlpha * theta;
    thetaFiltered = thetaFiltered - dt*fbk.gyroZ;
    
%     thetaGyro = thetaGyro - fbk.gyroZ
    gyroz = fbk.gyroZ

    cmd.position = thetaFiltered;
    cmd.velocity = -fbk.gyroZ;
%     cmd.velocity = fbk.accelX/10;
    
    g.set(cmd)
    
end
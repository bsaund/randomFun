


thetaAlpha = .1;

left = HebiLookup.newConnectedGroupFromName('X5-1', 'X-00037')
right = HebiLookup.newConnectedGroupFromName('X5-1', 'X-00033')

leftCmd = CommandStruct;
rightCmd = CommandStruct;
while true
    
    rightFbk = right.getNextFeedback;
    leftFbk = left.getNextFeedback;
    
    %     tq = rightFbk.torque
    
    
    x = leftFbk.accelX;
    y = leftFbk.accelY;
    z = leftFbk.accelZ;
    
    theta = atan2(x,z)
    
    
    
    rightCmd.torque = 0;
%     rightCmd.position = -leftFbk.position;
    
%     leftCmd.torque = 0;
    leftCmd.position = -rightFbk.position;
    
    left.set(leftCmd);
    right.set(rightCmd);
end
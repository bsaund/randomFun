
function segway

    global L M dt
    close all
    L = 1.6; %meters
    M = 2.2; %kg
    dt = 0.03; %s



    Q = diag([10 1 10 1]);
    [A, B] = linearizedDynamics;
    K = dlqr(A,B,Q,.2);
    radius = 0.04;



    thetaOffset = 0.02;
    gyroOffset = .03;

    thetaAlpha = .1;

    left = HebiLookup.newConnectedGroupFromName('X5-1', 'X-00037');
    right = HebiLookup.newConnectedGroupFromName('X5-1', 'X-00033');
    top = HebiLookup.newConnectedGroupFromName('X5-4', 'X-00036');

    gains = left.getGains();
    gains.velocityKp = 0.0025;
    left.set('gains', gains);
    right.set('gains', gains);


    cmdLeft = CommandStruct;
    cmdRight = CommandStruct;

    cmdLeft.velocity = 1;


    fbk = top.getNextFeedback();

    xAccel = fbk.accelX;
    yAccel = fbk.accelY;

    %     theta = atan2(y, -x)
    theta = -atan2(-yAccel,xAccel) - thetaOffset;
    thetaFiltered = theta;

    xFiltered = 9.8;
    yFiltered = 0;


    leftPosOffset = left.getNextFeedback.position;


    % thetaGyro = theta;
    tic;
    while true
        
        dt = toc;
        tic;
        fbk = top.getNextFeedback();
        
        xAccel = fbk.accelX;
        yAccel = fbk.accelY;
        
        yFiltered = yFiltered*.9 + yAccel*.1;
        xFiltered = xFiltered*.9 + xAccel*.1;
        
        %     theta = -atan2(-y,x) - thetaOffset;
        
        gyroZ = fbk.gyroZ - gyroOffset;
        
        %     thetaFiltered = (1-thetaAlpha)*thetaFiltered + thetaAlpha * theta;
        %     thetaFiltered = thetaFiltered - dt*fbk.gyroZ
        th2 = -atan2(-yFiltered, xFiltered) - thetaOffset;
        thetaFiltered = thetaFiltered*.99 + th2*.01 - dt*(gyroZ);
        
        if(abs(thetaFiltered) > .3)
            vel = 0;
            continue
        end
        
        leftFbk = left.getNextFeedback;
        rightFbk = right.getNextFeedback;
        
        x = -(leftFbk.position - leftPosOffset) * radius;
        xdot = -leftFbk.velocity * radius;
        %     thetaGyro = thetaGyro - fbk.gyroZ
        
        state = [x; xdot; thetaFiltered; -gyroZ]
        
        accel = -K*state/L;
        
        %     accel = 30*thetaFiltered;
        
        %     cmd.position = thetaFiltered;
        %     cmd.velocity = -fbk.gyroZ;
        %     cmd.velocity = fbk.accelX/10;
        %     cmd.torque = thetaFiltered
        

        
        if(isnan(left.getNextFeedback.velocityCmd))
            v = 0;
        else
            v = left.getNextFeedback.velocityCmd - accel;
        end
        %     v = v*.9;
        
        cmdLeft.velocity = v;
        %     vel = cmdLeft.velocity
        
        cmdRight.velocity = -cmdLeft.velocity;
        %     
        left.set(cmdLeft)
        right.set(cmdRight)
        
    end
end

function [A, B] = linearizedDynamics
    global L dt
    A = [1 dt 0 0;
         0 1 0 0;
         0 0 1 dt;
         0 0 dt*9.8/(1.5*L) 1];

    B = [0; dt; 0; -dt/L];
end

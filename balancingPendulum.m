function balancingPendulum()
    global L M dt
    close all
    L = 1.6; %meters
    M = 2.2; %kg
    dt = 0.03; %s
    state = [0.1; 0.1; 0.1; 0];
    plotState(state)


    Q = diag([1 0 1 0]);
    [A, B] = linearizedDynamics;
    K = dlqr(A,B,Q,1);


    tic
    while true
        u = -K*state;
        state = forward(state, u);
        plotState(state);
    end

end

function h = plotState(state, h)
    global L
    cellState = num2cell(state);
    [x, xdot, theta, thetadot] = cellState{:};
    if(nargin == 2)
        delete(h);
        end1

        h = plot([x, x+L*sin(theta)], [0, L*cos(theta)]);
        axis([-1,1,0,2]);
        drawnow
    end


    function [A, B] = linearizedDynamics
        global L dt
        A = [1 dt 0 0;
             0 1 0 0;
             0 0 1 dt;
             0 0 dt*9.8/L 1];

        B = [0; dt; 0; -dt/L];
    end

    function state = forward(state, input)
        [A, B] = linearizedDynamics();
        state = A*state + B*input;
    end
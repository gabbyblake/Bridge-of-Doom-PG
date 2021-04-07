function starterCodeForBridgeOfDoomQEA2021()
% Insert any setup code you want to run here
syms u;

    d = 0.235;
    beta = 0.25;
    x = 4.*(0.3960*cos(2.65*((beta * u) + 1.4)));
    y = 4.*-0.99*sin((beta * u) + 1.4);
    z = 0 * (beta * u);
    r = [x; y; z];
    dr = diff(r, u);
    lin_speed = norm(dr);
    T_hat = dr / lin_speed;
    dT = diff(T_hat);
    ang_vel = cross(T_hat, dT);
    V_L = lin_speed - (ang_vel(3,:) .* (d /2));
    V_R = lin_speed + (ang_vel(3,:) .* (d /2));
    speeds = [V_L; V_R];

pub = rospublisher('raw_vel');
message = rosmessage(pub);

% stop the robot if it's going right now
stopMsg = rosmessage(pub);
stopMsg.Data = [0 0];
send(pub, stopMsg);

bridgeStart = double(subs(r,u,0));
startingThat = double(subs(T_hat,u,0));
placeNeato(bridgeStart(1),  bridgeStart(2), startingThat(1), startingThat(2));

% wait a bit for robot to fall onto the bridge
pause(5);

rostic

while 1
    u_now = rostoc;
    V_L_now = double(subs(V_L, u, u_now));
    V_R_now = double(subs(V_R, u, u_now));
    
    if u_now > (3.05 / beta)
        stopMsg.Data = [0 0];
        send(pub, stopMsg)
        break
    end
    
    
    message.Data = [V_L_now V_R_now];
    send(pub, message)
    
end


end

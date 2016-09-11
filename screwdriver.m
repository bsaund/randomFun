g = HebiLookup.newGroupFromNames('Spare', 'wheel_01')

cmd = CommandStruct



for i=-1:.01:1
    disp(i)
    cmd.torque = i;
    g.set(cmd);
    pause(.1)
end
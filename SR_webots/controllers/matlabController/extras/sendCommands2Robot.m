function sendCommands2Robot(robot, q)
    for i=1:numel(robot.motor)
        wb_motor_set_position(robot.motor(i), q(i));
    end
end
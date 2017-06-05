function robot=initializeSRLink(TIME_STEP)
    robot.TIME_STEP=TIME_STEP;

    %enable motors
    robot.motorNames={'motor_1', 'motor_2', 'motor_3', 'motor_4','motor_5', 'motor_6', 'motor_7', 'motor_8', 'motor_9', 'motor_10', ...
                    'motor_leg_1', 'motor_leg_2', 'motor_leg_3', 'motor_leg_4'
                     };
                 
    robot.positionSensorsNames={'position_sensor_1', 'position_sensor_2', 'position_sensor_3', 'position_sensor_4','position_sensor_5', 'position_sensor_6', 'position_sensor_7', 'position_sensor_8', 'position_sensor_9', 'position_sensor_10', ...
                    'position_sensor_leg_1', 'position_sensor_leg_2', 'position_sensor_leg_3', 'position_sensor_leg_4'
                     };
    for i=1:numel(robot.motorNames);
        robot.motor(i)=wb_robot_get_device(robot.motorNames{i});
        robot.positionSensor(i)=wb_robot_get_device(robot.positionSensorsNames{i});
        wb_motor_enable_force_feedback(robot.motor(i), TIME_STEP);
        wb_position_sensor_enable(robot.positionSensor(i), TIME_STEP);
    end
    
    
    robot.gps=wb_robot_get_device('fgirdle_gps');
    wb_gps_enable(robot.gps, TIME_STEP);



end
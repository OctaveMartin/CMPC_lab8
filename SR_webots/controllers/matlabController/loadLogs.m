
% Read logged values from text files. Expand the list if needed. 
gps=dlmread('./logs/gps_log.txt');
t=dlmread('./logs/time_log.txt');
q=dlmread('./logs/angles_reference_log.txt');
positionSensor=dlmread('./logs/angles_feedback_log.txt');
torques=dlmread('./logs/torque_log.txt');







% EXAMPLE HOW TO PLOT REFERENCE ANGLES Q (spine only)
% add offset in y axis for each joint reference to have visual clarity
offsets=linspace(0, -5, 10); 
q_offset=q(:, 1:10) + repmat(offsets, size(q, 1), 1);
plot(t, q_offset)
text(1, 0.3, 'head')
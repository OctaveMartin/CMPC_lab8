
function do_we_stop=saveGridSearch(filename, GS, gps, f, f_limb, R, R_limb, t, q)
%%
%     input:
%         filename - name of the .mat file where results will be stored
%         GS - the structure containing parameters and metrics
%         gps - GPS values from the current run (X, Y, Z coordinates)
%         motor_position - positions of all motors during the simulation
%         motor_torque - torques of all motors during the simulation
%         t - time vector of the current simulation

%     output:
%         do_we_stop - flag to indicate if grid search is finished



    % An example how to formulate metrics. Use GS.iteration to index
    idx = find(t > 5);
    GS.distance(GS.iteration) = norm(gps([1 3], end) - gps([1 3], idx(1)));
    GS.speed(GS.iteration) = GS.distance(GS.iteration)/(t(end)-t(idx(1)));

    GS.pos{GS.iteration} = gps;
    GS.q{GS.iteration} = q;
    GS.f{GS.iteration} = f;
    GS.f_limb{GS.iteration} = f_limb;
    GS.R{GS.iteration} = R;
    GS.R_limb{GS.iteration} = R_limb;


    % Saving current GS into .mat file to preserve data between different
    % iterations
    save(filename, 'GS');

    if GS.iteration >= GS.maxIter
        wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_PAUSE);
        do_we_stop=1;
    else 
        wb_supervisor_simulation_revert;
        do_we_stop=0;
    end


end
function [var1, var2, GS]=initializeGridSearch(filename, var1_vector, var2_vector)
%%
%     input:
%         filename - name of the .mat file where results will be stored
%         var1_vector - first search parameter    (length N1)
%         var2_vector - second search parameter   (length N2)

%     output:
%         var1 - current value of the first search parameter
%         var2 - current value of the second search parameter
%         GS - the structure containing parameters and metrics

    % create a grid of search parameters - all possible combinations 
    % resulting parameter matrix has N1*N2 elements
    [var1_grid,var2_grid] = meshgrid(var1_vector,var2_vector);

    
    if exist([filename '.mat'])==0    % first iteration of the search
        GS.var1_grid=var1_grid;
        GS.var2_grid=var2_grid;
        GS.iteration=1;
        GS.maxIter=numel(var1_vector)*numel(var2_vector);
    else
        load(filename);
        GS.iteration=GS.iteration+1;
    end

    var1=GS.var1_grid(GS.iteration);
    var2=GS.var2_grid(GS.iteration);

    % if we checked all combinations of parameters, pause the simulation
    if GS.iteration > GS.maxIter
        wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_PAUSE);
    end

end
function [f_out, f_limb_out, R_out, R_limb_out, is_swimming]=limbCPGsaturation(f, f_limb, R, R_limb, gps)
    %%
    %     input:
    %         f - intrinsic frequency of spine oscillators in Hz
    %         f_limb - intrinsic frequency of limb oscillators in Hz
    %         R - nominal amplitude of spine oscillators
    %         R_limb - nominal amplitude of limb oscillators
    %         gps_x - x coordinate of the robot at the current time step
    %     output:
    %         f_out - intrinsic frequency of spine oscillators in Hz after saturation 
    %         f_limb_out - intrinsic frequency of limb oscillators in Hz after saturation 
    %         R_out - nominal amplitude of spine oscillators after saturation 
    %         R_limb_out - nominal amplitude of limb oscillators after saturation 
    %%
    
    persistent is_init...
               is_wimming...
               x_swimming   x_transition ...
               f_spine_walk f_spine_swim...
               R_spine_walk R_spine_swim...
               f_limb_walk  f_limb_swim f_limb_max...
               R_limb_walk  R_limb_swim R_limb_max;
           
    if isempty(is_init)
        is_init=true;
        
        if gps > x_swimming
            is_swimming = true;
        else
            is_swimming = false;
        end
        
        x_swimming = 0.575;     %0.98 %0.975
        x_transition = 0.175;   %0.95 %0.875
        
        f_spine_walk = 1;       %1
        f_spine_swim = 1.5;     %1.5
        
        R_spine_walk = 0.37;    %0.37
        R_spine_swim = 0.42;    %0.42
        
        f_limb_walk = 1;
        f_limb_swim = 0;
        f_limb_max = 1.2;
        
        R_limb_walk = 0.3;
        R_limb_swim = 0;
        R_limb_max = 0.3;
        
        figure();
        subplot(2,1,1);
        hold on;
        plot([x_transition-0.05 x_transition-0.0001 x_transition x_swimming x_swimming+0.0001 x_swimming+0.05],[f_spine_walk f_spine_walk f_spine_walk f_spine_swim f_spine_swim f_spine_swim]);
        plot([x_transition-0.05 x_transition-0.0001 x_transition x_swimming x_swimming+0.0001 x_swimming+0.05],[f_limb_walk f_limb_walk f_limb_walk f_limb_max f_limb_swim f_limb_swim]);
        subplot(2,1,2);
        hold on;
        plot([x_transition-0.05 x_transition-0.0001 x_transition x_swimming x_swimming+0.0001 x_swimming+0.05],[R_spine_walk R_spine_walk R_spine_walk R_spine_swim R_spine_swim R_spine_swim]);
        plot([x_transition-0.05 x_transition-0.0001 x_transition x_swimming x_swimming+0.0001 x_swimming+0.05],[R_limb_walk R_limb_walk R_limb_walk R_limb_max R_limb_swim R_limb_swim]);
    end
    
    if gps > x_swimming
        is_swimming=true;
        
        f_out = f_spine_swim;
        f_limb_out = f_limb_swim;
        R_out = R_spine_swim;
        R_limb_out = R_limb_swim;
        
    elseif gps < x_transition
        is_swimming=false;
        
        f_out = f_spine_walk;
        f_limb_out = f_limb_walk;
        R_out = R_spine_walk;
        R_limb_out = R_limb_walk;
        
    else
        is_swimming=false;
        
        Dx = x_swimming-x_transition;
        dx = gps-x_transition;
        
        f_out = f_spine_walk + dx*(f_spine_swim-f_spine_walk)/Dx;
        f_limb_out = f_limb_walk + dx*(f_limb_max-f_limb_walk)/Dx;
        R_out = R_spine_walk + dx*(R_spine_swim-R_spine_walk)/Dx;
        R_limb_out = R_limb_walk + dx*(R_limb_max-R_limb_walk)/Dx;
        
        %f_out = (10*f + f_out)/11;
        %f_limb_out = (10*f_limb + f_limb_out)/11;
        %R_out = (10*R + R_out)/11;
        %R_limb_out = (10*R_limb + R_limb_out)/11;
        
    end

end
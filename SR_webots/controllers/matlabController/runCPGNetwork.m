function [theta_out, r_out]=runCPGNetwork(f, f_limb, phi_total, R, R_limb, limb_spine_offset, dt)
%%
%     input:
%         f - intrinsic frequency of spine oscillators in Hz
%         f_limb - intrinsic frequency of limb oscillators in Hz
%         R - nominal amplitude of spine oscillators
%         R_limb - nominal amplitude of limb oscillators
%         limb_spine_offset - phase offset between the limbs and the spine
%         dt - basic time step
%     output:
%         theta_out - CPG phase of size (24 x 1)
%         r_out - CPG radius of size (24 x 1)
%
%%  Initialization
    % define here static variables (keep their value over multiple calls of the
    % function). You can also put here your coupling weights etc...
    persistent is_init r theta t w phi;
    if isempty(is_init)
        is_init=true;
        theta=rand(24,1);
        r=zeros(24,1);
        % initialization of static/persistent variables (happens only
        % during first call)
        t=0;
        
        %-----------------------------------------------------------------%
        %                 additional persistent variables                 %
        %-----------------------------------------------------------------%
        
        % weight matrix
        m1 = 10*(diag([ones(9,1);0;ones(9,1)],1)+diag([ones(9,1);0;ones(9,1)],-1)+diag(ones(10,1),10)+diag(ones(10,1),-10));
        m2 = zeros(20,4);
        m3 = [repmat([1;0;0;0],1,5) repmat([0;1;0;0],1,5) repmat([0;0;1;0],1,5) repmat([0;0;0;1],1,5)];
        m4 = [0 1 1 0;1 0 0 1;1 0 0 1;0 1 1 0];
        w = [[m1 m2];[30*m3 10*m4]]';
        
        %{
        figure();
        imagesc(w);
        colorbar;
        xlabel('j');
        ylabel('i');
        %}
        
        % phase lags matrix
        m1 = (phi_total/10)*(diag([ones(9,1);0;ones(9,1)],1)-diag([ones(9,1);0;ones(9,1)],-1))...
             +pi*diag(ones(10,1),10)...
             +pi*diag(ones(10,1),-10);
        phi = [[m1 m2];[limb_spine_offset*m3 pi*m4]]';
        
        %{
        figure();
        imagesc(phi);
        colorbar;
        xlabel('j');
        ylabel('i');
        figure();
        %}
    end   
    
    t=t+dt;

    
    
    %% HERE IMPLEMENT DOUBLE CHAIN OF OSCILLATORS REPRESENTING THE SPINAL CORD OF THE ROBOT
    dtheta = 2*pi*[f*ones(20,1);f_limb*ones(4,1)] + sum(repmat(r',24,1).*w.*sin(repmat(theta,1,24)'-repmat(theta,1,24)-phi),2);
    
    a = 1;  % convergence coefficient not provided in the code
    dr = a*([R*ones(20,1);R_limb*ones(4,1)]-r);


    %% HERE IMPLEMENT EULER INTEGRATION
    theta = theta + dtheta*dt;
    r = r + dr*dt;

    
    %% Assign output
    theta_out=theta;
    r_out=r;

end


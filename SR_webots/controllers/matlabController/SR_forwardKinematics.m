function p=SR2_forwardKinematics(q,h)

    persistent is_init hs hl counter;
    if isempty(is_init)
        is_init=true;
        counter=0;
        hs=plot3(0,0,0,'-o'); hold on;
        for k=1:4
            hl(k)=plot3(0,0,0, '-or');
        end
        axis([-1 1 -1 1 -1 1]);
        
    end 
    counter=counter+1;
    p=zeros(3,16);

    % forward kinematics
    link=0.096;
    % legs
    yoff=0.09;
    zoff=-0.09;

    
    
    p(1:3,2)=[-link, 0, 0]';
    phi=0;
    for k=1:10
        phi=phi+q(k);
        p(1:2,k+2)=p(1:2,k+1)+[-link*cos(phi); -link*sin(phi)];
    end
    fgird=(p(:,2)+p(:,3))/2;
    hgird=(p(:,7)+p(:,8))/2;
    
    b2=atan2(hgird(2)-fgird(2),-hgird(1)+fgird(1));
    for k=1:size(p,2)
        p(1:2,k)=[cos(b2) -sin(b2); sin(b2) cos(b2)]*p(1:2,k);
    end
    % center
    p(1,:)=p(1,:)+0.5;
    p(2,:)=p(2,:)-mean(p(2,:));
        
        
    % legs
    p(:,13)=[0, yoff, zoff]';    
    p(:,14)=[0, -yoff, zoff]';
    p(:,15)=[0, yoff, zoff]';
    p(:,16)=[0, -yoff, zoff]';
    for k=1:4
        p([1 3],12+k)=[cos(q(10+k)) -sin(q(10+k)); sin(q(10+k)) cos(q(10+k))]*p([1 3],12+k);
    end
    fgird=(p(:,2)+p(:,3))/2;    fgird_phi=-atan2(p(2,3)-p(2,2), -p(1,3)+p(1,2));
    hgird=(p(:,7)+p(:,8))/2;    hgird_phi=-atan2(p(2,8)-p(2,7), -p(1,8)+p(1,7));
    p(1:2,13)=[cos(fgird_phi) -sin(fgird_phi); sin(fgird_phi) cos(fgird_phi)]*p(1:2,13);
    p(1:2,14)=[cos(fgird_phi) -sin(fgird_phi); sin(fgird_phi) cos(fgird_phi)]*p(1:2,14);
    p(1:2,15)=[cos(hgird_phi) -sin(hgird_phi); sin(hgird_phi) cos(hgird_phi)]*p(1:2,15);
    p(1:2,16)=[cos(hgird_phi) -sin(hgird_phi); sin(hgird_phi) cos(hgird_phi)]*p(1:2,16);
    
    p(:,13)=p(:,13)+fgird;
    p(:,14)=p(:,14)+fgird;
    p(:,15)=p(:,15)+hgird;
    p(:,16)=p(:,16)+hgird;
    
    % plot
    set(hs, 'Xdata', p(1,1:12), 'Ydata', p(2,1:12), 'Zdata', p(3,1:12));
    for k=1:2
        set(hl(k), 'Xdata', [fgird(1) p(1,12+k)], 'Ydata', [fgird(2) p(2,12+k)], 'Zdata', [fgird(3) p(3,12+k)]);
    end
    for k=3:4
        set(hl(k), 'Xdata', [hgird(1) p(1,12+k)], 'Ydata', [hgird(2) p(2,12+k)], 'Zdata', [hgird(3) p(3,12+k)]);
    end
    if mod(counter, 1)==0
        drawnow; 
    end
    
    pause(20e-3);
    

end














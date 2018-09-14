classdef Primitive
    %PRIMITIVE Contains functionality for storing and mirroring primitives
  %   ACADO Toolkit is free software; you can redistribute it and/or
 %    modify it under the terms of the GNU Lesser General Public
 %    License as published by the Free Software Foundation; either
 %    version 3 of the License, or (at your option) any later version.
    properties
        primId
        fromHeadingId
        toHeadingId
        direction
        cost
        traj
    end
    
    methods
        function obj = Primitive(primId, fromHeadingId, toHeadingId, cost, traj, dir)
            obj.primId = primId;
            obj.fromHeadingId = fromHeadingId;
            obj.toHeadingId = toHeadingId;
            obj.direction = dir;
            obj.cost = cost;
            obj.traj = traj;
            
        end
        
        function newPrims = mirror_primitive(obj)
            fromHeading = obj.traj(3,1);
            alpha = obj.traj(4,:);
            if(sum(abs(alpha)) < 1e-3 && abs(fromHeading-pi/4) < 1e-3 ) % straight, heading pi/4
                m2p = obj.mirrorToSecond();
                m3p = obj.mirrorToThird();
                m4p = obj.mirrorToFourth();
                newPrims = [obj, m2p, m3p, m4p];
            elseif( sum(abs(alpha)) < 1e-3 && abs(fromHeading) < 1e-3 ) % straight, heading zero
                m1p = obj.mirrorInFirst();
                m31p = obj.mirrorToSecond();
                m32p = m1p.mirrorToThird();
                newPrims = [obj, m1p, m31p, m32p];
            else
                m1_p = obj.mirrorInFirst();
                m21_p = obj.mirrorToSecond();
                m22_p = m1_p.mirrorToSecond();
                m31_p = obj.mirrorToThird();
                m32_p = m1_p.mirrorToThird();
                m41_p = obj.mirrorToFourth();
                m42_p = m1_p.mirrorToFourth();
                
                newPrims = [obj, m1_p, m21_p, m22_p, m31_p, m32_p, m41_p, m42_p];
            end
            
        end
        
        function new_prim = mirrorInFirst(obj)
           m_Id = obj.primId;
           m_fromHeadingId = mod(4 - obj.fromHeadingId, 16); 
           m_toHeadingId = mod(4 - obj.toHeadingId, 16); 
           m_dir = obj.direction;
           m_cost = obj.cost;    
           [nStates, N] = size(obj.traj);
           m_traj = zeros(nStates, N);
           m_traj(1,:) = obj.traj(2,:); %x = y
           m_traj(2,:) = obj.traj(1,:); %y = x
           m_traj(3,:) = constrain_angle(-obj.traj(3,:)+pi/2); %theta
           m_traj(4,:) = -obj.traj(4,:); %alpha
           
           new_prim = Primitive(m_Id, m_fromHeadingId, m_toHeadingId, m_cost, m_traj, m_dir);        
           
        end
        
        function new_prim = mirrorToSecond(obj)
           m_Id = obj.primId;
           m_fromHeadingId = mod(8 - obj.fromHeadingId, 16); 
           m_toHeadingId = mod(8 - obj.toHeadingId, 16); 
           m_dir = obj.direction;
           m_cost = obj.cost;    
           [nStates, N] = size(obj.traj);
           m_traj = zeros(nStates, N);
           m_traj(1,:) = -obj.traj(1,:); %x
           m_traj(2,:) = obj.traj(2,:); %y
           m_traj(3,:) = constrain_angle(-obj.traj(3,:)-pi); %theta
           m_traj(4,:) = -obj.traj(4,:); %alpha
           
           new_prim = Primitive(m_Id, m_fromHeadingId, m_toHeadingId, m_cost, m_traj, m_dir);        
           
        end
        function new_prim = mirrorToThird(obj)
           m_Id = obj.primId;
           m_fromHeadingId = mod(8 + obj.fromHeadingId, 16); 
           m_toHeadingId = mod(8 + obj.toHeadingId, 16);
           m_dir = obj.direction;
           m_cost = obj.cost;    
           [nStates, N] = size(obj.traj);
           m_traj = zeros(nStates, N);
           m_traj(1,:) = -obj.traj(1,:); %x
           m_traj(2,:) = -obj.traj(2,:); %y
           m_traj(3,:) = constrain_angle(obj.traj(3,:)+pi); %theta
           m_traj(4,:) = obj.traj(4,:); %alpha
           
           new_prim = Primitive(m_Id, m_fromHeadingId, m_toHeadingId, m_cost, m_traj, m_dir);        
           
        end
        function new_prim = mirrorToFourth(obj)
           m_Id = obj.primId;
           m_fromHeadingId = mod(16 - obj.fromHeadingId, 16); 
           m_toHeadingId = mod(16 - obj.toHeadingId, 16); 
           m_dir = obj.direction;
           m_cost = obj.cost;    
           [nStates, N] = size(obj.traj);
           m_traj = zeros(nStates, N);
           m_traj(1,:) = obj.traj(1,:); %x
           m_traj(2,:) = -obj.traj(2,:); %y
           m_traj(3,:) = constrain_angle(-obj.traj(3,:)); %theta
           m_traj(4,:) = -obj.traj(4,:); %alpha
           
           new_prim = Primitive(m_Id, m_fromHeadingId, m_toHeadingId, m_cost, m_traj, m_dir);        
           
        end        
        
        function store_primitive(obj, path, number)
            filename = strcat(path,'primitive',int2str(number),'.txt');
            fileID = fopen(filename,'w');
            fprintf(fileID, 'primId: %i \n', obj.primId);
            fprintf(fileID, 'cost: %f \n', obj.cost);
            fprintf(fileID, 'direction: %i \n', obj.direction);
            fprintf(fileID, 'fromHeadingId: %i \n', obj.fromHeadingId);
            fprintf(fileID, 'toHeadingId: %i \n', obj.toHeadingId);
            [~, N] = size(obj.traj);
            fprintf(fileID, 'path: \n');
            fprintf(fileID,'x        y         theta      alpha \n');
            for j=1:N
            fprintf(fileID,'%f %f %f %f \n',obj.traj(:,j));
            end
            fclose(fileID);
        end
        
        function plot_primitive(obj, nstep, lw, col)
            plot(obj.traj(1,1:nstep:end), obj.traj(2,1:nstep:end), 'linewidth',lw, 'color',col )
        end
    end
end


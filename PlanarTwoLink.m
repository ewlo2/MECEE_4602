classdef PlanarTwoLink < handle
    % This class defines a planar manipulator, which has two links. Each
    % link rotates about a revolute joint located at its end.
    % This program solves the kinematics and dynamics of system.
    % Author:      Haohan Zhang
    % Edits: Rand Hidayah and Fitsum E. Petros
    % Affiliation: ROAR @ Columbia

    
    properties
        Link      % link lengths; a vector.
    end
    properties (Access = private)
        JointAngle    % angles of each joint, a vector
    end
    
    methods 
        % constructor
        function this = PlanarTwoLink(link)
            % construct a robot
            if nargin > 0
                this.Link = link;
            end
        end
        
        % setters
        function setJointAngle(this,value)
            % this assigns the joint angles
            this.JointAngle = value;
        end
        
        
        
        
        % getters
        function value = getJointAngle(this)
            % this returns the joint angles
            value = this.JointAngle;
        end
        
        % helpers
        function posA = calcPosA(this,q1)
            % this calculate the postion of the tip of the first link
            %FILL IN HERE 
            L = this.Link;
            x = L(1)*cosd(q1+180);
            y = -L(1)*sind(q1+180);
            z = 0;
            posA = [x;y;z];
        end
        function posB = calcPosB(this,q)
            % this calculate the position of the far end of the 2nd link
            %FILL IN HERE 
            L = this.Link;
%             x = (L(1)+L(2))*cos(q(1));
%             y = (L(1)+L(2))*sin(q(1));
            x = L(1)*cosd(q(1)+180);
            y = -L(1)*sind(q(1)+180)+L(2)*sind(q(2));
            z = L(2)/cosd(q(2))
            posB = [x;y;z];
        end
        
        function posC = calcPosC(this,q)
            % this calculate the position of the far end of the 2nd link
            %FILL IN HERE 
            L = this.Link;
%             x = (L(1)+L(2))*cos(q(1));
%             y = (L(1)+L(2))*sin(q(1));
            x = L(1)*cosd(q(1)+180);
            y = -L(1)*sind(q(1)+180)-L(2)*sind(q(2));
            z = -L(2)/cosd(-q(2))
            posC = [x;y;z];
        end
        
        function q = InverseKinematics(this,x, y, z)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % FILL THIS OUT: you can use a symbolic solver as well  
            % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            L = this.Link;
            %solved by hand 
%            q2 = acos((x^2 + y^2 -L(2)^2 -L(1)^2)/(2*L(2)*L(1)));
%            q1 = atan(-x/y);
%            q = [q1;q2];
           % or you can use a symbolic solver 
           syms q1 q2
           eqns = [ L(1)*cos(q1)== x, -L(1)*sin(q1)+L(2)*sin(q2)==y, L(2)-L(2)*cos(q2) == z ];
           qsol = solve(eqns,[q1 q2]);
           q = [qsol.q1(1)  qsol.q2(1)]; %
         
           if any(isnan((q)))
               disp("can not reach or is not in the workspace")
               return
           end 
           
        end 
        
        function [q, v, a] = IKmotion(this,q1i, q2i,q1f, q2f,q1vi, q1vf, q2vi, q2vf,t_0, t,n)
           q1 = [q1i q1f q1vi q1vf];
           q2 = [q2i q2f q2vi q2vf];
%            T = linspace(t_0,t,n*(t-t_0));
            T = linspace(0,t,n*(t-t_0));
            coffq1 = inv([ 0 0 0 1; t^3 t^2 t 1; 0 0 1 0; 3*t^2 2*t 1 0])*q1';
            coffq2 = inv([ 0 0 0 1; t^3 t^2 t 1; 0 0 1 0; 3*t^2 2*t 1 0])*q2';
            q1t = coffq1(1).*T.^3 +coffq1(2).*T.^2+ coffq1(3)*T + coffq1(4);
            q2t = coffq2(1).*T.^3 +coffq2(2).*T.^2+ coffq2(3).*T+coffq2(4);
            v1t = 3*coffq1(1).*T.^2 + 2*coffq1(2).*T+ coffq1(3);
            v2t = 3*coffq2(1).*T.^2 + 2*coffq2(2).*T+ coffq2(3);
            a1t = 6*coffq1(1).*T + 2*coffq1(2);
            a2t = 6*coffq2(1).*T + 2*coffq2(2);
            q = [q1t;q2t];
            v = [v1t;v2t];
            a = [a1t;a2t];
        end 
        function plotRobot(this)
            % this plots the geometry of robot
            O    = [0;0;0];
            q    = this.getJointAngle;
            posA = this.calcPosA(q(1));
            posB = this.calcPosB(q);
            posC = this.calcPosC(q);
            L1   = [O posA].'; 
            L2   = [posA posB].';
            L3   = [posA posC].';
            hold on
            l1 = animatedline('Color','r', 'linewidth',2);
            l2 = animatedline('Color','b','linewidth',2);
            l3 = animatedline('Color','b','linewidth',2);
            view(3);
            
            addpoints(l1, L1(:,1), L1(:,3), L1(:,2))
            addpoints(l2, L2(:,1), L2(:,3), L2(:,2))
            addpoints(l3, L3(:,1), L3(:,3), L3(:,2))
            hold off
        end
        function animateMotion(this,dt, vid)
            % this animates the motion of the robot
%            	set(gca, 'Xlim', [-30 30], 'Ylim', [-10,10], 'Zlim', [0,10]);
            axis([-30 30,-10,10,-30,30]);
            axis square
            grid on
            this.plotRobot;
            drawnow
            pause(dt) % pause with a 'correct' timing
            frame = getframe(gcf);
            writeVideo(vid,frame)
            clf
        end
    end
    
end


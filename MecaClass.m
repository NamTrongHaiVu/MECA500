classdef MecaClass < handle
    properties
        model;
        simulation;
        eStop = false;
        %toolOffset = transl(0.06,0,0.065);
        workspace = [-0.6 0.6 -0.6 0.6 -0.1 0.65];    
        qNeutral = [0,deg2rad(45),deg2rad(90),deg2rad(45),0];
        draw = 0;
    end
    methods
        %% Constructor
        function self = MecaClass()
            Placemeca = transl(0,0,0);
            Creation(self,Placemeca);
        end
        %% E-stop function
        function estop(self)
            while(self.eStop == true)
                disp('E-stop pressed');
                pause(0.05);
            end
        end
        %% Creating the Dobot both model with attachment and simulation
        function Creation(self, Placemeca)
            
            pause(0.001);
        name = ['Meca500_',datestr(now,'yyyymmddTHHMMSSFFF')]; 
        L1 = Link('d',0.135,'a',0,'alpha',(-3*pi)/2,'qlim',deg2rad([-175 175]), 'offset',0);
        L2 = Link('d',0,'a',0.135,'alpha',2*pi,'qlim', deg2rad([-70 90]), 'offset',pi/2);
        L3 = Link('d',0,'a',0.038,'alpha',pi/2,'qlim', deg2rad([-70 135]), 'offset', 0);
        L4 = Link('d',0.12,'a',0,'alpha',pi/2,'qlim',deg2rad([-170 170]),'offset', 0);
        L5 = Link('d',0,'a',0,'alpha',pi/2,'qlim',deg2rad([-115 115]), 'offset',-pi);
        L6 = Link('d',0.07,'a',0,'alpha',0,'qlim',deg2rad([-115 115]), 'offset',0);

            self.model = SerialLink([L1 L2 L3 L4 L5 L6], 'name', name);            
            self.model.base = Placemeca;
        end
        %% Plot Stick figure of the robot
        function PlotRobot(self,Placemeca)
            self.model.base = Placemeca;
            self.model.plot(self.qNeutral);
        end
        %% Plot 3D model with attachment
        function Plotmeca(self)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['Link_',num2str(linkIndex),'.ply'],'tri');
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end
            self.model.plot3d(self.qNeutral,'noarrow','workspace',self.workspace,'tile1color',[1 1 1]);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.model.delay = 0;
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                        , plyData{linkIndex+1}.vertex.green ...
                        , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
        %% RMRC
        function qOut = rmrc(self, steps, deltaTime, obj)
            %RMRC Move robot in RMRC
            %   Generate trajectory and animate robot motion
            
            minMani = 0.1;
            wayPoints = 2;
            wayPointRMRC = [0.3    0.0   (0.15+0.0754);
                0.3    0.0   (0.02+0.0754)];
            
            qMat = zeros(steps,5);
            trans = zeros(3,steps);
            rot = zeros(3,steps);
            
            s = lspb(0,1,steps);                                    % Trapezoidal trajectory scalar
            for i = 1:wayPoints-1
                for j = 1:steps
                    trans(1,j,i) = (1-s(j))*wayPointRMRC(i,1) + s(j)*wayPointRMRC(i+1,1);            % Points in x
                    trans(2,j,i) = (1-s(j))*wayPointRMRC(i,2) + s(j)*wayPointRMRC(i+1,2);            % Points in y
                    trans(3,j,i) = (1-s(j))*wayPointRMRC(i,3) + s(j)*wayPointRMRC(i+1,3);            % Points in z
                    rot(:,j,i) = zeros(3,1);                                       % Yaw angle
                end
                startPos = makehgtform('translate', trans(:,1,i));
                qMat(1,:,i) = self.model.ikcon(startPos, [0 pi/3 -pi/3 0 0]);
            end
            
            for i = 1:wayPoints-1
                text_h = text(-0.5, -0.5, 0.45, 'RMRC Mode Juice Extraction', 'FontSize', 10, 'Color', [.6 .2 .6]);
                for j = 1:steps-1
                    
                    if self.sensors_sim(qMat(j,:,i))
                        disp('Obstacle detected, halt operation!')
                        
                        qOut = [];
                        self.eStop = true;
                        
                        pause(1)
                        delete(text_h);
                        return
                    end
                    if (evalin("base","eStop") || self.eStop)
                        self.eStop = true;
                        while self.eStop
                            self.eStop = evalin("base","eStop");
                            pause(0.5);
                        end
                    end
                    T = self.model.fkine(qMat(j,:,i));               % Get forward transformation at current joint state
                    deltaTrans = trans(:,j+1,i) - T(1:3,4);         % Get position error from next waypoint
                    Rd = rpy2r(rot(1,i+1),rot(2,i+1),rot(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                    Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                    Rdot = (1/deltaTime)*(Rd - Ra);                                                % Calculate rotation matrix error
                    S = Rdot*Ra';                                                           % Skew symmetric!
                    veloRot = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                    veloTrans = deltaTrans / deltaTime;             % Calculate velocity at discrete time step
                    xdot = [veloTrans; veloRot];                          % Calculate end-effector velocity to reach next waypoint
                    J = self.model.jacob0(qMat(j,:,i));         % Jacobian at current pose
                    % Check Manipulability
                    m = sqrt(det(J*J'));
                    if m < minMani
                        lambda = 0.01;      %(1 - m(i)/minMani)*5E-2;
                    else
                        lambda = 0;
                    end
                    invJ = inv(J'*J + lambda*eye(5))*J';                                   % DLS Inverse
                    qdot(j,:,i) = (invJ * xdot)';                             % Singularity avoidance with DLS
                    %         for j = 1:6                                                             % Loop through joints 1 to 6
                    %             if qMatrix(i,j) + deltaT*qdot(i,j) < p560.qlim(j,1)                     % If next joint angle is lower than joint limit...
                    %                 qdot(i,j) = 0; % Stop the motor
                    %             elseif qMatrix(i,j) + deltaT*qdot(i,j) > p560.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                    %                 qdot(i,j) = 0; % Stop the motor
                    %             end
                    %         end
                    qMat(j+1,:,i) = qMat(j,:,i) + deltaTime*qdot(j,:,i);        % Update next joint state
                    self.model.animate(qMat(j,:,i));
                    
                    obj.MoveObj([(T(1:3,4)'+[0.02 0 -0.0954]) 0 0 0]);
                    obj.MoveObj([(T(1:3,4)'+[0.0 0 -0.0954]) 0 0 0]);
                    drawnow();
                    pause(0.02);
                    
                end
                delete(text_h);
            end
            qOut = qMat;
            % Sugar cane falls
            obj.MoveObj([0.3 0 -0.1 0 0 0]);
            pause(0.2);
            obj.MoveObj([0.3 0 -0.16 0 0 0]);
        end
        
        %% Trapezoidal velocity profile
        function qMat = trapezoidal(self, path, steps, animate, payload)
            pos1 = makehgtform('translate', path(1,:));
            pos2 = makehgtform('translate', path(2,:));
            
            qStart = self.model.ikcon(pos1, [0 pi/3 pi/3 0 0]);
            qEnd = self.model.ikcon(pos2, [-pi/2 pi/3 pi/3 0 0]);
            
            s = lspb(0,1,steps);
            
            for i = 1:steps
                qMat(i,:) = (1-s(i))*qStart + s(i)*qEnd;
                if (nargin < 4)
                    continue
                elseif (animate)
                    if (evalin("base","eStop") || self.eStop)
                        self.eStop = true;
                        while self.eStop
                            self.eStop = evalin("base","eStop");
                            pause(0.5);
                        end
                        if (nargin == 4)
                            text_h = text(-0.5, -0.5, 0.45, 'Trapezoidal no payload', 'FontSize', 10, 'Color', [.6 .2 .6]);
                            
                        end
                        self.model.animate(qMat(i,:));
                        if (nargin == 5)
                            effectorPos = self.model.fkine(qMat(i,:));
                            payload.MoveObj([(effectorPos(1:3,4)'+[0.0 0 -0.0954]) 0 0 0]);
                            text_h = text(-0.5, -0.5, 0.45, 'Trapezoidal with payload', 'FontSize', 10, 'Color', [.6 .2 .6]);
                        end
                        drawnow();
                        pause(0.02)
                    end
                    delete(text_h);
                end
            end
        end
        
    end
end

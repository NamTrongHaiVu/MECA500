clear all; close all; clc;
%% Environment
workspace = [-2 2 -2 2 -0.61 2];
environment();
%enviGUI();
%% Call Robots
r = Meca500;
r2 = myUR3;
%% Object
side = input('input the obstruction:');
switch side
    case 0.1
        centerpnt = [-0.5,0.49,0.055];
    case 0.2
        centerpnt = [-0.3,0.49,0.1];
    case 0.23
        centerpnt = [-0.3,0.49,0.14];
    case 0.27
        centerpnt = [-0.3,0.49,0.14];
    otherwise
        centerpnt = [0,0,-0.14];
end

plotOptions.plotFaces = true;
[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);

%% Tubes

tube1 = placetube('bigtube.ply');
tube2(:,:,1) = placetube('tube2.ply');
tube2(:,:,2) = placetube('tube2.ply');
tube2(:,:,3) = placetube('tube2.ply');
tube1.update([0.92,0,0.08,0,0,0]);
tube2(:,:,1).update([-0.785,-0.04,0.07,0,0,0]);
tube2(:,:,2).update([-0.785,0.02,0.07,0,0,0]);
tube2(:,:,3).update([-0.785,0.08,0.07,0,0,0]);

tubeo(:,:,1) = transl(-0.8,-0.1, 0.07)*trotx(90,'deg')*transl(0.185,0,-0.331);
tubeo(:,:,2) = transl(-0.8,0, 0.07)*trotx(90,'deg')*trotz(90,'deg')*transl(0,-0.25,-0.3);
tubeo(:,:,3) = transl(-0.8, 0.1, 0.07)*trotx(-90,'deg')*troty(-90,'deg')*transl(-0.022,0,-0.15);
%tube2.update([-0.785,0.02,0.07,0,0,0]);

%% Waypoint of Meca500
steps = 100;
qm0 = [0,0,0,0,0,0];
qm1 = [-90 -20 -60 0 -78 0]*pi/180;
qm2 = r.my3.ikcon(transl(0,0,0)*tube1.Pose*troty(90,'deg')*transl(-0.06,0,-0.05),qm1);
qm3 = [90 -70 30 0 -40 0]*pi/180;
qm4 = [-90 -70 30 0 -40 0]*pi/180;


% Meca 500 trajectory
qMatrix51 = jtraj(qm0,qm1,steps);
qMatrix52 = jtraj(qm1,qm2,steps);
qMatrix53 = jtraj(qm2,qm3,steps);
qMatrix54 = jtraj(qm3,qm0,steps);
qMatrix55 = jtraj(qm1,qm4,steps);
qMatrix56 = jtraj(qm4,qm0,steps);

%way point of UR3
qr0 = [0 0 0 0 0 0];
qr1 = [0 -60 0 0 0 0]*pi/180;
qr2 = [150 -60 120 -60 60 90]*pi/180;

qMatrix1 = jtraj(qr0,qr1,steps);
for i =1:steps
    if IsCollision(r2.model,qMatrix1(i,:),faces,vertex,faceNormals)
        error('Emergency Stop');
    end
    r.my3.animate(qMatrix51(i,:));
    r2.model.animate(qMatrix1(i,:));
    drawnow()
end

qMatrix2 = jtraj(qr1,qr2,steps);
for i =1:steps
    if IsCollision(r2.model,qMatrix2(i,:),faces,vertex,faceNormals)
        error('Emergency Stop');
    end
    r.my3.animate(qMatrix52(i,:));
    r2.model.animate(qMatrix2(i,:));
    drawnow()
end

for i =1:steps
    r.my3.animate(qMatrix53(i,:));
    ee53 = r.my3.fkine(qMatrix53(i,:)) * troty(-90,'deg')* transl(0.05,0,-0.06); %end of effect
    tube1.update([ee53(1:3,4)',0,0,0]);
    drawnow()
end
%loop of tube
for m = 1:3
    if m == 3
        centerpnt = [0,0.3,0.1];
        side = 0.27;
        plotOptions.plotFaces = true;
        [vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
    end
    steps = 100;
            % qr0 = [0 0 0 0 0 0];
            % qr1 = [0 -60 0 0 0 0]*pi/180;
            % qr2 = [150 -60 120 -60 60 90]*pi/180;
    qr3 = r2.model.ikcon(transl(0,0,0)*tube2(:,:,m).Pose*troty(-90,'deg')*transl(0,0,-0.135),qr2);
    qr4 = r2.model.ikcon(transl(0,0,0)*tube2(:,:,m).Pose*troty(-90,'deg')*transl(0.1,0,-0.135),qr3);
    qr5 = [-40 -30 0 0 0 120]*pi/180;
    qr5d = [-40 -30 0 0 0 30]*pi/180;
    qr6 = [167 -70 100 -30 -270 90]*pi/180;
    qr7 = r2.model.ikcon(tubeo(:,:,m),qr6);

    % UR3 trajectory
    qMatrix1 = jtraj(qr0,qr1,steps);
    qMatrix2 = jtraj(qr1,qr2,steps);
    qMatrix3 = jtraj(qr2,qr3,steps);
    qMatrix4 = jtraj(qr3,qr4,steps);

    qMatrix5 = jtraj(qr4,qr5,steps);
    qMatrix5d = jtraj(qr5,qr5d,steps); %
    qMatrix6 = jtraj(qr5d,qr6,steps);
    qMatrix7 = jtraj(qr6,qr7,steps);
    qMatrix8 = jtraj(qr7,qr2,steps);

    %Ur3 New trajectory for Collision
    qc1 = [75 -60 90 -30 75 90]*pi/180;
    qc2 = [75 -90 90 -30 75 90]*pi/180;
    qc3 = [-10 -90 60 30 75 90]*pi/180;

    qMatrix5n = jtraj(qr4,qc1,steps);
    qMatrix6n = jtraj(qc1,qc2,steps);
    qMatrix7n = jtraj(qc2,qc3,steps);
    qMatrix8n = jtraj(qc3,qr5,steps);
    qMatrix9n = jtraj(qr5d,qc3,steps);
    qMatrix10n = jtraj(qc3,qr6,steps);

    for i =1:steps
        if IsCollision(r2.model,qMatrix3(i,:),faces,vertex,faceNormals)
            error('Emergency Stop');
        end
        r2.model.animate(qMatrix3(i,:));
        drawnow()
    end

    for i =1:steps
        if IsCollision(r2.model,qMatrix4(i,:),faces,vertex,faceNormals)
            error('Emergency Stop');
        end
        r2.model.animate(qMatrix4(i,:));

        e1 = r2.model.fkine(qMatrix4(i,:)) * troty(90,'deg')* transl(-0.135,0,0); %end of effect
        tube2(:,:,m).update([e1(1:3,4)',0,0,0]);

        drawnow()

    end

    %if
    if m ==3
        for i =1:steps
            if IsCollision(r2.model,qMatrix5n(i,:),faces,vertex,faceNormals)
                error('Emergency Stop');
            end
            r2.model.animate(qMatrix5n(i,:));

            e2 = r2.model.fkine(qMatrix5n(i,:)) * troty(90,'deg')* transl(-0.135,0,0); %end of effect
            tube2(:,:,m).update([e2(1:3,4)',0,0,0]);
            drawnow()
        end

        for i =1:steps
            if IsCollision(r2.model,qMatrix6n(i,:),faces,vertex,faceNormals)
                error('Emergency Stop');
            end
            r2.model.animate(qMatrix6n(i,:));

            e2 = r2.model.fkine(qMatrix6n(i,:)) * troty(90,'deg')* transl(-0.135,0,0); %end of effect
            tube2(:,:,m).update([e2(1:3,4)',0,0,0]);
            drawnow()

        end

        for i =1:steps
            if IsCollision(r2.model,qMatrix7n(i,:),faces,vertex,faceNormals)
                error('Emergency Stop');
            end
            r2.model.animate(qMatrix7n(i,:));

            e2 = r2.model.fkine(qMatrix7n(i,:)) * troty(90,'deg')* transl(-0.135,0,0); %end of effect
            tube2(:,:,m).update([e2(1:3,4)',0,0,0]);
            drawnow()

        end

        for i =1:steps
            if IsCollision(r2.model,qMatrix8n(i,:),faces,vertex,faceNormals)
                error('Emergency Stop');
            end
            r2.model.animate(qMatrix8n(i,:));

            e2 = r2.model.fkine(qMatrix8n(i,:)) * troty(90,'deg')* transl(-0.135,0,0); %end of effect
            tube2(:,:,m).update([e2(1:3,4)',0,0,0]);
            drawnow()

        end

         % rotate
        for i =1:steps
            if IsCollision(r2.model,qMatrix5d(i,:),faces,vertex,faceNormals)
                error('Emergency Stop');
            end
            r2.model.animate(qMatrix5d(i,:));

            e2d2 = r2.model.fkine(qMatrix5d(i,:)) * troty(90,'deg')* transl(-0.135,0,0); %end of effect
            tube2(:,:,m).update([e2d2(1:3,4)',0,-pi/2,pi/4]);
            drawnow()

        end

        for i =1:steps
            if IsCollision(r2.model,qMatrix9n(i,:),faces,vertex,faceNormals)
                error('Emergency Stop');
            end
            r2.model.animate(qMatrix9n(i,:));

            e2 = r2.model.fkine(qMatrix9n(i,:)) * troty(90,'deg')* transl(-0.135,0,0); %end of effect
            tube2(:,:,m).update([e2(1:3,4)',0,0,0]);
            drawnow()

        end

        for i =1:steps
            if IsCollision(r2.model,qMatrix10n(i,:),faces,vertex,faceNormals)
                error('Emergency Stop');
            end
            r2.model.animate(qMatrix10n(i,:));

            e2 = r2.model.fkine(qMatrix10n(i,:)) * troty(90,'deg')* transl(-0.135,0,0); %end of effect
            tube2(:,:,m).update([e2(1:3,4)',0,0,0]);
            drawnow()

        end

        %else
    else
        for i =1:steps
            if IsCollision(r2.model,qMatrix5(i,:),faces,vertex,faceNormals)
                error('Emergency Stop');
            end
            r2.model.animate(qMatrix5(i,:));

            e2 = r2.model.fkine(qMatrix5(i,:)) * troty(90,'deg')* transl(-0.135,0,0); %end of effect
            tube2(:,:,m).update([e2(1:3,4)',0,0,0]);
            drawnow()

        end
        % rotate
        for i =1:steps
            if IsCollision(r2.model,qMatrix5d(i,:),faces,vertex,faceNormals)
                error('Emergency Stop');
            end
            r2.model.animate(qMatrix5d(i,:));

            e2d = r2.model.fkine(qMatrix5d(i,:)) * troty(90,'deg')* transl(-0.135,0,0); %end of effect
            tube2(:,:,m).update([e2d(1:3,4)',0,-pi/2,pi/4]);
            drawnow()

        end

        for i =1:steps
            if IsCollision(r2.model,qMatrix6(i,:),faces,vertex,faceNormals)
                error('Emergency Stop');
            end
            r2.model.animate(qMatrix6(i,:));

            e3 = r2.model.fkine(qMatrix6(i,:)) * troty(90,'deg')* transl(-0.135,0,0); %end of effect
            tube2(:,:,m).update([e3(1:3,4)',0,0,0]);
            drawnow()

        end
    end
    %end else

    for i =1:steps
        if IsCollision(r2.model,qMatrix7(i,:),faces,vertex,faceNormals)
            error('Emergency Stop');
        end
        r2.model.animate(qMatrix7(i,:));

        e4 = r2.model.fkine(qMatrix7(i,:)) * troty(90,'deg')* transl(-0.135,0,0); %end of effect
        tube2(:,:,m).update([e4(1:3,4)',0,0,0]);
        drawnow()

    end

    for i =1:steps
        if IsCollision(r2.model,qMatrix8(i,:),faces,vertex,faceNormals)
            error('Emergency Stop');
        end
        r2.model.animate(qMatrix8(i,:));
        drawnow()

    end
end %%loop done

for i =1:steps
    r.my3.animate(qMatrix54(i,:));
    ee54 = r.my3.fkine(qMatrix54(i,:))* troty(-90,'deg')* transl(0.05,0,-0.06); %end of effect
    tube1.update([ee54(1:3,4)',0,0,0]);
    drawnow()
end

%% Shake
I1 = transl([0.4 0.12 0.3])*trotz(-pi/2)
I2 = transl([0.8 0.12 0.3])*trotz(-pi/2)

x1 = [0.4 0.12 0.3 0 1 0]';
x2 = [0.8 0.12 0.3 0 1 0]';

shakesteps = 50;

deltaT = 0.05;                                 % Discrete time step

s = lspb(0,1,shakesteps);                           % Create interpolation scalar

x = zeros(6,shakesteps);

for i = 1:shakesteps
    x(:,i) = x1*(1-s(i)) + s(i)*x2;            % Create trajectory in x-y plane
end

qShake = zeros(shakesteps,r.my3.n);
qShake(1,:) = r.my3.ikcon(I1);

for i = 1:shakesteps-1
    xdot = (x(:,i+1) - x(:,i))/deltaT;                  % Calculate velocity at discrete time step
    J = r.my3.jacob0(qShake(i,:));               % Get the Jacobian at the current state                        
    qdot = inv(J)*xdot;                                 % Solve velocitities via RMRC
    qShake(i+1,:) =  qShake(i,:) + deltaT*qdot';      % Update next joint state   
    r.my3.animate(qShake(i,:));
    es = r.my3.fkine(qShake(i,:))* troty(-90,'deg')* transl(0.05,0,-0.06); %end of effect
    tube1.update([es(1:3,4)',0,0,0]);
    drawnow();
end

% xx = zeros(6,shakesteps);
% for i = 1:shakesteps
%     xx(:,i) = x2*(1-s(i)) + s(i)*x1;            % Create trajectory in x-y plane
% end
% 
% qShake1 = zeros(shakesteps,r.my3.n);
% qShake1(1,:) = r.my3.ikcon(I2);
% 
% for i = 1:shakesteps-1
%     xdot1 = (xx(:,i+1) - xx(:,i))/deltaT;                  % Calculate velocity at discrete time step
%     J1 = r.my3.jacob0(qShake1(i,:));               % Get the Jacobian at the current state                                    % Take only first 2 rows
%     qdot1 = inv(J1)*xdot1;                                 % Solve velocitities via RMRC
%     qShake1(i+1,:) =  qShake1(i,:) + deltaT*qdot1';      % Update next joint state
%    
%     r.my3.animate(qShake1(i,:));
%     es1 = r.my3.fkine(qShake1(i,:)) * troty(-90,'deg')* transl(0.05,0,-0.06); %end of effect
%     tube1.update([es1(1:3,4)',0,0,0]);
%     drawnow();
% end
%end Shake

for i =1:steps
    r.my3.animate(qMatrix51(i,:));
    ee51_2 = r.my3.fkine(qMatrix51(i,:)) * troty(-90,'deg')* transl(0.05,0,-0.06); %end of effect
    tube1.update([ee51_2(1:3,4)',0,0,0]);
    drawnow()
end

for i =1:steps
    r.my3.animate(qMatrix55(i,:));
    ee55 = r.my3.fkine(qMatrix55(i,:)) * troty(-90,'deg')* transl(0.05,0,-0.06); %end of effect
    tube1.update([ee55(1:3,4)',0,0,0]);
    drawnow()
end

%%Ending
qend = jtraj(qr2,qr0,steps);
centerpnt = [0,0.3,0.1];
side = 0.27;
plotOptions.plotFaces = true;
[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
for i =1:steps
    if IsCollision(r2.model,qend(i,:),faces,vertex,faceNormals)
        error('Emergency Stop');
    end
    r.my3.animate(qMatrix56(i,:));
    r2.model.animate(qend(i,:));
    drawnow()
end

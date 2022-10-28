I1 = transl([0.85 0.19 0.308]); 
I2 = transl([-0.85 0.19 0.308]);

x1 = [0.85 0.19 0.308 0 1 0]';
x2 = [-0.85 0.19 0.308 0 1 0]';

shakesteps = 20;

deltaT = 0.005;                                 % Discrete time step

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
%     tr = r.model.fkine(qShake(i,:));
%     transformedVertices1 = [tr * [brick1_Verts,ones(brick1_VertexCount,1)]']';
%     brick1_h.Vertices = transformedVertices1(:,1:3);
    drawnow();
end


xx = zeros(6,shakesteps);
for i = 1:shakesteps
    xx(:,i) = x2*(1-s(i)) + s(i)*x1;            % Create trajectory in x-y plane
end

qShake1 = zeros(shakesteps,r.my3.n);
qShake1(1,:) = r.my3.ikcon(I2);

for i = 1:shakesteps-1
    xdot1 = (xx(:,i+1) - xx(:,i))/deltaT;                  % Calculate velocity at discrete time step
    J1 = r.my3.jacob0(qShake1(i,:));               % Get the Jacobian at the current state                                    % Take only first 2 rows
    qdot1 = inv(J1)*xdot1;                                 % Solve velocitities via RMRC
    qShake1(i+1,:) =  qShake1(i,:) + deltaT*qdot1';      % Update next joint state
   
    r.my3.animate(qShake1(i,:));
%     tr1 = r.my3.fkine(qShake1(i,:));
%     transformedVertices1 = [tr1 * [brick1_Verts,ones(brick1_VertexCount,1)]']';
%     brick1_h.Vertices = transformedVertices1(:,1:3);
    drawnow();
end
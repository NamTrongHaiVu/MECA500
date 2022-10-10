function goalJointState = GetGoalJointState(blasterRobot,ufoFleet,solutionToUse)

    if nargin < 3
        solutionToUse = 1;
    end
if solutionToUse == 1
    goalJointState = blasterRobot.getpos() + (rand(1,6)-0.5) * 20*pi/180;
    endEffectorTr = blasterRobot.fkine(goalJointState);
    % Ensure the Z component of the Z axis is positive (pointing upwards), and the Z component of the point is above 1 (approx mid height)
    
    while endEffectorTr(3,3) < 0.1 || endEffectorTr(3,4) < 1
       goalJointState = blasterRobot.getpos() + (rand(1,6)-0.5) * 20*pi/180;
       endEffectorTr = blasterRobot.fkine(goalJointState);
       display('trying again');
    end
elseif solutionToUse == 2
        goalJointState = blasterRobot.getpos();
        targetUFOIndex = find(0 < ufoFleet.healthRemaining,1);

        % We want the Z axis of the end-effector to be a line from the
        % end-effector to the ufo base (although we haven't solved for ikine
        % yet)
        actualTr = blasterRobot.fkine(goalJointState);
        aVector =  ufoFleet.model{targetUFOIndex}.base(1:3,4)' - actualTr(1:3,4)';
        % Make a unit vector
        aVector = unit(aVector);
        % What is the angle to the global Z?(if zero use the X and Y axis as
        % is)
        tr = eye(4);
        if ~all(aVector == [0,0,1])
            % N and O axes are arbitary (we don't care about yaw)
            oVector = unit(cross(aVector,[0,0,1]));
            nVector = cross(oVector,aVector);                
            R = [nVector',oVector',aVector'];
            tr(1:3,1:3) = R;
        end
        % Then we don't really care about the x,y,z since the UFOs are far away
        % only that we are point in the correct direction with z axis (so roll
        % and pitch are correct)

        minDistError = sqrt(sum((actualTr(1:3,3)' - aVector).^2));
        bestJointState = goalJointState;
        maxAllowableDistError = 0.2;
        iKineAlpha = 0.9;    

        while maxAllowableDistError < minDistError && 0 < iKineAlpha
            candidateJointState = blasterRobot.ikine(tr,bestJointState,[0,0,0,1,1,0],'alpha',iKineAlpha,'ilimit',20);

            % Fix joint limits if needed
            if ~all(blasterRobot.qlim(:,1) < candidateJointState'  &  candidateJointState' < blasterRobot.qlim(:,2))
                lessThanMinIndex = candidateJointState' < blasterRobot.qlim(:,1);
                greaterThanMaxIndex = blasterRobot.qlim(:,2) < candidateJointState';
                candidateJointState(lessThanMinIndex) = blasterRobot.qlim(lessThanMinIndex,1) + 10*pi/180;
                candidateJointState(greaterThanMaxIndex) = blasterRobot.qlim(greaterThanMaxIndex,2) - 10*pi/180;               
                display('Not in joint limits, so fixing');
            end

            actualTr = blasterRobot.fkine(candidateJointState);
            iKineAlpha = iKineAlpha - 0.1;
            currentDistError = sqrt(sum((actualTr(1:3,3)' - aVector).^2))
            % If less than the best so far and within joint limits
            if  currentDistError < minDistError            
                minDistError = currentDistError;
                bestJointState = candidateJointState;
            end
        end
        % If error is small enough then use it otherwise stay put
        if minDistError < maxAllowableDistError
            goalJointState = bestJointState;
        % If all zero already then choose a random pose
        elseif all(blasterRobot.getpos() == 0)
            goalJointState = (rand(1,6)-0.5) * 30*pi/180;
        % If not already zero and all current solutions are poor then go to zero 
        else
            goalJointState = zeros(1,6);
        end    
end
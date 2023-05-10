function exampleCommandMoveToTaskConfigROSGazebo(coordinator, taskConfig, tolerance)% This class is for internal use and may be removed in a future release
%
%   Move the manipulator to a task-space position
%   This command moves the manipulator from its current pose to a
%   desired task-space pose. 

% Copyright 2020 The MathWorks, Inc.

    % First, check if robot is already at destination
    anglesTarget = rotm2eul(taskConfig(1:3,1:3),'XYZ');
    poseTarget = [taskConfig(1:3,4);anglesTarget']; 
    isAway = checkTargetAchieved();


    if isAway % if robot not at desired pose, compute a task-based joint trajectory
        ik = inverseKinematics('RigidBodyTree',coordinator.Robot);
        ik.SolverParameters.AllowRandomRestart = false;
        weights = [1 1 1 1 1 1];

        %Initial task config
        taskInit = coordinator.CurrentRobotTaskConfig;

        % Final task config
        taskFinal = taskConfig;

        % Time intervals
        timeInterval = [0;4];
        trajTimes = timeInterval(1):coordinator.TimeStep:timeInterval(end);

        % Retrieve task configurations between initial and final
        [s,sd,sdd] = trapveltraj(timeInterval',numel(trajTimes));
        [T, ~, ~] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes, 'TimeScaling',[s;sd;sdd]/timeInterval(end));

        % Compute corresponding joint configurations
        robotPos = zeros(size(T,3),coordinator.NumJoints);
        initialGuess = wrapToPi(coordinator.CurrentRobotJConfig);
        for i=1:size(T,3)            
            robotPos(i,:) = ik(coordinator.RobotEndEffector,T(:,:,i),weights,initialGuess);
            robotPos(i,:) = wrapToPi(robotPos(i,:));
            initialGuess = robotPos(i,:);            
        end   

        %%  Compute joint velocities and accelerations at required rate for execution by the robot
        disp('Done planning trajectory, now sampling...')

        % Interpolated joint velocities
        h = coordinator.TimeStep;
        robotVelTemp = diff(robotPos)/h;
        robotVel= [zeros(1,coordinator.NumJoints);robotVelTemp];

        % Interpolated joint accelerations
        robotAccTemp = diff(robotVelTemp)/h;
        robotAcc = [zeros(2,coordinator.NumJoints);robotAccTemp];

        q = robotPos';
        qd = robotVel';
        qdd = robotAcc';    

        %% Package and send the desired trajectory to robot for execution by the JointTrajectoryController
        disp('Done sampling trajectory, now packaging...')
        [trajAct,trajGoal] = rosactionclient('/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory');
        exampleHelperPackageJointTrajectoryKINOVAROSGazebo(trajGoal,coordinator.ROSinfo.gazeboJointNames,q,qd,qdd,trajTimes);  

        disp('Done packaging trajectory, now sending...')
        sendGoal(trajAct,trajGoal)

        % Wait until the robot reaches destination
        pause(1.0);        % this pause is needed to start asking for movement status
        isAway = true;

        while isAway
            isAway = checkTargetAchieved();
            pause(0.2);
        end   

        % Wait until the robot stops moving again
        isMoving = true;
        while isMoving
            [isMoving] = getMovementStatus(coordinator);
            pause(0.2);
        end            
    end

    % Update current robot configuration
    coordinator.CurrentRobotJConfig = getCurrentRobotJConfig(coordinator);
    coordinator.CurrentRobotTaskConfig = getTransform(coordinator.Robot, coordinator.CurrentRobotJConfig, coordinator.RobotEndEffector); 

    % Trigger Stateflow chart Event
    coordinator.FlowChart.taskConfigReached; 

    function isAway = checkTargetAchieved()
        isAway = true;
        jointCurrent = getCurrentRobotJConfig(coordinator);
        taskCurrent = getTransform(coordinator.Robot, jointCurrent, coordinator.RobotEndEffector);
        anglesCurrent = rotm2eul(taskCurrent(1:3,1:3), 'XYZ');
        poseCurrent =  [taskCurrent(1:3,4);anglesCurrent'];
        diffCurrent = abs([poseTarget(1:3)-poseCurrent(1:3); angdiff(poseCurrent(4:6),poseTarget(4:6))]);
        if all(diffCurrent<max(0.05,tolerance))       
            isAway=false; % goal achieved       
        end        
    end
end
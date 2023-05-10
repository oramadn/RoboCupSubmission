%initiate ROS connection
% robot = loadrobot("frankaEmikaPanda", "DataFormat","row");
robot = importrobot("frankaEmikaPanda.urdf");
% rosIP = "192.168.79.128"; % IP address of ROS enabled machine  
% rosshutdown;
% rosinit(rosIP,11311);

% setInitialConfig_franka; This file was not modified, it is now
% implemented in classRobot for convience

% [gripAct,gripGoal] = rosactionclient('/franka_gripper/gripper_action');
% gripperCommand = rosmessage(gripAct); 

% frankaSLActivateGripper(1);

endEffectorFrame = 'panda_hand';
coordinator = classRobot(robot, endEffectorFrame);

coordinator.HomeRobotTaskConfig = getTransform(robot, coordinator.CurrentRobotJConfig, endEffectorFrame);
coordinator.FlowChart = exampleHelperFlowChartPickPlaceROSGazebo('coordinator', coordinator);

answer = questdlg('Do you want to start the pick-and-place job now?', ...
         'Start job','Yes','No', 'No');

switch answer
    case 'Yes'
        % Trigger event to start Pick and Place in the Stateflow Chart
        coordinator.FlowChart.startPickPlace;       
    case 'No'
        coordinator.FlowChart.endPickPlace;
        delete(coordinator.FlowChart)
        delete(coordinator);
end

if strcmp(answer,'Yes')
    while  coordinator.NumDetectionRuns <  4
        % Wait for no parts to be detected.
    end
end

% curImage = receive(coordinator.ROSinfo.rgbImgSub,3);
% pause(1)
% rgbImg = rosReadImage(coordinator.ROSinfo.rgbImgSub.LatestMessage); 
% imshow(rgbImg)
classdef classRobot <handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        FlowChart
        Robot
        World = {};
        Parts = {};
        Obstacles = {};
        ObstaclesVisible = {};
        DetectedParts = {};
        RobotEndEffector
        CurrentRobotJConfig
        CurrentRobotTaskConfig
        NextPart = 0;
        PartOnRobot = 0;
        HomeRobotTaskConfig 
        PlacingPose
        GraspPose
        Figure
        TimeStep
        MotionModel
        NumJoints
        NumDetectionRuns = 0;
        CollisionHelper
        ROSinfo
        DetectorModel
        jointMess
        jointPub
    end
    
    methods
        function obj = classRobot(robot, robotEndEffector)
            obj.Robot = robot;

            % Initialize ROS utilities
            disp('Initializing ROS Utilities')
            obj.ROSinfo.configClient = rossvcclient('/gazebo/set_model_configuration');
            obj.ROSinfo.gazeboJointNames = {'panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7','panda_finger_joint1','panda_finger_joint2'}; % joint names of robot model in GAZEBO
            obj.ROSinfo.rgbImgSub = rossubscriber("/camera/rgb/image_raw");
            % obj.ROSinfo.rgbImgSub = rossubscriber("/camera/rgb/image_raw","sensor_msgs/Image","DataFormat","struct"); %To convert the pic to struct
            obj.jointMess = rosmessage("geometry_msgs/PoseStamped");
            obj.jointPub = rospublisher("/cartesian_impedance_example_controller/equilibrium_pose");
            % initialconfig = [0.4000,0,0.8000,0.0000,-1.0000,-0.0000,0.0050]

            % Initialize robot configuration in GAZEBO
            disp('Initializing robot configuration')
            gripperX = 0.4;
            gripperY = 0;
            gripperZ = 0.4;

            gripperRotationX = -pi; % radians
            gripperRotationY = -0.01; % radians
            gripperRotationZ = 0; % radians

            obj.jointMess.Pose.Position.X = gripperX;
            obj.jointMess.Pose.Position.Y = gripperY;
            obj.jointMess.Pose.Position.Z = gripperZ;
            quat = angle2quat(gripperRotationX, gripperRotationY, gripperRotationZ, "XYZ");
            obj.jointMess.Pose.Orientation.W = quat(1);
            obj.jointMess.Pose.Orientation.X = quat(2);
            obj.jointMess.Pose.Orientation.Y = quat(3);
            obj.jointMess.Pose.Orientation.Z = quat(4);
            send(obj.jointPub,obj.jointMess)
            pause(2);
            
            % Unpause GAZEBO physics
            disp('Unpausing GAZEBO physics')
            physicsClient = rossvcclient('gazebo/unpause_physics');
            physicsResp = call(physicsClient,'Timeout',5);
            pause(3);

            % Update robot properties
            obj.CurrentRobotJConfig = getCurrentRobotJConfig(obj);
            obj.RobotEndEffector = robotEndEffector;
            obj.CurrentRobotTaskConfig = getTransform(obj.Robot, obj.CurrentRobotJConfig, obj.RobotEndEffector);
            obj.TimeStep = 0.01; % used by Motion Planner
            obj.NumJoints = numel(obj.CurrentRobotJConfig);

            % Load deep learning model for object detection
            temp = load('exampleHelperYolo2DetectorROSGazebo.mat');
            obj.DetectorModel = temp.detectorYolo2;
        end
        
        function configResp = setCurrentRobotJConfig(obj, JConfig)            
            configReq = rosmessage(obj.ROSinfo.configClient)
            configReq.ModelName = "panda"
            configReq.UrdfParamName = "/robot_description"
            configReq.JointNames = obj.ROSinfo.gazeboJointNames
            configReq.JointPositions = JConfig
            configResp = call(obj.ROSinfo.configClient, configReq, 'Timeout', 3)
        end

        function JConfig = getCurrentRobotJConfig(obj)
            controllerStateSub = rossubscriber('/joint_states','BufferSize', 100); 
            jMsg = receive(controllerStateSub);
            delete(controllerStateSub);
            jointNames = {'panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7','panda_finger_joint1','panda_finger_joint2'};
            jointPositions = [jMsg.Position]';
            jointStruct = struct('JointName', jointNames, 'JointPosition', num2cell(jointPositions));    
            JConfig = jointStruct;
        end

        function isMoving = getMovementStatus(obj)
            statusMsg = receive(obj.ROSinfo.controllerStateSub);
            velocities = statusMsg.Actual.Velocities;
            if all(velocities<0.03)
                isMoving = 0;
            else
                isMoving = 1;
            end
        end
        
        % Display current job state
        function displayState(obj, message)
            disp(message);
            set(obj.Figure, 'NumberTitle', 'off', 'Name', message)
        end
        
        % Delete function
        function delete(obj)
            delete(obj.FlowChart)
        end
    end
end


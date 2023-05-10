% getWorldPropServ = rossvcclient('gazebo/get_world_properties','DataFormat','struct');
% serviceMsg = rosmessage(getWorldPropServ);
% msg = call(getWorldPropServ, serviceMsg)


% Create ROS service client
setModelStateClient = rossvcclient('/gazebo/set_model_state');

% Define model state
modelState = rosmessage('gazebo_msgs/ModelState');
modelState.ModelName = 'gCan2_Static';
modelState.Pose.Position.X = 0.221645;
modelState.Pose.Position.Y = -0.021967;
modelState.Pose.Position.Z = 0.547827;
modelState.Pose.Orientation.X = 0;
modelState.Pose.Orientation.Y = 0;
modelState.Pose.Orientation.Z = 0;
modelState.Pose.Orientation.W = 0;
modelState.ReferenceFrame = 'world';
% Call service to set model state
setModelStateReq = rosmessage(setModelStateClient);
setModelStateReq.ModelState = modelState;
response = call(setModelStateClient, setModelStateReq);
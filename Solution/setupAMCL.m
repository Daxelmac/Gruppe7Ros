%% Monte Carlo - Taken from "Localize TurtleBot Using Monte Carlo Localization" example
function [amcl] = setupAMCL(map)
    odometryModel = odometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];
rangeFinderModel = likelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0.45 8];
rangeFinderModel.Map = map;

% Query the Transformation Tree (tf tree) in ROS.
tftree = rostf;
waitForTransform(tftree, '/base_link', '/camera_depth_frame');
sensorTransform = getTransform(tftree,'/base_link', '/camera_depth_frame');

% Get the euler rotation angles.
laserQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X ...
    sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');

% Setup the |SensorPose|, which includes the translation along base_link's
% +X, +Y direction in meters and rotation angle along base_link's +Z axis
% in radians.
rangeFinderModel.SensorPose = [sensorTransform.Transform.Translation.X sensorTransform.Transform.Translation.Y laserRotation(1)];
amcl = monteCarloLocalization;
amcl.UseLidarScan = true;
amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;
amcl.UpdateThresholds = [0.2,0.2,0.2];
amcl.ResamplingInterval = 1;
amcl.ParticleLimits = [500 5000];
amcl.GlobalLocalization = false;
amcl.InitialPose = ExampleHelperAMCLGazeboTruePose;
amcl.InitialCovariance = eye(3) * 0.5;
end
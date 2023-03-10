function [initializationParam, continuousParam] = KittiParameters(K)

% INITIALIZATION PARAMETERS
initializationParam = struct();

% detectHarrisFeatures() parameters
initializationParam.MinQuality = 3e-4;
initializationParam.FilterSize = 11;

% vision.PointTracker() parameters
initializationParam.NumPyramidLevels = 4;
initializationParam.MaxBidirectionalError = 1;  
initializationParam.BlockSize = [21 21];            
initializationParam.MaxIterations = 30;

% estimateFundamentalMatrix() parameters
initializationParam.Method = 'RANSAC';
initializationParam.NumTrials = 3000;              
initializationParam.DistanceThreshold = 0.2;

% Camera Intrinsic Parameters
initializationParam.Intrinsics = cameraParameters('IntrinsicMatrix',K');

% Threshold for considering 2 points as the same 
initializationParam.Threshold_same_point = 6^2;

% range depth
initializationParam.RangeDepth = 7;

% -----------------------------
% CONTINUOUS PARAMETERS
continuousParam = struct();

% detectHarrisFeatures() parameters
continuousParam.MinQuality = 0.0001;
continuousParam.FilterSize = 11;

% vision.PointTracker() parameters
continuousParam.NumPyramidLevels = 3;
continuousParam.MaxBidirectionalError = 1;  
continuousParam.BlockSize = [15 15];            
continuousParam.MaxIterations = 17;

% Camera Intrinsic Parameters
continuousParam.Intrinsics = cameraParameters('IntrinsicMatrix',K');
continuousParam.K = K;

% estimateWorldCameraPose() parameters
continuousParam.MaxNumTrials = 3000;
continuousParam.Confidence = 85;
continuousParam.MaxReprojectionError = 2;

% Min angle for a landmark to be added 
continuousParam.Threshold_angle = 4;

% Threshold for considering 2 points as the same 
continuousParam.Threshold_same_point = 6^2;

% range depth
continuousParam.RangeDepth = 7;

% Pose refinement
continuousParam.poseRefinementFlag = true;

end
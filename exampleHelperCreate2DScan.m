function lidarScan2d = exampleHelperCreate2DScan(ptCloud)
% This helper function is used to create 2D Lidar scans from 3D Lidar
% Scans. These 2D scans are required for 2D submap creation which are
% useful for loop closure query. 3D Point Cloud Down sampling is done
% before creating a 2D scan to reduce the computations.
% annularSamplingRatio specifies the sample ratio to uniform sample the
% annular region. The sampling ratio is empirically chosen for this
% example.

%   Copyright 2019 The MathWorks, Inc.

annularSamplingRatio = 0.1; % Ratio used to sample extracted  3D point cloud annular region

% Use only a subset of the points randomly sampled, since we will have too
% many points if not
ptCloudDownSampled = pcdownsample(ptCloud, 'random', annularSamplingRatio);
loc = ptCloudDownSampled.Location;

% Get the cartesian co-ordinates of the accepted points
cart = double(loc(:,1:2));

lidarScan2d = lidarScan(cart);
end
function [submap] = exampleHelperCreateSubmap(lidarScans,poseGraph,currentId,nScansPerSubmap,maxRange)
% This helper function is used to create 2D submaps from 2D Lidar Scans which
% have been created by slicing the point clouds along an annular region.
% These Submaps are required for detecting loop closures. Each submap
% represents multiple scans.

%   Copyright 2019 The MathWorks, Inc.

submapResolution = 3; % Submap resolution
submapMaxLevel = 3; % Maximum Grid levels 

%% Initialize variables
scanIndices = (currentId-nScansPerSubmap+1) : currentId; % Scans to be added to the submap
poses3D = poseGraph.nodes(scanIndices); % Poses of these scans
anchorIndex = round(nScansPerSubmap/2); % Anchor index for the Submap

% Find the 2D position for this submap
poses2D = zeros(nScansPerSubmap,3);
lScans = cell(1,nScansPerSubmap);
for i = 1:nScansPerSubmap
    poses2D(i,1:2) = poses3D(i,1:2);
    eu = quat2eul(poses3D(i,4:end));
    poses2D(i,3) = eu(1);
    lScans{i} = lidarScans{scanIndices(i)};
end

% Create the submap
submap = nav.algs.internal.createSubmap(lScans, 1:nScansPerSubmap, poses2D, anchorIndex, submapResolution, maxRange, submapMaxLevel);

end


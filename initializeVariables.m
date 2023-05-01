pGraph = poseGraph3D;                                   % 3D Posegraph object for storing estimated relative poses
infoMat = [1,0,0,0,0,0,1,0,0,0,0,1,0,0,0,1,0,0,1,0,1];  % Default serialized upper-right triangle of 6-by-6 Information Matrix
numLoopClosuresSinceLastOptimization = 0;               % Number of loop closure edges added since last pose graph optimization and map refinement
mapUpdated = false;                                     % True after pose graph optimization until the next scan
scanAccepted = 0;                                       % Equals to 1 if the scan is accepted

% 3D Occupancy grid object for creating and visualizing 3D map
mapResolution = 8; % cells per meter
omap = occupancyMap3D(mapResolution);


% Preallocate variables for the processed point clouds, lidar scans, and submaps. 
% Create a downsampled set of point clouds for quickly visualizing the map.

pcProcessed = cell(1,length(pClouds));
lidarScans2d = cell(1,length(pClouds)); 
submaps = cell(1,length(pClouds)/nScansPerSubmap);

pcsToView = cell(1,length(pClouds)); 

% Create variables for display purposes.

viewMap = 1;    % Set to 1 to visualize created map and posegraph during build process
viewPC = 0;     % Set to 1 to visualize processed point clouds during build process

rng(0);         % Set random seed to guarantee consistent random sampling.

% Initialize figure windows if desired.

% If you want to view the point clouds while processing them sequentially
if viewPC==1
    pplayer = pcplayer([-50 50],[-50 50],[-10 10],'MarkerSize',10);
end

% If you want to view the created map and posegraph during build process
if viewMap==1
    ax = newplot; % Figure axis handle
    view([70 70]);
    % pause(10)
    grid on;
end
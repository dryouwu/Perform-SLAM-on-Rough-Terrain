function [loopSubmapIds,loopScores] = exampleHelperEstimateLoopCandidates(pGraph,currentScanId,submaps,currScan,...
    nScansPerSubmap,loopClosureSearchRadius,loopClosureThreshold,subMapThresh)
% This helper function to returns submap ids which lie within a radius from
% current scan and match with the current scan.Instead of matching the
% current scan with all the previously accepted scans for faster query
% current scan is matched against a submap (group of scans). Due to this
% the number of matching operation reduces significantly. The submaps are
% said to be matching with the current scan when the submap and scan match
% score is greater than loopClosureThreshold. Most recent subMapThresh
% submaps are not considered while estimating a loop sub map.

%   Copyright 2019 The MathWorks, Inc.

loopClosureMaxAttempts = 8; % Number of submaps checked
maxLoopClosureCandidates = 2; % Number of matches to send back, if matches are found

%% Initialize variables

loopSubmapIds = []; % Variable to hold the candidate submap IDs 
loopScores = []; % Variable to hold the score of the matched submaps

pose3D = pGraph.nodes(currentScanId); % Pose of the current node
currentSubMapId = floor((currentScanId-1)/nScansPerSubmap)+1; % Submap corresponding to the node

%% Find the submaps to be considered based on distance and Submap ID
% Find the most recent scan center
mostRecentScanCenter = zeros(1,3); 
mostRecentScanCenter(1:2) = pose3D(1:2);
eulAngles = quat2eul(pose3D(4:end));
mostRecentScanCenter(3) = eulAngles(1);

% Compute the centers of all Submaps 
nsbmp = floor(pGraph.NumNodes/nScansPerSubmap);
centers = zeros(nsbmp-1, 2);
for i = 1:nsbmp-1 % ignore the most recent submap
    centers(i,:) = submaps{i}.Center;
end

% Compute the distance of all submaps from the current scan
centerCandidates = zeros(nsbmp-1,2);
n=0; % To keep track of the number of candidates added
for i = 1:nsbmp-1
    distanceToCenters = norm(centers(i, :) - mostRecentScanCenter(1:2));
    % Accept the submap only if it is within the search radius and if its 
    % ID is above the submap threshold w.r.t the current submap
    if (distanceToCenters < loopClosureSearchRadius)&&(abs(i-currentSubMapId)>subMapThresh)
        n = n+1; % Increase the number of candidates added
        centerCandidates(n,:) = [distanceToCenters i]; % Distance and the Submap ID
    end
end
% Only keep the candidates added
centerCandidates = centerCandidates(1:n,:);

% If there are submaps to be considered, sort them by distance from the
% current scan
if ~isempty(centerCandidates)
    % Sort them based on the distance from the current scan
    centerCandidates = sortrows(centerCandidates);
    
    % Return only the minimum number of loop candidates to be returned
    N = min(loopClosureMaxAttempts, size(centerCandidates,1));
    nearbySubmapIDs = centerCandidates(1:N, 2)';
else
    nearbySubmapIDs = [];
end

%% Match the current scan with the candidate submaps
newLoopCandidates = zeros(1000,1); % Loop candidates
newLoopCandidateScores = zeros(1000,1); % Loop candidate scores
count = 0; % Number of loop closure matches found

% If there are submaps to consider
if ~isempty(nearbySubmapIDs)
    % For every candidate submap 
    for k = 1:length(nearbySubmapIDs)
        submapId = nearbySubmapIDs(k);
        
        % Match the scan with the submap
        [~, score, ~] = nav.algs.internal.matchScansGridSubmap(currScan, submaps{submapId}, 0, [0 0 0], [0 0], 0); 
        
        % Accept submap only if it meets the score threshold
        if score > loopClosureThreshold 
            count = count + 1;
            % Keep track of matched Submaps and their scores
            newLoopCandidates(count) = submapId;
            newLoopCandidateScores(count) = score;
        end
        
    end
    
    % If there are candidates to consider
    if ~isempty(newLoopCandidates)
        % Sort them by their scores in descending order
        [~,ord] = sort(newLoopCandidateScores,'descend');
        % Return the required number of submaps matched, and their scores
        loopSubmapIds = newLoopCandidates(ord(1:min(count,maxLoopClosureCandidates)));
        loopScores = newLoopCandidateScores(ord(1:min(count,maxLoopClosureCandidates)));  
    end
end
end
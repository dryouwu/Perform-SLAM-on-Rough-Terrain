function exampleHelperVisualizeMapAndPoseGraph(omap, pGraph, ax)
% This helper function is useful for visualizing the built occupancy map 3D
% (omap) and pose graph (pGraph). The plot view is tuned for this example.

%   Copyright 2019 The MathWorks, Inc.

show(omap,'Parent',ax);
hold on;
pGraph.show('Parent',ax,"IDs","off");
xlim([-40 80]);
ylim([-40 80]);
zlim([-20 10]);
zlim([-30 40]);
view([70 70]);
% view(2);

xlabel('X')
ylabel('Y')
zlabel('Z')

drawnow
hold off;
grid on;
end


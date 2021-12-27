% Set of obstacle points and april tag landmarks
X = [0.2 -0.2; 0.2 0.2; -0.2 0.2; -0.2 -0.2; 1 -0.5; 1 0; 1 0.5; -0.5 1; 0 1; 0.5 1; -1 -0.5; -1 0; -1 0.5; -0.5 -1; 0 -1; 0.5 -1];
% Function to plot coronoi diagram
voronoi(X(:,1),X(:,2));
% Get x and y coordinates of the endoints of each segment in vornoi diagram
[ei, ej] = voronoi(X(:,1),X(:,2));
% writing to disk
writematrix(ei, "ei.csv")
writematrix(ej, "ej.csv")
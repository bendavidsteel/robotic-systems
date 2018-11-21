function pathDist = pathLength(path)
%Finds the euclidean length of the path

pathDist = 0;

for i = 1:length(path)-1
    dist = sqrt((path(i + 1, 1) - path(i, 1))^2 + (path(i + 1, 2) - path(i, 2))^2);
    
    pathDist = pathDist + dist;
end
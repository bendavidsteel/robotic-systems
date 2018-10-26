function [weights, edges, locations, startNode, finishNode] = initialMapGridGraph(robot, map, start, finish, GRID_DENSITY, MIN_WALL_DIST)

%Accepts a robot with assigned map
%Returns a graph with distances between all nodes in matrix form,
%A boolean matrix indicating which nodes are connected
%And the cartesian locations of the nodes on the graph

%formatting map
map(length(map)+1, :) = map(1, :);
mapLines = zeros(length(map)-1, 4);  %each row represents a border of the map
for i = 1:size(mapLines,1)
    mapLines(i,:) = [map(i,:) map(i+1,:)];
end

limsMin = min(map); % minimum limits of the map
limsMax = max(map); % maximum limits of the map

noPoints = 2;

startNode = 1;
finishNode = 2;

points = zeros(round(((limsMax(1) - limsMin(1)) / GRID_DENSITY) * ((limsMax(2) - limsMin(2)) / GRID_DENSITY)) + 2, 2);

points(startNode, :) = start;
points(finishNode, :) = finish;

% plot(start(1) , start(2), '^b');
% plot(finish(1) , finish(2), 'vb');

for x = limsMin(1):GRID_DENSITY:limsMax(1)
    for y = limsMin(2):GRID_DENSITY:limsMax(2)
        if min(disToLineSeg([x,y], mapLines)) > MIN_WALL_DIST && robot.pointInsideMap([x,y])
            noPoints = noPoints + 1;
            points(noPoints, :) = [x , y];
    %         plot(x , y, '*b');%inside map
        end
    end
end





locations = points(1:noPoints, :);

edges = false(noPoints);
weights = zeros(noPoints);

%comparing all combinations of points to check for edge on map within bounds
%if within bounds generating weights
for i = 1 : noPoints
    for j = i + 1 : noPoints
        withinBounds = true;
        
        %checking points along line good distance from map edge
        for r = 0:1:sqrt((locations(i,1) - locations(j,1))^2 + (locations(i,2) - locations(j,2))^2)
            
            theta = atan2(locations(j,2) - locations(i,2) , locations(j,1) - locations(i,1));
            x = locations(i,1) + (r * cos(theta));
            y = locations(i,2) + (r * sin(theta));
            
            if min(disToLineSeg([x,y], mapLines)) < MIN_WALL_DIST*sqrt(2) && robot.pointInsideMap([x,y])
                withinBounds = false;
                break
            end
        end
        
        if withinBounds
            edges(i,j) = true;
            edges(j,i) = true;
            
            %finding weights
            dist = sqrt((locations(i,1) - locations(j,1))^2 + (locations(i,2) - locations(j,2))^2);
            weights(i,j) = dist;
            weights(j,i) = dist;
            
%             plot([locations(i,1), locations(j,1)], [locations(i,2), locations(j,2)]);
        end
    end
end
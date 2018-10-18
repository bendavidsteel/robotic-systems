clf;        %clears figures
clc;        %clears console
clear;      %clears workspace

MAX_POINTS = 10;
GEN_RADIUS = 2;
GEN_TIME_OUT = 100;
MIN_WALL_DIST = 1;
MAX_GRAPH_DIST = 10;
NO_GRAPH_ITERATIONS = 100;
TIME_OUT = 10;

% map = [0,0 ; 60,0 ; 60,45 ; 45,45 ; 45,59 ; 106,59 ; 106,105 ; 0,105]; %defining map
% start = [55,25 ; 20,30];
% finish = [100,85 ; 20,80];
map = [0,0 ; 100,0 ; 100,20 ; 40,20 ; 40,70 ; 80,70 ; 80,50 ; 100,50 ; 100,100 ; 0,100];
start = [80,10];
finish = [90,60];

robot = BotSim(map); %generating map



foundPath = false;

%find initial path with random spread of nodes throughout accessable areas in map
while ~foundPath
    %clear figure of previous tries
    clf;
    robot.drawMap();
    hold on;
    %generating initial map of nodes
    [weights, edges, locations, startNode, finishNode] = initialMapGraph(robot, map, start, finish, MAX_POINTS, GEN_RADIUS, GEN_TIME_OUT, MIN_WALL_DIST);
    %find path
    path = aStarSearch(robot, weights, edges, locations, startNode, finishNode);
    
    if path == false
        foundPath = false;
    else
        foundPath = true;
    end
end

bestPath = path;
bestPathLen = pathLength(path);
%now that we've found the path, lets look for a better one
%we'll regenerate the map at random in areas around the located path
for i = 1:NO_GRAPH_ITERATIONS
    
    oldPath = path;
    oldPathLen = pathLength(oldPath);
    
    newPathLen = Inf;
    
    foundPath = false;
    
    iterations = 0;
    
    %check that this new path is actually better than the old one
    while ~foundPath && newPathLen >= oldPathLen
        
        clf;
        robot.drawMap();
        hold on;
        %regenerating map with tighter constraints
        [weights, edges, locations, startNode, finishNode] = refinedMapGraph(robot, map, locations, start, finish, MAX_POINTS, GEN_RADIUS, GEN_TIME_OUT, MAX_GRAPH_DIST, MIN_WALL_DIST);
        %find path
        path = aStarSearch(robot, weights, edges, locations, startNode, finishNode);
        
        if path == false
            foundPath = false;
        else
            foundPath = true;
            newPathLen = pathLength(path);
            %saving best path
            if newPathLen < bestPathLen
                bestPath = path;
            end
        end
        
        %if can't come up with better path, exit
        iterations = iterations + 1;
        if iterations > TIME_OUT
            return
        end
    end
end
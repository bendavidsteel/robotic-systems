function score = pathFindingTesting(INITIAL_TRIES, MAX_POINTS, GEN_RADIUS, GEN_TIME_OUT, GRID_DENSITY, MIN_WALL_DIST, MAX_GRAPH_DIST, NO_GRAPH_ITERATIONS, RAD_EXPANSION)

% clf;        %clears figures

% MAX_POINTS = 10;
% GEN_RADIUS = 2;
% GEN_TIME_OUT = 100;
% MIN_WALL_DIST = 1;
% MAX_GRAPH_DIST = 10;
% NO_GRAPH_ITERATIONS = 100;
% TIME_OUT = 10;
% RAD_EXPANSION = 1.5;
% TUNE_FACTOR = 0.8;

% map = [0,0 ; 60,0 ; 60,45 ; 45,45 ; 45,60 ; 100,60 ; 100,100 ; 0,100]; %defining map
% start = [55,25];
% finish = [90,85];
% shortestDist = 88.8;
% map = [0,0 ; 100,0 ; 100,20 ; 40,20 ; 40,70 ; 80,70 ; 80,50 ; 100,50 ; 100,100 ; 0,100];
% start = [80,10];
% finish = [90,60];
% shortestDist = 145.4;
% map = [0,0 ; 30,0 ; 30,70 ; 40,70 ; 40,30 ; 100,30 ; 100,100 ; 80,100 ; 80,50 ; 70,50 ; 70,100 ; 0,100];
% start = [10,10];
% finish = [90,90];
% shortestDist = 160.5;
% map = [0,0 ; 100,0 ; 100,100 ; 0,100];
% start = [10,10];
% finish = [90,90];
% shortestDist = 113.1;

score = Inf;

%trial maps
maps = cell(4,1);
maps(1) = {[0,0 ; 60,0 ; 60,45 ; 45,45 ; 45,60 ; 100,60 ; 100,100 ; 0,100]};
maps(2) = {[0,0 ; 100,0 ; 100,20 ; 40,20 ; 40,70 ; 80,70 ; 80,50 ; 100,50 ; 100,100 ; 0,100]};
maps(3) = {[0,0 ; 30,0 ; 30,70 ; 40,70 ; 40,30 ; 100,30 ; 100,100 ; 80,100 ; 80,50 ; 70,50 ; 70,100 ; 0,100]};
maps(4) = {[0,0 ; 100,0 ; 100,100 ; 0,100]};
    
starts = cell(4,1);
starts(1) = {[55,25]};
starts(2) = {[80,10]};
starts(3) = {[10,10]};
starts(4) = {[10,10]};
      
finishes = cell(4,1);
finishes(1) = {[90,85]};
finishes(2) = {[90,60]};
finishes(3) = {[90,90]};
finishes(4) = {[90,90]};
        
shortestDists = [88.8, 145.4, 160.5, 113.1];

%randomly select a map
randint = randi(4);

map = maps{randint};
start = starts{randint};
finish = finishes{randint};
shortestDist = shortestDists(randint);

tic;

robot = BotSim(map); %generating map

foundPath = false;
iterations = 0;

%find initial path with random spread of nodes throughout accessable areas in map
while ~foundPath
    %clear figure of previous tries
%     clf;
%     robot.drawMap();
%     hold on;
    %generating initial map of nodes
    if iterations < INITIAL_TRIES
        [weights, edges, locations, startNode, finishNode] = initialMapGraph(robot, map, start, finish, MAX_POINTS, GEN_RADIUS, GEN_TIME_OUT, MIN_WALL_DIST);
    else
        [weights, edges, locations, startNode, finishNode] = initialMapGridGraph(robot, map, start, finish, GRID_DENSITY, MIN_WALL_DIST);
    end
    %find path
    path = aStarSearch(robot, weights, edges, locations, startNode, finishNode);
    
    if path == false
        foundPath = false;
        %prevent GEN_RADIUS going to infinity
        if GEN_RADIUS < 3
            GEN_RADIUS = GEN_RADIUS * RAD_EXPANSION;
        end
        
        iterations = iterations + 1;
    else
        foundPath = true;
    end
end

bestPath = path;
bestPathLen = pathLength(path);
%now that we've found the path, lets look for a better one
%we'll regenerate the map at random in areas around the located path
    
newPathLen = Inf;

foundPath = false;

iterations = 0;

%check that this new path is actually better than the old one
while ~foundPath && newPathLen >= bestPathLen

%         clf;
%         robot.drawMap();
%         hold on;
    %regenerating map with tighter constraints
    [weights, edges, locations, startNode, finishNode] = refinedMapGraph(robot, map, bestPath, start, finish, MAX_POINTS, GEN_RADIUS, GEN_TIME_OUT, MAX_GRAPH_DIST, MIN_WALL_DIST);
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
            bestPathLen = newPathLen;
        end
    end

    %if can't come up with better path, exit
    iterations = iterations + 1;
    if iterations > NO_GRAPH_ITERATIONS
        break
    end
end

%evaluating method
time = toc;
factor = bestPathLen/shortestDist;

%calculating score
score = 1 / (time * factor);

return
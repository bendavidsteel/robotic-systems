
MIN_WALL_DIST = 2;
MIN_PATH_DIST = 1;

map = [0,0 ; 30,0 ; 30,70 ; 40,70 ; 40,30 ; 100,30 ; 100,100 ; 80,100 ; 80,50 ; 70,50 ; 70,100 ; 0,100];
start = [10,10];
finish = [90,90];

robot = BotSim(map); %generating map

[weights, edges, locations, startNode, finishNode] = initialMapPointsGraph(robot, map, start, finish, MIN_WALL_DIST, MIN_PATH_DIST);
    
robot.drawMap();
hold on;
scatter(locations(:,1), locations(:,2), '*y')

path = aStarSearch(robot, weights, edges, locations, startNode, finishNode);
    
scatter(path(:,1), path(:,2), '*b')
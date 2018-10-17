clf;        %clears figures
clc;        %clears console
clear;      %clears workspace

MAX_POINTS = 10;

map = [0,0 ; 60,0 ; 60,45 ; 45,45 ; 45,59 ; 106,59 ; 106,105 ; 0,105]; %defining map

robot = BotSim(map); %generating map

start = [55, 25];
finish = [100, 85];

path = false;

while ~path
    clf;
    robot.drawMap();
    hold on;
    [weights, edges, locations, startNode, finishNode] = mapGraph(robot, map, start, finish, MAX_POINTS);

    path = aStarSearch(robot, weights, edges, locations, startNode, finishNode);
end
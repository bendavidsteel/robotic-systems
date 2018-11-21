clf;        %clears figures
clc;        %clears console
clear;      %clears workspace

NUMBER_OF_SCANS = 8; %number of scans

map = [0,0 ; 60,0 ; 60,45 ; 45,45 ; 45,59 ; 106,59 ; 106,105 ; 0,105]; %defining map

robot1 = BotSim(map); %generating map
robot2 = BotSim(map);

%adding noise
robot1.setSensorNoise(0.1);
robot1.setMotionNoise(0.1);
robot1.setTurningNoise(0.1);

robot2.setSensorNoise(0.1);
robot2.setMotionNoise(0.1);
robot2.setTurningNoise(0.1);

robot1.setScanConfig(robot1.generateScanConfig(NUMBER_OF_SCANS)) %setting scan configuration
robot2.setScanConfig(robot2.generateScanConfig(NUMBER_OF_SCANS))

robot1.drawMap(); %drawing map

robot1.randomPose(10); %drawing bot
robot2.randomPose(10);

[distances1, crossingPoint1] = robot1.ultraScan(); %performing scan
[distances2, crossingPoint2] = robot2.ultraScan();

%visualing map and scan
robot1.drawMap()
robot1.drawBot(3) 
robot2.drawBot(3)
drawnow;

%main loop

GOOD_CLEARANCE = 30;

while true
    %ROBOT 1
    
    %finding directions with good clearance
    goodDirections = find(distances1 > GOOD_CLEARANCE);
    
    if ~isempty(goodDirections)
        %if a few good directions, randomly select one, and turn and move in that direction
        newAngle = ((2*pi) / NUMBER_OF_SCANS) * (goodDirections(randi(length(goodDirections))) - 1);
        
        robot1.turn(newAngle)
        robot1.move(5)
    else
        %finding maximum distance to go in if no good distances
        [dist,index] = max(distances1);
        newAngle = ((2*pi) / NUMBER_OF_SCANS) * (index(0) - 1);
        
        robot1.turn(newAngle)
        robot1.move(max([dist*0.5, 5])) %move fraction of distance for best bet for new direction
    end
    
    %ROBOT2
    
    %finding directions with good clearance
    goodDirections = find(distances2 > GOOD_CLEARANCE);
    
    if ~isempty(goodDirections)
        %angle to robot
        pos1 = robot1.getBotPos();
        pos2 = robot2.getBotPos();
        
        angleToRobot1 = atan2(pos2(2) - pos1(2), pos2(1) - pos1(1));
        
        %find nearest good angle to direction of robot1
        closestIndex = 1;
        closest = 2*pi;
        
        for i = 1:length(goodDirections)
            if abs(angleToRobot1 - ((goodDirections(i) - 1)*2*pi / NUMBER_OF_SCANS)) < closest
                closest = abs(angleToRobot1 - ((goodDirections(i) - 1)*2*pi / NUMBER_OF_SCANS));
                closestIndex = i;
            end
        end
        
        newAngle = (goodDirections(closestIndex) - 1)*2*pi / NUMBER_OF_SCANS;
        
        robot2.turn(newAngle)
        robot2.move(5)
    else
        %finding maximum distance to go in if no good distances
        [dist,index] = max(distances1);
        newAngle = ((2*pi) / NUMBER_OF_SCANS) * (index(0) - 1);
        
        robot2.turn(newAngle)
        robot2.move(max([dist*0.5, 5])) %move fraction of distance for best bet for new direction
    end
        
    
    [distances1, crossingPoint1] = robot1.ultraScan(); %performing scan
    [distances2, crossingPoint2] = robot2.ultraScan();

    %visualing map and scan
    hold off
    robot1.drawMap()
    robot1.drawBot(3) 
    robot2.drawBot(3)
    drawnow;
end
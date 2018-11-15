function [botSim] = localiseParameters(botSim,map,target, noise, NO_PARTICLES, NUMBER_OF_SCANS, MODEL_NOISE, K, SEARCH_VAR, CONV_DIST, RESAMPLE_VAR, CLUSTER_PROPORTION, MAX_STEP, REDIST_PRO, BREAK_DIST)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);

botSim.setScanConfig(botSim.generateScanConfig(NUMBER_OF_SCANS));

%generate some random particles inside the map
%optimize for this as well
num = NO_PARTICLES; % number of particles
particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    %particles(i) = BotSim(modifiedMap, [0,0,0]);
    if MODEL_NOISE == 1
        particles(i) = BotSim(modifiedMap, noise);  %each particle should use the same map as the botSim object
    else
        particles(i) = BotSim(modifiedMap, [0,0,0]);
    end
    particles(i).setScanConfig(particles(i).generateScanConfig(NUMBER_OF_SCANS));
    particles(i).randomPose(0); %spawn the particles in random locations
end

%% Localisation code
GOOD_CLEARANCE = 10;
MIN_WALL_DIST = 2;
MIN_PATH_DIST = 1;
MAX_NUM_OF_ITERATIONS = 100;

drawMap = false;

n = 0;
converged =0; %The filter has not converged yet

path = [0,0];
clusterParticles = [0,0];
clusterCount = 1;
bestPos = [0,0];

while(n < MAX_NUM_OF_ITERATIONS) %%particle filter loop
%while true
    n = n+1; %increment the current number of iterations
    botScan = botSim.ultraScan(); %get a scan from the real robot.
    
    %% Write code for updating your particles scans
    scans = zeros(length(botScan),num);
    %virtually scanning from every particle
    for i = 1:num
        scans(:,i) = particles(i).ultraScan();
    end
    
    if drawMap
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'g'); %draw robot with line length 30 and green
%         for i =1:num
%             particles(i).drawBot(3); %draw particle with line length 3 and default color
%         end
        %REMOVE FOR REAL ROBOT
        positions = zeros(num,2);
        for i = 1:num
            positions(i,:) = particles(i).getBotPos();
        end
        
%         [positionsUnique,ia,ic] = unique(positions, 'rows', 'stable');           % Unique Values By Row, Retaining Original Order
%         occurences = accumarray(ic, 1);
        
        %displaying particles
        scatter(positions(:,1), positions(:,2), 0.5, '*r')
        %displaying path
        scatter(path(:,1), path(:,2), '*y')
        %displaying target
        scatter(target(1), target(2), '*g')
        %displaying converged location
        if converged
            scatter(sum(clusterParticles(:,1))/clusterCount, sum(clusterParticles(:,2))/clusterCount, '*b')
        end
        %displaying best guess for location
        scatter(bestPos(1), bestPos(2), '*c')
        
        drawnow;
    end
    
    %% Write code for scoring your particles   
    particlesCorr = zeros(num, 1);
    corrSum = 0;
    
    %finding correlation of each virtual scan with real scan, using cyclical shift
    for particle = 1:num
        maxCorr = 0;
        for offset = 0:length(botScan)-1
            corr = K;
            for scan = 0:length(botScan)-1
                corr = corr + normpdf(scans(mod(scan + offset, length(botScan)) + 1, particle), botScan(scan+1), SEARCH_VAR);
            end
            
            if corr > maxCorr
                maxCorr = corr;
                bestOffset = offset;
            end
        end
        
        particles(particle).turn(bestOffset * (2*pi / 6));
        
        particlesCorr(particle) = maxCorr;
        corrSum = corrSum + maxCorr;
    end
    
    %normalising weights
    for particle = 1:num
        particlesCorr(particle) = particlesCorr(particle) / corrSum;
    end
    
    %save position of most correlated particle
    [~, particleMaxCorr] = max(particlesCorr);
    bestPos = particles(particleMaxCorr).getBotPos();
    bestAng = mod(particles(particleMaxCorr).getBotAng(), 2*pi);
    
    %% Write code for resampling your particles
    %using roulette wheel sampling for each particle to find a new position
    newParticles = particles;
    
    %redistribute some particles according to sampling
    for i = 1:(num*REDIST_PRO)
        randNum = rand();

        for j = 1:num
            randNum = randNum - particlesCorr(j);

            if randNum < 0
                newPoint = j;
                break
            end
        end
        
        pos = particles(newPoint).getBotPos();
        angle = mod(particles(newPoint).getBotAng(), 2*pi);
        pos(1) = normrnd(pos(1), RESAMPLE_VAR);
        pos(2) = normrnd(pos(2), RESAMPLE_VAR);
        
        newParticles(i).setBotPos(pos);
        newParticles(i).setBotAng(angle);
    end
    
    for i = round((REDIST_PRO*num))+1:num
        newParticles(i).randomPose(0);
    end
    
    %others should be rerandoml
    
    %copying new particle positions into old particle elements
    particles = newParticles;
    
    if drawMap
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'g'); %draw robot with line length 30 and green
%         for i =1:num
%             particles(i).drawBot(3); %draw particle with line length 3 and default color
%         end

        %REMOVE FOR REAL ROBOT
        positions = zeros(num,2);
        for i = 1:num
            positions(i,:) = particles(i).getBotPos();
        end
        
%         [positionsUnique,~,ic] = unique(positions, 'rows', 'stable');           % Unique Values By Row, Retaining Original Order
%         occurences = accumarray(ic, 1);
        
        %displaying particles
        scatter(positions(:,1), positions(:,2), 0.5, '*r')
        %displaying path
        scatter(path(:,1), path(:,2), '*y')
        %displaying target
        scatter(target(1), target(2), '*g')
        %displaying converged location
        if converged
            scatter(sum(clusterParticles(:,1))/clusterCount, sum(clusterParticles(:,2))/clusterCount, '*b')
        end
        %displaying best guess for location
        scatter(bestPos(1), bestPos(2), '*c')
        
        drawnow;
    end
    
    %% Write code to check for convergence   
    %checking to see if all particles are within a certain distance of a random particle
    
    pos1 = bestPos;
    clusterCount = 0;
    clusterParticles = zeros(num,3);
    
    for i = 1:num
        pos2 = particles(i).getBotPos();
        
        if sqrt((pos1(1) - pos2(1))^2 + (pos1(2) - pos2(2))^2) < CONV_DIST
            clusterCount = clusterCount + 1;
            clusterParticles(i,1:2) = pos2;
            clusterParticles(i,3) = mod(particles(i).getBotAng(), 2*pi);
        end
    end
    %checking for suitable proportion of particles in one place indicating convergence
    if clusterCount > num*CLUSTER_PROPORTION
        converged = true;
        convLoc = [sum(clusterParticles(:,1))/clusterCount, sum(clusterParticles(:,2))/clusterCount];
        if sqrt((convLoc(1) - target(1))^2 + (convLoc(2) - target(2))^2) < BREAK_DIST
            break
        end
    else
        converged = false;
    end
    
      
    %% Write code to decide how to move next
    % move robot randonly while avoiding walls
    
    
    % using converged position or best guess
%     if converged
%         pos1 = [sum(clusterParticles(:,1))/clusterCount, sum(clusterParticles(:,2))/clusterCount];
%         % pos1Ang = sum(clusterParticles(:,3))/clusterCount;
%         pos1Ang = mode(clusterParticles(:,3));
%     else
%         pos1 = bestPos;
%         pos1Ang = bestAng;
%     end

    pos1 = bestPos;
    pos1Ang = bestAng;

    if converged
        [weights, edges, locations, startNode, finishNode] = initialMapPointsGraph(botSim, map, bestPos, target, MIN_WALL_DIST, MIN_PATH_DIST);

        path = aStarSearch(botSim, weights, edges, locations, startNode, finishNode);
    
        pos2 = path(2,:);

        goodDirections = find(botScan > min([GOOD_CLEARANCE, sqrt((pos2(2) - pos1(2))^2 + (pos2(1) - pos1(1))^2)]));
    else
        goodDirections = find(botScan > GOOD_CLEARANCE);
    end
    
    if ~isempty(goodDirections)
        if converged
            %if a few good directions, randomly select one, and turn and move in that direction
            %angle to robot

            angleToTarget = atan2(pos2(2) - pos1(2), pos2(1) - pos1(1));

            %finding absolute angle of first point in path from robot
            angle = mod(angleToTarget - pos1Ang, 2*pi);
            %checking if correct angle is has path which is in map

%             goodAngle = false;
%             for i = 1:length(goodDirections)
%                 if ((goodDirections(i) - 1)*2*pi / NUMBER_OF_SCANS < angle) && (goodDirections(i)*2*pi / NUMBER_OF_SCANS > angle)
%                     goodAngle = true;
%                 end
%             end
% 
%             if goodAngle
%                 turn = angle;
%             else
%                 %find nearest good angle to direction of next step in path if can't go directly
%                 closestIndex = 1;
%                 closest = 2*pi;
% 
%                 for i = 1:length(goodDirections)
%                     if abs(angle - ((goodDirections(i) - 1)*2*pi / NUMBER_OF_SCANS)) < closest
%                         closest = abs(angle - ((goodDirections(i) - 1)*2*pi / NUMBER_OF_SCANS));
%                         closestIndex = i;
%                     elseif abs((angle - 2*pi) - ((goodDirections(i) - 1)*2*pi / NUMBER_OF_SCANS)) < closest
%                         closest = abs((angle - 2*pi) - ((goodDirections(i) - 1)*2*pi / NUMBER_OF_SCANS));
%                         closestIndex = i;
%                     end
%                 end
% 
%                 turn = (goodDirections(closestIndex) - 1)*2*pi / NUMBER_OF_SCANS;
%             end
            
            % find nearest good angle to direction of next step in path if can't go directly
            closestIndex = 1;
            closest = 2*pi;

            for i = 1:length(goodDirections)
                if abs(angle - ((goodDirections(i) - 1)*2*pi / NUMBER_OF_SCANS)) < closest
                    closest = abs(angle - ((goodDirections(i) - 1)*2*pi / NUMBER_OF_SCANS));
                    closestIndex = i;
                elseif abs((angle - 2*pi) - ((goodDirections(i) - 1)*2*pi / NUMBER_OF_SCANS)) < closest
                    closest = abs((angle - 2*pi) - ((goodDirections(i) - 1)*2*pi / NUMBER_OF_SCANS));
                    closestIndex = i;
                end
            end

            turn = (goodDirections(closestIndex) - 1)*2*pi / NUMBER_OF_SCANS;

            move = min([MAX_STEP, sqrt((pos2(2) - pos1(2))^2 + (pos2(1) - pos1(1))^2)]);
        else
            %if not converged try to cover as much of map as possible
            turn = (goodDirections(1) - 1)*2*pi / NUMBER_OF_SCANS;
            move = 5;
        end
    else
        %finding maximum distance to go in if no good distances
        [dist,index] = max(botScan);
        turn = ((2*pi) / NUMBER_OF_SCANS) * (index(0) - 1);
        move = min([dist*0.5, MAX_STEP]); %move fraction of distance for best bet for new direction
    end
    
    %trying to prevent symmetry locks
%     if (abs(oldTurn - pi) < 0.2) && (abs(turn - pi) < 0.2)
%         turn = 0;
%     end
    
    botSim.turn(turn); %turn the real robot.  
    botSim.move(move); %move the real robot. These movements are recorded for marking 
    for i =1:num %for all the particles. 
        particles(i).turn(turn); %turn the particle in the same way as the real robot
        particles(i).move(move); %move the particle in the same way as the real robot
    end
    
    oldTurn = turn;
    
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    if drawMap
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'g'); %draw robot with line length 30 and green
%         for i =1:num
%             particles(i).drawBot(3); %draw particle with line length 3 and default color
%         end
        
        positions = zeros(num,2);
        for i = 1:num
            positions(i,:) = particles(i).getBotPos();
        end
        
%         [positionsUnique,ia,ic] = unique(positions, 'rows', 'stable');           % Unique Values By Row, Retaining Original Order
%         occurences = accumarray(ic, 1);
        
        %displaying particles
        scatter(positions(:,1), positions(:,2), 0.5, '*r')
        %displaying path
        scatter(path(:,1), path(:,2), '*y')
        %displaying target
        scatter(target(1), target(2), '*g')
        %displaying converged location
        if converged
            scatter(sum(clusterParticles(:,1))/clusterCount, sum(clusterParticles(:,2))/clusterCount, '*b')
        end
        %displaying best guess for location
        scatter(bestPos(1), bestPos(2), '*c')
        
        drawnow;
    end
end

end

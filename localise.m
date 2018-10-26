function [botSim] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);

NUMBER_OF_SCANS = 20;

%generate some random particles inside the map
num =300; % number of particles
particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(modifiedMap, [0,0,0]);
    particles(i).setScanConfig(particles(i).generateScanConfig(NUMBER_OF_SCANS));
    %particles(i) = BotSim(modifiedMap, [1,0.001,0.0005]);  %each particle should use the same map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations
end

%% Localisation code
K = 0; % constant to improve lifetime of particles
SEARCH_VAR = 5; % variance of norm distribution
CONV_DIST = 10;
RESAMPLE_VAR = 1;
GOOD_CLEARANCE = 20;
maxNumOfIterations = 100;
n = 0;
converged =0; %The filter has not converged yet
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    botScan = botSim.ultraScan(); %get a scan from the real robot.
    
    %% Write code for updating your particles scans
    scans = zeros(length(botScan),num);
    %virtually scanning from every particle
    for i = 1:num
        scans(:,i) = particles(i).ultraScan();
    end
    
    if botSim.debug()
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
        
        scatter(positions(:,1), positions(:,2), 0.5, '*r')
        
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
    
    %% Write code for resampling your particles
    %using roulette wheel sampling for each particle to find a new position
    newParticles = particles;
    
    %redistribute some particles according to sampling
    for i = 1:num
        randNum = rand();

        for j = 1:num
            randNum = randNum - particlesCorr(j);

            if randNum < 0
                newPoint = j;
                break
            end
        end
        
        pos = particles(newPoint).getBotPos();
        angle = particles(newPoint).getBotAng();
        pos(1) = normrnd(pos(1), RESAMPLE_VAR);
        pos(2) = normrnd(pos(2), RESAMPLE_VAR);
        
        newParticles(i).setBotPos(pos);
        newParticles(i).setBotAng(angle);
    end
    
    %others should be rerandoml
    
    %copying new particle positions into old particle elements
    particles = newParticles;
    
    if botSim.debug()
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
        
        scatter(positions(:,1), positions(:,2), 0.5, '*r')
        
        drawnow;
    end
    
    %% Write code to check for convergence   
    %checking to see if all particles are within a certain distance of a random particle
    
    pos1 = bestPos;
    clusterCount = 0;
    clusterParticles = zeros(num,2);
    
    for i = 1:num
        pos2 = particles(i).getBotPos();
        
        if sqrt((pos1(1) - pos2(1))^2 + (pos1(2) - pos2(2))^2) < CONV_DIST
            clusterCount = clusterCount + 1;
            clusterParticles(i,:) = pos2;
        end
    end
    
    if clusterCount > num*0.5
        break
    end
      
    %% Write code to decide how to move next
    % move robot randonly while avoiding walls
    
    goodDirections = find(botScan > GOOD_CLEARANCE);
    
    if ~isempty(goodDirections)
        %if a few good directions, randomly select one, and turn and move in that direction
        %angle to robot
        pos1 = bestPos;
        pos2 = target;
        
        angleToTarget = atan2(pos2(2) - pos1(2), pos2(1) - pos1(1));
        
        %find nearest good angle to direction of robot1
        closestIndex = 1;
        closest = 2*pi;
        
        for i = 1:length(goodDirections)
            if abs(angleToTarget - ((goodDirections(i) - 1)*2*pi / NUMBER_OF_SCANS)) < closest
                closest = abs(angleToTarget - ((goodDirections(i) - 1)*2*pi / NUMBER_OF_SCANS));
                closestIndex = i;
            end
        end
        
        turn = (goodDirections(closestIndex) - 1)*2*pi / NUMBER_OF_SCANS;
        move = 5;
    else
        %finding maximum distance to go in if no good distances
        [dist,index] = max(botScan);
        turn = ((2*pi) / NUMBER_OF_SCANS) * (index(0) - 1);
        move = max([dist*0.5, 5]); %move fraction of distance for best bet for new direction
    end
    
    botSim.turn(turn); %turn the real robot.  
    botSim.move(move); %move the real robot. These movements are recorded for marking 
    for i =1:num %for all the particles. 
        particles(i).turn(turn); %turn the particle in the same way as the real robot
        particles(i).move(move); %move the particle in the same way as the real robot
    end
    
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    if botSim.debug()
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
        
        scatter(positions(:,1), positions(:,2), 0.5, '*r')
        
        drawnow;
    end
end

scatter(sum(clusterParticles(:,1))/clusterCount, sum(clusterParticles(:,2))/clusterCount, '*b');

end

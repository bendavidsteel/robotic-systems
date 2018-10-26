function [botSim] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);

%generate some random particles inside the map
num =300; % number of particles
particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(modifiedMap, [0,0,0]);
    particles(i).setScanConfig(particles(i).generateScanConfig(20));
    %particles(i) = BotSim(modifiedMap, [1,0.001,0.0005]);  %each particle should use the same map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations
end

%% Localisation code
k = 0.1; % constant to improve lifetime of particles
var = 10; % variance of norm distribution
convConfidence = 0.6;
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
        
        [positionsUnique,ia,ic] = unique(positions, 'rows', 'stable');           % Unique Values By Row, Retaining Original Order
        occurences = accumarray(ic, 1);
        
        scatter(positionsUnique(:,1), positionsUnique(:,2), occurences, '*')
        
        drawnow;
    end
    
    %% Write code for scoring your particles   
    particlesCorr = zeros(num, 1);
    corrSum = 0;
    
    %finding correlation of each virtual scan with real scan, using cyclical shift
    for particle = 1:num
        maxCorr = 0;
        for offset = 0:length(botScan)-1
            corr = k;
            for scan = 0:length(botScan)-1
                corr = corr + normpdf(scans(mod(scan + offset, length(botScan)) + 1, particle), botScan(scan+1), var);
            end
            
            if corr > maxCorr
                maxCorr = corr;
                bestOffset = offset;
            end
        end
        
        particlesCorr(particle) = maxCorr;
        corrSum = corrSum + maxCorr;
    end
    
    %normalising weights
    for particle = 1:num
        particlesCorr(particle) = particlesCorr(particle) / corrSum;
    end
    
    %% Write code for resampling your particles
    %using roulette wheel sampling for each particle to find a new position
    newParticles(num,1) = BotSim;
    
    for i = 1:num
        randNum = rand();

        for j = 1:num
            randNum = randNum - particlesCorr(j);

            if randNum < 0
                newPoint = j;
                break
            end
        end
        
        newParticles(i) = particles(newPoint);
    end
    
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
        
        [positionsUnique,~,ic] = unique(positions, 'rows', 'stable');           % Unique Values By Row, Retaining Original Order
        occurences = accumarray(ic, 1);
        
        scatter(positionsUnique(:,1), positionsUnique(:,2), occurences, '*')
        
        drawnow;
    end
    
    %% Write code to check for convergence   
    %finding particle with greatest correlation and checking if it meets threshold
    
    [maxCorr, particleMaxCorr] = max(particlesCorr);
    
    if maxCorr > convConfidence
        converged = 1;
    end
      
    %% Write code to decide how to move next
    % here they just turn in cicles as an example
    turn = 0.5;
    move = 2;
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
        
        %REMOVE FOR REAL ROBOT
        positions = zeros(num,2);
        for i = 1:num
            positions(i,:) = particles(i).getBotPos();
        end
        
        [positionsUnique,ia,ic] = unique(positions, 'rows', 'stable');           % Unique Values By Row, Retaining Original Order
        occurences = accumarray(ic, 1);
        
        scatter(positionsUnique(:,1), positionsUnique(:,2), occurences, '*')
        
        drawnow;
    end
end

disp(particles(particleMaxCorr).getBotPos())

end

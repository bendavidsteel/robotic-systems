function result = localisationFunction(NO_PARTICLES, NUMBER_OF_SCANS, MODEL_NOISE, K, SEARCH_VAR, CONV_DIST, RESAMPLE_VAR, CLUSTER_PROPORTION, MAX_STEP, REDIST_PRO, BREAK_DIST)
%This is an example of how your work will be marked.

%IMPORTANT: You you need finish the function localise.m (a template has
%been provided for this example)

%Your localise.m function will be called for a variety of maps and targets.
%The localisation will be repeated several times for each map and an average
%score will be calculated. An average of all the scores for all the maps
%will give a total score for the simulation section of the assignment.

%The admin key is set outside of your function so you will not be able to
%access the debug functionality of the BotSim class.  Test your function
%works in this example.  If your function does not work in this example,
%you will get 0 marks.

%The final marking code will be different to this example, but if your
%function works in this example it will work in the final marking code.

%%setup
% clf;        %clears figures
% clc;        %clears console
% clear;      %clears workspace
% axis equal; %keeps the x and y scale the same

%More maps will be used
maps = cell(4,1); %needed for making jagged arrays
maps{1} = [0,0;60,0;60,45;45,45]; %Quadrilateral Map
maps{2} = [0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105]; %default map
maps{3} = [0,0 ; 100,0 ; 100,20 ; 40,20 ; 40,70 ; 80,70 ; 80,50 ; 100,50 ; 100,100 ; 0,100];
maps{4} = [0,0 ; 30,0 ; 30,70 ; 40,70 ; 40,30 ; 100,30 ; 100,100 ; 80,100 ; 80,50 ; 70,50 ; 70,100 ; 0,100];

%Different noise levels to be tested
%noiseLevel(:,1) = [0,0,0]; %no noise
noiseLevel(:,2) = [1,0.001,0.0005]; %all the noise

%The number of time the function is run so that the average performance can
%be calculated. This will be much larger during real marking.
%if the value is 1 it will run from predefined start and target positions
%If the number is greater than 1, the first test will be from predefined
%positions, and the rest will be randomised.
numberOfrepeats = 1;

%Predefined start and target positions
startPositions =  [10,5;20,20;80,10;10,10]; %These will change
targetPositions = [50,35;80,80;90,60;90,90]; %These will change

adminKey = rand(1); %During marking another key will be used ;)

resultsTime = zeros(size(maps,1),size(noiseLevel,3),numberOfrepeats);
resultsDis = resultsTime;
resultsLength = resultsTime;
resultsCollision = resultsTime;
% disp('checking BotSim.m checksum...')
% if(strcmp(md5(strcat(pwd,'\BotSim.m')), 'B0C6C4418E11B8D362DA430B938502BE') ==0)
%     disp('BotSim.m checksum failed to match.  Please replace with an umodified copy')
% else
%     disp('marking...')
    
    %% marking
    for i = 1:size(maps,1)
        for j=1:size(noiseLevel,2)
            %fprintf('map %0.f\t noiseLevels %0.f \n',i,j);
            for k = 1:numberOfrepeats
                %clf;        %clears figures
                botSim = BotSim(maps{i},noiseLevel(:,j),adminKey);  %sets up botSim object with adminKey
                botSim.setScanConfig(botSim.generateScanConfig(20));
                
                if k ==1 %runs from preset start and target positions first time, after that choose random positions
                    botSim.setBotPos(startPositions(i,:),adminKey)
                    target = targetPositions(i,:);
                else
                    botSim.randomPose(10); %puts the robot in a random position at least 10cm away from a wall
                    target = botSim.getRndPtInMap(10);  %gets random target
                end
%                 botSim.drawMap();
%                 botSim.drawBot(3);
%                 plot(target(1),target(2),'*');
%                 hold off;
%                 drawnow;
                
                tic %starts timer
                %calls your (hopefully finished) localisation function
                
                returnedBot = localiseParameters(botSim, maps{i}, target, noiseLevel(:,j), NO_PARTICLES, NUMBER_OF_SCANS, MODEL_NOISE, K, SEARCH_VAR, CONV_DIST, RESAMPLE_VAR, CLUSTER_PROPORTION, MAX_STEP, REDIST_PRO, BREAK_DIST);
                
                %results calculation
                resultsTime(i,j,k) = toc; %stops timer
                resultsDis(i,j,k) =  distance(target, returnedBot.getBotPos(adminKey));
                path = returnedBot.getBotPath(adminKey);             
                pathLength = 0;
                collided = 0;
                
                %% Calculate path length and if bot collides
                if size(path,1) >0
                    for m = 1:size(path,1)-1
                        if ~botSim.pointInsideMap(path(m,:))
                            collided = 1;
                        end
                        pathLength = pathLength + distance(path(m,:),path(m+1,:));
                    end
                    if ~botSim.pointInsideMap(path(size(path,1),:))
                        collided = 1;
                    end
                end
                
                %% collate and print results
                resultsLength(i,j,k) = pathLength;
                resultsCollision(i,j,k) = collided;
                % fprintf('Time: %.3f, Distance: %.3f, Length: %.3f, Collision: %.0f\n',resultsTime(i,j,k), resultsDis(i,j,k),pathLength,collided);
            end
            % disp(' ');
        end
    end
    
    %% Calculate average scores
    
    averageCompletionTime = sum(sum(resultsTime,3)/numberOfrepeats,2)/size(noiseLevel,2);
    averageDisFromTgt = sum(sum(resultsDis,3)/numberOfrepeats,2)/size(noiseLevel,2);
    averagePathLength = sum(sum(resultsLength,3)/numberOfrepeats,2)/size(noiseLevel,2);
    percentCollision = sum(sum(resultsCollision,3)/numberOfrepeats,2)/size(noiseLevel,2);
    
    compiledResults = [averageCompletionTime averageDisFromTgt averagePathLength percentCollision];
    
    result = sum(1 / (averageCompletionTime .* averageDisFromTgt .* averagePathLength .* (percentCollision+1)));
    % ResultsTable = table(averageCompletionTime, averageDisFromTgt, averagePathLength, percentCollision,'RowNames',{'Map1';'Map2';'Map3'})
end
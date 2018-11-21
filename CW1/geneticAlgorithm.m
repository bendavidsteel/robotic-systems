%genetic algorithm parameters
MUTATION_RATE = 0.1;
POPULATION = 10;
NO_GENES = 11;
NO_PARENTS = 2;
maxScore = 0;
greatestScore = 0;
generation = 0;

chromosomes = zeros(POPULATION, NO_GENES); 
scores = zeros(POPULATION, 1);

%initial population
for i = 1:POPULATION
    %Setting range that arguments can be randomly generated in
    NO_PARTICLES = abs(round(normrnd(293, 10)));
    NUMBER_OF_SCANS = abs(round(normrnd(7, 3)))+4;
    MODEL_NOISE = 0;
    K = abs(normrnd(0.1105, 0.01));
    SEARCH_VAR = abs(normrnd(0.31, 0.2));
    CONV_DIST = abs(normrnd(10.1, 2));
    RESAMPLE_VAR = abs(normrnd(2.985, 0.5));
    CLUSTER_PROPORTION = abs(normrnd(0.811, 0.1));
    MAX_STEP = abs(normrnd(5.53,1));
    REDIST_PRO = abs(normrnd(0.967, 0.01));
    BREAK_DIST = abs(normrnd(0.464, 1));
    
    %storing randomly generated arguments
    chromosomes(i, :) = [NO_PARTICLES, NUMBER_OF_SCANS, MODEL_NOISE, K, SEARCH_VAR, CONV_DIST, RESAMPLE_VAR, CLUSTER_PROPORTION, MAX_STEP, REDIST_PRO, BREAK_DIST];
    %running and evaluating algorithm with a set of arguments
    scores(i) = localisationFunction(NO_PARTICLES, NUMBER_OF_SCANS, MODEL_NOISE, K, SEARCH_VAR, CONV_DIST, RESAMPLE_VAR, CLUSTER_PROPORTION, MAX_STEP, REDIST_PRO, BREAK_DIST);
end

parents = zeros(NO_PARENTS, NO_GENES);

while true
    %choosing parents based on which algorithm performed best
    [scores_ranks, index_ranks] = sort(scores);
    %saving best performing argument sets
    for i = 1:NO_PARENTS
        parents(i,:) = chromosomes(index_ranks(POPULATION - i + 1), :);
    end
    
    %creating new population of algorithms with arguments either taken from
    %one of the parents, or re randomly generated
    for i = 1:POPULATION
        gene_choices = round((NO_PARENTS - 1)*rand(NO_GENES,1) + 1);
        
        %crossover or mutation
        if (rand(1) < MUTATION_RATE)
            NO_PARTICLES = abs(round(normrnd(NO_PARTICLES, 10)));
            
            %len_data = 40;
        else
            NO_PARTICLES = parents(gene_choices(1),1);
        end
        
        if (rand(1) < MUTATION_RATE)
            NUMBER_OF_SCANS = abs(round(normrnd(NUMBER_OF_SCANS, 3)))+4;
            
            %len_data = 40;
        else
            NUMBER_OF_SCANS = parents(gene_choices(2),2);
        end
        
        if (rand(1) < MUTATION_RATE)
            MODEL_NOISE = 0;
        else
            MODEL_NOISE = parents(gene_choices(3),3);
        end
        
        if (rand(1) < MUTATION_RATE)
            K = abs(normrnd(K, 0.01));
        else
            K = parents(gene_choices(4),4);
        end
        
        if (rand(1) < MUTATION_RATE)
            SEARCH_VAR = abs(normrnd(SEARCH_VAR, 1));
        else
            SEARCH_VAR = parents(gene_choices(5),5);
        end
        
        if (rand(1) < MUTATION_RATE)
            CONV_DIST = abs(normrnd(CONV_DIST, 2));
        else
            CONV_DIST = parents(gene_choices(6),6);
        end
        
        if (rand(1) < MUTATION_RATE)
            RESAMPLE_VAR = abs(normrnd(RESAMPLE_VAR, 0.5));
        else
            RESAMPLE_VAR = parents(gene_choices(7),7);
        end
        
        if (rand(1) < MUTATION_RATE)
            CLUSTER_PROPORTION = abs(normrnd(CLUSTER_PROPORTION, 0.1));
        else
            CLUSTER_PROPORTION = parents(gene_choices(8),8);
        end
        
        if (rand(1) < MUTATION_RATE)
            MAX_STEP = abs(normrnd(MAX_STEP,1));
        else
            MAX_STEP = parents(gene_choices(9),9);
        end
        
        if (rand(1) < MUTATION_RATE)
            REDIST_PRO = abs(normrnd(REDIST_PRO, 0.01));
        else
            REDIST_PRO = parents(gene_choices(10),10);
        end
        
        if (rand(1) < MUTATION_RATE)
            BREAK_DIST = abs(normrnd(BREAK_DIST, 1));
        else
            BREAK_DIST = parents(gene_choices(11),11);
        end
    
        chromosomes(i, :) = [NO_PARTICLES, NUMBER_OF_SCANS, MODEL_NOISE, K, SEARCH_VAR, CONV_DIST, RESAMPLE_VAR, CLUSTER_PROPORTION, MAX_STEP, REDIST_PRO, BREAK_DIST];
        %running and evaluating algorithm with a set of arguments
        scores(i) = localisationFunction(NO_PARTICLES, NUMBER_OF_SCANS, MODEL_NOISE, K, SEARCH_VAR, CONV_DIST, RESAMPLE_VAR, CLUSTER_PROPORTION, MAX_STEP, REDIST_PRO, BREAK_DIST);
    end
    
    [maxScore, index] = max(scores);
    %if this population features the best scorer of all time, save
    %argument set
    if(maxScore > greatestScore)
        greatestScore = maxScore;
        greatestAnimal = chromosomes(index, :);
    end
    %display score and generation number
    maxScore
    
    generation = generation + 1
end




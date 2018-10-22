%genetic algorithm parameters
MUTATION_RATE = 0.2;
POPULATION = 20;
NO_GENES = 9;
NO_PARENTS = 4;
maxScore = 0;
greatestScore = 0;
generation = 0;

chromosomes = zeros(POPULATION, NO_GENES); 
scores = zeros(POPULATION, 1);

%initial population
for i = 1:POPULATION
    %Setting range that arguments can be randomly generated in
    INITIAL_TRIES = randi(30);
    MAX_POINTS = randi(50);
    GEN_RADIUS = (4-1)*rand(1) + 1;
    GEN_TIME_OUT = randi(20);
    GRID_DENSITY = (20-5)*randi(1) + 5;
    MIN_WALL_DIST = 1;
    MAX_GRAPH_DIST = randi(100);
    NO_GRAPH_ITERATIONS = randi(100);
    RAD_EXPANSION = (2-1)*rand(1) + 1;
    
    %storing randomly generated arguments
    chromosomes(i, :) = [INITIAL_TRIES, MAX_POINTS, GEN_RADIUS, GEN_TIME_OUT, GRID_DENSITY, MIN_WALL_DIST, MAX_GRAPH_DIST, NO_GRAPH_ITERATIONS, RAD_EXPANSION];
    %running and evaluating algorithm with a set of arguments
    scores(i) = pathFindingTesting(INITIAL_TRIES, MAX_POINTS, GEN_RADIUS, GEN_TIME_OUT, GRID_DENSITY, MIN_WALL_DIST, MAX_GRAPH_DIST, NO_GRAPH_ITERATIONS, RAD_EXPANSION);
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
            INITIAL_TRIES = round(normrnd(INITIAL_TRIES, 5));
            %len_data = 40;
        else
            INITIAL_TRIES = parents(gene_choices(1),1);
        end
        
        if (rand(1) < MUTATION_RATE)
            MAX_POINTS = round(normrnd(MAX_POINTS, 10));
            %len_data = 40;
        else
            MAX_POINTS = parents(gene_choices(2),2);
        end
        
        if (rand(1) < MUTATION_RATE)
            GEN_RADIUS = normrnd(GEN_RADIUS, 1);
        else
            GEN_RADIUS = parents(gene_choices(3),3);
        end
        
        if (rand(1) < MUTATION_RATE)
            GEN_TIME_OUT = round(normrnd(GEN_TIME_OUT, 20));
        else
            GEN_TIME_OUT = parents(gene_choices(4),4);
        end
        
        if (rand(1) < MUTATION_RATE)
            GRID_DENSITY = round(normrnd(GRID_DENSITY, 5));
        else
            GRID_DENSITY = parents(gene_choices(5),5);
        end
        
        if (rand(1) < MUTATION_RATE)
            MIN_WALL_DIST = 1;
        else
            MIN_WALL_DIST = parents(gene_choices(6),6);
        end
        
        if (rand(1) < MUTATION_RATE)
            MAX_GRAPH_DIST = round(normrnd(MAX_GRAPH_DIST, 10));
        else
            MAX_GRAPH_DIST = parents(gene_choices(7),7);
        end
        
        if (rand(1) < MUTATION_RATE)
            NO_GRAPH_ITERATIONS = round(normrnd(NO_GRAPH_ITERATIONS, 10));
        else
            NO_GRAPH_ITERATIONS = parents(gene_choices(8),8);
        end
        
        if (rand(1) < MUTATION_RATE)
            RAD_EXPANSION = normrnd(RAD_EXPANSION, 0.1);
        else
            RAD_EXPANSION = parents(gene_choices(9),9);
        end
    
        chromosomes(i, :) = [INITIAL_TRIES, MAX_POINTS, GEN_RADIUS, GEN_TIME_OUT, GRID_DENSITY, MIN_WALL_DIST, MAX_GRAPH_DIST, NO_GRAPH_ITERATIONS, RAD_EXPANSION];
        %running and evaluating algorithm with a set of arguments
        scores(i) = pathFindingTesting(INITIAL_TRIES, MAX_POINTS, GEN_RADIUS, GEN_TIME_OUT, GRID_DENSITY, MIN_WALL_DIST, MAX_GRAPH_DIST, NO_GRAPH_ITERATIONS, RAD_EXPANSION);
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




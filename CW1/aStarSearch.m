function path = aStarSearch(robot, weights, edges, locations, start, finish)

%weights is a square symmetric integer matrix with distances between all nodes
%edges is a square symmetric boolean matrix showing which nodes are adjacent
%start is the number of the starting node
%end is the number of the ending node

%INITIALISATION

import java.util.ArrayList;

[noNodes,w] = size(weights);

if noNodes ~= w
    disp("Matrices should be square")
end

path = [locations(start,:); locations(finish,:)];

%Creating stacks
openList = ArrayList();
closedList = ArrayList();

%initialising f(x) = g(x) + h(x) vectors
f = zeros(noNodes, 1);
g = zeros(noNodes, 1);
h = zeros(noNodes, 1);

%parents
parents = zeros(noNodes, 1);

%adding start to empty list
openList.add(start);

%while open list is not empty
while openList.size() ~= 0
    
    %finding the node with the lowest f on the open list
    lowestF = Inf;
    indexLowestF = 1;
    
    %0 indexing due to using java stacks
    for i = 0:openList.size() - 1
        if f(openList.get(i)) < lowestF
            lowestF = openList.get(i);
            indexLowestF = i;
        end
    end
    
    %popping node and setting as predecessor
    pre = openList.get(indexLowestF);
    openList.remove(indexLowestF);
    
    %pushing pre onto closed list
    closedList.add(pre);
    
    plot(locations(pre,1), locations(pre,2), '*r');
    
    %checking if finish node is reached
    if pre == finish
        %reconstruct path
        route = zeros(noNodes, 1);
        
        i = 1;
        route(i) = pre;
        
        while parents(route(i)) ~= 0
            route(i + 1) = parents(route(i));
            i = i + 1;
        end
        
        %reversing path
        route = route(1:i);
        
        path = zeros(i,2);
        
        for i = 1:length(route)
            path(length(path)+1-i,:) = locations(route(i),:);
            plot(locations(route(i),1), locations(route(i),2), '*y');
        end
        
        return
    end
    
    %finding connected nodes to this predecessor
    for node = 1:noNodes
        if edges(pre,node) == true
            
            %making sure this node is not already in closed list with lower f
            inSet = false;
            
            for i = 0:closedList.size() - 1
                if closedList.get(i) == node
                    inSet = true;
                end
            end
            
            if inSet
                continue
            end
            
            %setting successor nodes' f, g and h
            tentativeG = g(pre) + weights(pre, node);
           
            %making sure this node is not already in open list with lower f
            inSet = false;
           
            for i = 0:openList.size() - 1
                if openList.get(i) == node
                    inSet = true;
                end
            end
           
            if ~inSet
                %pushing node onto open list
                openList.add(node);
                plot(locations(node,1), locations(node,2), '*g')
            elseif tentativeG >= g(node)
                continue
            end
            
            parents(node) = pre;
            
            %update f, g and h
            h(node) = sqrt((locations(finish, 1) - locations(node, 1))^2 + (locations(finish, 2) - locations(node, 2))^2);
            g(node) = tentativeG;
            f(node) = g(node) + h(node);
        end
    end
end
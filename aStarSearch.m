function path = aStarSearch(weights, edges, start, finish)

%weights is a square symmetric integer matrix with distances between all nodes
%edges is a square symmetric boolean matrix showing which nodes are adjacent
%start is the number of the starting node
%end is the number of the ending node

%INITIALISATION

[noNodes,w] = size(weights);

if noNodes ~= w
    "Matrices should be square"
end

%Creating stacks
openList = zeros(noNodes,1);
closedList = zeros(noNodes,1);

openListPtr = 0;
closedListPtr = 0;

%initialising f(x) = g(x) + h(x) vectors
f = zeros(noNodes, 1);
g = zeros(noNodes, 1);
h = zeros(noNodes, 1);

%parents
parents = zeros(noNodes, 1);

%adding start to empty list
openList(1) = start;
openListPtr = openListPtr + 1;

%while open list is not empty
while openListPtr ~= 0
    
    %finding the node with the lowest f on the open list
    lowestF = Inf;
    indexLowestF = 1;
    
    for i = 1:openListPtr
        if f(openList(i)) < lowestF
            lowestF = f(i);
            indexLowestF = i;
        end
    end
    
    %popping node and setting as predecessor
    pre = openList(indexLowestF);
    openList(indexLowestF) = 0;
    openListPtr = openListPtr - 1;
    
    %pushing pre onto closed list
    closedListPtr = closedListPtr + 1;
    closedList(closedListPtr) = pre;
    
    %checking if finish node is reached
    if pre == finish
        %reconstruct path
        path = zeros(noNodes, 1);
        
        i = 1;
        path(i) = pre;
        
        while parents(path(i)) ~= 0
            path(i + 1) = parents(path(i));
            i = i + 1;
        end
        
        %reversing path
        path = fliplr(path(1:i));
        
        return
    end
    
    %finding connected nodes to this predecessor
    for node = 1:noNodes
        if edges(pre,node) == true
            
           cont = False;
            
           %setting successor nodes' f, g and h
           nodeG = g(pre) + weights(pre, node);
           nodeH = weights(node, finish);
           nodeF = g(node) + h(node);
           
           %making sure this node is not already in open list with lower f
           for i = 1:openListPtr
               if openList(i) == node
                   if g(node) < nodeG
                       cont = True;
                   end
               end
           end
           
           %making sure this node is not already in closed list with lower f
           for i = 1:closedListPtr
               if closedList(i) == node
                   cont = True;
               end
           end
           
           if cont == True
               break
           end
           
           parents(node) = pre;
           
           %updating f, g and h vectors
           f(node) = nodeF;
           g(node) = nodeG;
           h(node) = nodeH;
           
           %pushing node onto open list
           openListPtr = openListPtr + 1;
           openList(openListPtr) = node;
      
        end
    end
end
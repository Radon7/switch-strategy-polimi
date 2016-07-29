%Disegna il grafo di adiacenza dell'ambiente
%Viene fornita: la matrice di adiacenza (adj)

function [] = plotEnvironment(adj)
    
    bg = biograph(tril(adj), [], 'ShowArrows','off', 'EdgeType', 'straight');
    
    for i = 1:length(adj)
        set(bg.nodes(i), 'Label', strcat('C', int2str(i)));
    end
    
    view(bg);
end
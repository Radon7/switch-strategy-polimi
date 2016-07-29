%Disegna il grafo che rappresenta le relazioni tra i robot
%Viene fornita: la matrice delle comunicazioni tra i robot (D)

function [] = plotRelations(D)
    bg = biograph(tril(D), [], 'ShowArrows','off', 'EdgeType', 'straight');
    
    for i = 1:length(D)
        set(bg.nodes(i), 'Label', strcat('R', int2str(i)));
    end
    
    view(bg);
end
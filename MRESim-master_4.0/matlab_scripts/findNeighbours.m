%Viene calcolata la lista dei vicini (list) di un particolare nodo (v).
%Vengono forniti: l'albero (G) e il nodo interessato (v).

function [list] = findNeighbours(G, v)
    list = [];
    
    for i = v:length(G)
        if G(i,v) == 1
            list = [list i];  
        end
    end
    for j = 1:v
        if G(v,j) == 1
            list = [list j];
        end
    end
end
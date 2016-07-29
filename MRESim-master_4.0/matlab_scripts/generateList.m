%Calcola la lista (list) contenenti i nodi adiacenti ad un nodo fornito (n)
%ed il nodo stesso.
%Vengono forniti: l'albero (G) e il nodo interessato (n).

function [list] = generateList(G,n)
    list = [];
    
    for i = n:length(G)
        if G(i,n) == 1
            list = [list i];  
        end
    end
    for j = 1:n
        if G(n,j) == 1
            list = [list j];
        end
    end
    
    list = [n list];
end
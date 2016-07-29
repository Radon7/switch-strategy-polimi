%Funzione che crea tutte le topologie di albero possibili.
%Passo G=(V,E) che rappresenta il grafo con robots ai vertici V e
%relazioni tra essi denotati dagli archi E (è un albero).
%Ritorna Te che è un vettore colonna contenente tutte le topologie di 
%albero ottenute a partire da G.

function [Te] = modifyTopology(G)
    Te = [];
    for i = 2 : length(G)
        for j = 1 : (i-1)
            if G(i,j) == 1
                T = buildTopology(tril(G), i, j);
                Te = [Te; T];
            end
        end
    end
end
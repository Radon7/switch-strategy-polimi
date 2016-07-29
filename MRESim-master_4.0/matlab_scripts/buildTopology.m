%Vengono costruite tutte le topologie di albero (T) rimuovendo un arco
%(m,k) da un albero G e sostituendolo con un arco (i,j) diverso da (m,k).
%L'arco (i,j) collega un nodo presente nel sotto-albero che contiene il
%nodo m con un nodo presente nel sotto-albero che contiene il nodo k.
%Vengono forniti: un albero di partenza (G) e i nodi di G collegati 
%all'arco da rimuovere (m e k).

function [T] = buildTopology (G, m, k)
    T = [];
    F = G;
    F(m,k) = 0;
    
    list_m = refineList(generateList(F,m), F);
    list_k = refineList(generateList(F,k), F);
    
    for i = 1:length(list_m)
        for j = 1:length(list_k)
            if ~(list_m(i) == m && list_k(j) == k)
                H = F;
                H(list_m(i),list_k(j)) = 1;
                H = tril(H + H');
                T = [T; H];
            end
        end
    end
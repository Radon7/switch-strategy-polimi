%Vengono calcolati il costo minimo (costMin) presente nella struttura di 
%costo totale (costs), l'indice del nodo corrente (vMin) corrispondente al 
%costo minimo e l'indice dell'albero (tMin) corrispondente al costo minimo.
%Viene fornita la struttura di costo di spostamento totale (costs).

function [costMin, vMin, tMin] = findMinCosts(costs)
     [vec, indeces] = min(costs);
     [costMin, index] = min(vec);
     vMin = index;
     tMin = indeces(index);
end
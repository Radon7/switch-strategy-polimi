%Vengono ritornati i costi minimi di spostamento verso le locazioni
%ottime presenti in m_2 calcolate precedentemente dalla funzione
%buildLocations.
%Vengono forniti: le locazioni ottime dei robot (m_2), la struttura dei
%costi robot/location aggiornata (q), i pesi virtuali (w), la topologia di
%albero scelta (T), il coefficiente mu, il nodo corrente (V_v), la radice
%dell'albero T (V_a) e l'insieme delle locations (C).

function [costs] = totalRedeploymentCosts(m_2, q, w, T, mu, V_v, V_a, C)
    
    costs = 0;
    
    rowV = [];
    
    N_T_v = findNeighbours(T, V_v);
    
    if isempty(V_a)
        for j = 1:length(C)
            rowV = [rowV q(V_v, j)];
        end
        minimum = min(rowV);
        costs = costs + minimum;
        
    else
        for j = 1:length(C)
            sum = q(V_v, j) + mu * w(m_2(V_a), j);
            rowV = [rowV sum];
        end
        minimum = min(rowV);
        costs = costs + minimum;
    end
    
    for n = 1:length(N_T_v)
        if isempty(V_a) || N_T_v(n) ~= V_a
            costs = costs + totalRedeploymentCosts(m_2, q, w, T, mu, N_T_v(n), V_v, C);
        end
    end
end
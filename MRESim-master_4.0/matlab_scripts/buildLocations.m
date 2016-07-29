%Vengono calcolate le locazioni ottime (m_opt) per ogni robot.
%Vengono forniti: le locazioni ottime dei robot (m_2), la struttura dei
%costi robot/location aggiornata (q), i pesi virtuali (w), la topologia di
%albero scelta (T), il coefficiente mu, il nodo corrente (V_v), la radice
%dell'albero T (V_a) e l'insieme delle locations (C).

function [m_fin] = buildLocations(m_2, q, w, T, mu, V_v, V_a, C)
        
    rowV = [];
    
    m_fin = m_2;
    
    if m_fin(V_a) == 0
        V_a = [];
    end

    N_T_v = findNeighbours(T, V_v);
    
    if isempty(V_a)
        for j = 1:length(C)
            rowV = [rowV q(V_v, C(j))];
        end
        [~, index] = min(rowV);
        m_fin(V_v) = C(index);
        
    else
        for j = 1:length(C)
            sum = q(V_v, C(j)) + mu * w(m_fin(V_a), C(j));
            rowV = [rowV sum];
        end
        [~, index] = min(rowV);
        m_fin(V_v) = C(index);
        
    end
    
    for n = 1:length(N_T_v)
        %Se V_a = [], allora N_T_v(n) ~= V_a restituisce [], quindi
        % ho dovuto inserire isempty(V_a) per coprire questo caso.
        if isempty(V_a) || N_T_v(n) ~= V_a
            m_fin = buildLocations(m_fin, q, w, T, mu, N_T_v(n), V_v, C);
        end
    end
end
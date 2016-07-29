%Ritorna la struttura dei costi robot/location aggiornata (q_2).
%Vengono forniti: la struttura dei costi robot/location inizializzata (q),
%la struttura dei costi virtuali (w), l'albero (T), il coefficiente mu, il
%nodo corrente (V_v), il nodo radice (V_a) e l'insieme delle locations (C).

function [q_2] = redeployment(q, w, T, mu, V_v, V_a, C)
    
    q_2 = q;
    
    N_T_v = findNeighbours(T,V_v);
    
    for j = 1:length(N_T_v)
        if N_T_v(j) ~= V_a
            q_2 = redeployment(q_2,w,T,mu,N_T_v(j),V_v, C);
            for k = 1:length(C)
                totalSum = [];
                for p = 1:length(C)
                    sum = q_2(N_T_v(j),C(p)) + mu*w(C(k),C(p));
                    totalSum = [totalSum sum];
                end
                q_2(V_v,C(k)) = q_2(V_v,C(k)) + min(totalSum);
            end
        end
    end
end
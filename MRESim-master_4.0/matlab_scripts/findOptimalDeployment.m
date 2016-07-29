%Vengono ritornati: le locazioni finali ottime dei robot (m_opt) e l'albero
%che rappresenta le relazioni di comunicazione tra i robot (T_opt).
%Vengono forniti: l'indice del nodo corrente (vMin) corrispondente al 
%costo minimo e l'indice dell'albero (tMin) corrispondente al costo minimo
%trovati precedentemente nella funzione findMinCosts, la struttura
%contenente tutte le possimili locazioni finali dei robot (M_2), la
%struttura contenente tutti gli alberi possibili (Te) costruiti
%precedentemente nella funzione buildTopology, il numero di robot (x).

function [m_opt, T_opt] = findOptimalDeployment(vMin, tMin, M_2, Te, x)
    m_opt = M_2(tMin, (vMin-1) * x + 1 : vMin * x);
    T_opt = Te((tMin-1) * x + 1 : tMin * x, : );
end
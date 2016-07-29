%Carico dati da un file esterno che contiene tutte le variabili di ingresso
%necessarie per il corretto funzionamento della funzione main.

load data.mat;

[costMin, m_opt, D_opt] = main(D, w_p, R, C, C_beta, V, V_beta, adj, G, m_1, mu);


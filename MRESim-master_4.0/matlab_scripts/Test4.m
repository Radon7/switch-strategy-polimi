%Mappa a ferro di cavallo, 4 robot.

G = [1 1 1 0 0 0 0;
     1 1 1 1 0 0 0;
     1 1 1 1 1 0 0;
     0 1 1 1 1 1 0;
     0 0 1 1 1 1 1;
     0 0 0 1 1 1 1;
     0 0 0 0 1 1 1];
 
 V = [1 2 3 4];
 C = [1 2 3 4 5 6 7];
 
 V_beta = [2 4];
 C_beta = [1 7];
 
 w_p = [0 1 2 2.4 3.4 3.8 4.8;
        1 0 1 1.4 2.4 2.8 3.8;
        2 1 0 1 2 2.4 3.4;
        2.4 1.4 1 0 1 1.4 2.4;
        3.4 2.4 2 1 0 1 2;
        3.8 2.8 2.4 1.4 1 0 1;
        4.8 3.8 3.4 2.4 2 1 0];
    
m_1 = [7 6 5 4];

R = 3.;

mu = 0.5;

D = [0 0 0 0;
     1 0 0 0;
     1 0 0 0;
     0 1 0 0];
 
 adj = [0 1 0 0 0 0 0;
        1 0 1 1 0 0 0;
        0 1 0 1 0 0 0;
        0 1 1 0 1 1 0;
        0 0 0 1 0 1 0;
        0 0 0 1 1 0 1;
        0 0 0 0 0 1 0];
 
[costMin, m_opt, D_opt] = main(D, w_p, R, C, C_beta, V, V_beta, adj, G, m_1, mu);
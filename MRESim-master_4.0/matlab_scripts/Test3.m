%Test finale, prova del MAIN

m_1 = [7 2 5 3 10 13];

V = [1 2 3 4 5 6];

C = [1 2 3 4 5 6 7 8 9 10 11 12 13 14];

C_beta = [7 12];

V_beta = [5 1];

w_p = [0. 1. 2. 2. 3. 4. 4.4 5.4 3. 3.4 4.4 5.4 6.4 6.8;
       1. 0. 1. 1. 2. 3. 3.4 4.4 2. 2.4 3.4 4.4 5.4 5.8;
       2. 1. 0. 1. 2. 3. 3.4 4.4 1. 1.4 2.4 3.4 4.4 4.8;
       2. 1. 1. 0. 1. 2. 2.4 3.4 2. 2.4 3.4 4.4 5.4 5.8;
       3. 2. 2. 1. 0. 1. 1.4 2.4 3. 3.4 4.4 5.4 6.4 6.8;
       4. 3. 3. 2. 1. 0. 1. 2. 4. 4.4 5.4 6.4 7.4 7.8;
       5. 4. 4. 3. 1.4 1. 0. 1. 4.4 4.8 5.8 6.8 7.8 8.2;
       6. 5. 5. 4. 2.4 2. 1. 0. 5.4 5.8 6.8 7.8 8.8 9.2;
       3. 2. 1. 2. 3. 4. 4.4 5.4 0. 1. 2. 3. 4. 4.4;
       3.4 2.4 1.4 2.4 3.4 4.4 4.8 5.8 1. 0. 1. 2. 3. 3.4;
       4.4 3.4 2.3 3.4 4.4 5.4 5.8 6.8 2. 1. 0. 1. 2. 2.4;
       5.4 4.4 3.4 4.4 5.4 6.4 6.8 7.8 3. 2. 1. 0. 1. 1.4;
       6.4 5.4 4.4 5.4 6.4 7.4 7.8 8.8 4. 3. 2. 1. 0. 1.;
       6.8 5.8 4.8 5.8 6.8 7.8 9.2 10.2 4.4 3.3 2.4 1.4 1. 0.];

G = [1 1 1 1 0 0 0 0 0 0 0 0 0 0; 
     1 1 1 1 0 0 0 0 0 0 0 0 0 0; 
     1 1 1 1 1 0 0 0 1 1 0 0 0 0; 
     1 1 1 1 1 1 0 0 1 0 0 0 0 0; 
     0 0 1 1 1 1 1 0 0 0 0 0 0 0; 
     0 0 0 1 1 1 1 1 0 0 0 0 0 0; 
     0 0 0 0 1 1 1 1 0 0 0 0 0 0; 
     0 0 0 0 0 1 1 1 0 0 0 0 0 0; 
     0 0 1 1 0 0 0 0 1 1 1 0 0 0; 
     0 0 1 0 0 0 0 0 1 1 1 1 0 0; 
     0 0 0 0 0 0 0 0 1 1 1 1 1 0; 
     0 0 0 0 0 0 0 0 0 1 1 1 1 1;
     0 0 0 0 0 0 0 0 0 0 1 1 1 1; 
     0 0 0 0 0 0 0 0 0 0 0 1 1 1];

R = 3.; 

D = [0 0 1 0 0 0;
     0 0 1 0 0 0;
     1 1 0 1 0 0;
     0 0 1 0 1 0;
     0 0 0 1 0 1;
     0 0 0 0 1 0];
 
D_2 = [0 0 0 0 0 0;
     0 0 0 0 0 0;
     1 0 0 0 0 0;
     0 1 1 0 0 0;
     0 0 0 1 0 0;
     0 0 0 0 1 0];

mu = 0.5;

adj = [0 1 0 0 0 0 0 0 0 0 0 0 0 0; 
       1 0 1 1 0 0 0 0 0 0 0 0 0 0; 
       0 1 0 1 0 0 0 0 1 1 0 0 0 0; 
       0 1 1 0 1 0 0 0 0 0 0 0 0 0; 
       0 0 0 1 0 1 1 0 0 0 0 0 0 0; 
       0 0 0 0 1 0 1 0 0 0 0 0 0 0; 
       0 0 0 0 1 1 0 1 0 0 0 0 0 0; 
       0 0 0 0 0 0 1 0 0 0 0 0 0 0; 
       0 0 1 0 0 0 0 0 0 1 0 0 0 0; 
       0 0 1 0 0 0 0 0 1 0 1 0 0 0; 
       0 0 0 0 0 0 0 0 0 1 0 1 0 0; 
       0 0 0 0 0 0 0 0 0 0 1 0 1 1;
       0 0 0 0 0 0 0 0 0 0 0 1 0 1; 
       0 0 0 0 0 0 0 0 0 0 0 1 1 0];

[costMin, m_opt, D_opt] = main(D, w_p, R, C, C_beta, V, V_beta, adj, G, m_1, mu);
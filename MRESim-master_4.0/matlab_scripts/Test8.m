%Carico i dati da vari file .txt e salvo i dati su vari file .txt

D = dlmread('C:\\Users\\Mattia\\workspace\\MRESim-master\\D.txt');
w_p = dlmread('C:\\Users\\Mattia\\workspace\\MRESim-master\\w_p.txt');
R = dlmread('C:\\Users\\Mattia\\workspace\\MRESim-master\\R.txt');
C = dlmread('C:\\Users\\Mattia\\workspace\\MRESim-master\\C.txt');
C_beta = dlmread('C:\\Users\\Mattia\\workspace\\MRESim-master\\C_beta.txt');
V = dlmread('C:\\Users\\Mattia\\workspace\\MRESim-master\\V.txt');
V_beta = dlmread('C:\\Users\\Mattia\\workspace\\MRESim-master\\V_beta.txt');
adj = dlmread('C:\\Users\\Mattia\\workspace\\MRESim-master\\adj.txt');
G = dlmread('C:\\Users\\Mattia\\workspace\\MRESim-master\\G.txt');
m_1 = dlmread('C:\\Users\\Mattia\\workspace\\MRESim-master\\m_1.txt');
mu = dlmread('C:\\Users\\Mattia\\workspace\\MRESim-master\\mu.txt');

[costMin, m_opt, D_opt] = main(D, w_p, R, C, C_beta, V, V_beta, adj, G, m_1, mu);

dlmwrite('C:\\Users\\Mattia\\workspace\\MRESim-master\\costMin.txt',costMin,'delimiter','\t');
dlmwrite('C:\\Users\\Mattia\\workspace\\MRESim-master\\m_opt.txt',m_opt,'delimiter','\t');
dlmwrite('C:\\Users\\Mattia\\workspace\\MRESim-master\\D_opt.txt',D_opt,'delimiter','\t', 'newline', 'pc');
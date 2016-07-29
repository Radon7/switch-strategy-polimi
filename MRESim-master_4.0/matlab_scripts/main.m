%Ritorna la struttura che rappresenta le relazioni di comunicazione dei
%robot (D_opt) a costo minimo, le locazioni finali dei robot (m_opt) a
%costo minimo e il costo minimo totale di spostamento dei robot (costMin).
%Vengono forniti: la struttura che rappresenta le relazioni di
%comunicazione iniziali tra i robot (D), il costo di spostamento fisico tra
%le locations (w_p), il raggio di comunicazione (R), l'insieme di tutte le
%locazioni (C), l'insieme delle locazioni "target" (C_beta), l'insieme di
%tutti i robot (V), l'insieme di tutti i robot che devono raggiungere le
%locazioni target (V_beta), il mutual visibility graph (G) che rappresenta 
%l'ambiente, la matrice di adiacenza dell'ambiente (adj), le locazioni 
%iniziali dei robot (m_1) ed il coefficiente mu.

function [costMin, m_opt, D_opt] = main()
    
    root_folder = getenv('STUMP_FILES_PATH'); % environment variable for setting the path where to find the .txt files and where to save the results of the computation NOTE: add / at the end of the path
    if (isempty(root_folder) == 0) && (strcmp(root_folder(end), '/') == 0)
        root_folder = strcat(root_folder, '/');
    end
    D = importdata(strcat(root_folder, 'D.txt'),'\t');
    w_p = importdata(strcat(root_folder, 'w_p.txt'),'\t');
    R = importdata(strcat(root_folder,'R.txt'));
    C = importdata(strcat(root_folder,'C.txt'),'\t');
    C_beta = importdata(strcat(root_folder,'C_beta.txt'),'\t');
    V = importdata(strcat(root_folder,'V.txt'),'\t');
    V_beta = importdata(strcat(root_folder,'V_beta.txt'),'\t');
    G = importdata(strcat(root_folder,'G.txt'),'\t');
    m_1 = importdata(strcat(root_folder,'m_1.txt'),'\t');
    mu = importdata(strcat(root_folder,'mu.txt'));


    %D = str2num(D_s);
    %w_p = str2num(w_p_s);
    %R = str2num(R_s);
    %C = str2num(C_s);
    %C_beta = str2num(C_beta_s);
    %V = str2num(V_s);
    %V_beta = str2num(V_beta_s);
    %G = str2num(G_s);
    %m_1 = str2num(m_1_s);
    %mu = str2num(mu_s);
    
    %adj = str2num(adj_s);
    
    %plotEnvironment(adj);
    
    %plotRelations(D);
    
    %plotDeployment(adj, D, m_1);
    
    
    Te = modifyTopology(tril(D));
    
    [y, x] = size(Te);
    c = y/x;
    
    w = buildWeights(G,R,C,w_p);
    
    Q = [];
    M_2 = [];
    costs = [];
    m_opt = [];
    D_opt = [];
    
    for i = 1:c
        
        T = generateTree(Te, x, i);
        
        Q_2 = []; 
        M_2_fin = [];
        
        for j = 1:length(V)
            
            q = [];
            q_2 = [];
            m_2 = [];
            m_2_fin = [];
            
            V_v = V(j);

            V_a = V_v;
            
            q = initializeRedeployment(m_1, C_beta, w_p, C, V, V_beta);

            q_2 = redeployment(q, w, T, mu, V_v, V_a, C);

            Q_2 = [Q_2 q_2];
            
            m_2 = generateOutput(C_beta, V_beta, V);
            
            m_2_fin = buildLocations(m_2, q_2, w, T, mu, V_v, V_a, C);

            M_2_fin = [M_2_fin m_2_fin];
            
            costs(i,j) = totalRedeploymentCosts(m_2_fin, q_2, w, T, mu, V_v, V_a, C);
            
        end        
        
        Q = [Q; Q_2];
        
        M_2 = [M_2; M_2_fin];
        
    end

    [costMin, vMin, tMin] = findMinCosts(costs);
    
    if costMin == inf
        fprintf('Non è stata trovata alcuna struttura che permetta di collegare i target indicati. \n');
    else
        [m_opt, D_opt] = findOptimalDeployment(vMin, tMin, M_2, Te, x);
        fprintf('La struttura di comunicazione di costo minimo è: \n');
        %plotRelations(D_opt);
        fprintf('\nLe locations finali dei robot sono: \n');
        %plotDeployment(adj, D_opt, m_opt);
        fprintf('\nIl costo minimo totale di spostamento è: \n');
        costMin
    end
    
    dlmwrite(strcat(root_folder,'costMin.txt'),costMin,'delimiter','\t');
    dlmwrite(strcat(root_folder,'m_opt.txt'),m_opt,'delimiter','\t');
    dlmwrite(strcat(root_folder,'D_opt.txt'),D_opt,'delimiter','\t', 'newline', 'pc');
      
end

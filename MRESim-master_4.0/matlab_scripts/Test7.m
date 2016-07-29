 
  adj = [0 0 0 0 0 0 0 0;
         1 0 0 0 0 0 0 0;
         1 1 0 0 0 0 0 0;
         0 0 1 0 0 0 0 0;
         0 0 1 1 0 0 0 0;
         0 0 0 0 1 0 0 0;
         0 0 0 0 1 1 0 0;
         0 0 0 0 0 0 1 0];
  
plotEnvironment(adj);


D = [0 1 1 1;
     1 0 0 0;
     1 0 0 0;
     1 0 0 0];
 
 plotRelations(D);
 
  m = [2 4 6 8];
 
 plotDeployment(adj, D, m);

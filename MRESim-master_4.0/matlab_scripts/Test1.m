%Test per:
%   - modifyTopology
%   - buildTopology
%   - generateList
%   - refineList
%   - findNeighbours

G = [0 0 0 0 0 0;
     1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 0 0 0 0;
     0 1 0 1 0 0;
     1 0 0 0 0 0];
 
 D = [0 0 1 0 0 0;
      0 0 1 0 0 0;
      1 1 0 1 0 0;
      0 0 1 0 1 0;
      0 0 0 1 0 1;
      0 0 0 0 1 0];

H = tril(G);

H(2,1) = 0;

list = generateList(H,3);
finalList = refineList(list, H);

T_1 = [0     0     0     0     0     0
       0     0     0     0     0     0
       0     1     0     0     0     0
       1     0     1     0     0     0
       0     0     0     1     0     0
       0     0     0     0     1     0];


list_1 = generateList(T_1,5);
finalList_1 = refineList(list_1, T_1);

T = buildTopology(H, 2, 1);

Te = modifyTopology(G);

Te_2 = modifyTopology(D);

N_T_v = findNeighbours(T_1,6);
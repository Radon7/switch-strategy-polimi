%Disegna il deployment dei robot nell'ambiente
%Viene fornita: la matrice di adiacenza (adj), la matrice delle relazioni
%tra i robot (D) e il vettore del deployment dei robot (m)

function [] = plotDeployment(adj, D, m)
    adj_1 = adj;

    for i = 1:length(D)
        for j = 1:i
            if D(i,j) == 1
                adj_1(m(i), m(j)) = 1;
            end
        end
    end
    
    adj_1 = adj_1 + adj_1';
    
    bg = biograph(tril(adj_1), [], 'ShowArrows','off', 'EdgeType', 'curved');
    
    for i = 1:length(adj_1)
        set(bg.nodes(i), 'Label', strcat('C', int2str(i)));
        
        for j = 1:length(m)
            if i == m(j)
                label = strcat(get(bg.nodes(i), 'Label'), ' + ', 'R', int2str(j));
                set(bg.nodes(i), 'Label', label);
            end
        end
    end
    
    
    for i = 1:length(m)
        for j = 1:length(m)
            if i ~= j && (D(i,j) == 1 || D(j,i) == 1)
                k = getedgesbynodeid(bg, get(bg.nodes(m(i)),'ID'), get(bg.nodes(m(j)),'ID'));
                
                set(k, 'LineColor', [0.7 0.0 0.1], 'LineWidth', 2.0);
            end
        end
    end
    
    view(bg);
end
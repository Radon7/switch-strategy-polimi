%Ritorna una lista dei vicini (finalList) a nodi contenuti in una lista
%(list).
%Vengono forniti: la lista dei nodi interessata (list) e un albero (G)
%contenente i nodi della lista.

function [finalList] = refineList(list, G)
    finalList = list;
    midList = [];
    k = length(finalList);
    j=0;
    while(j < k)
        midList = generateList(G, finalList(j+1));
        
        for i = 1:length(midList)
            if ismember(midList(i), finalList) == 0
                finalList = [finalList midList(i)];
            end
        end
        
        k = length(finalList);
        j = j+1;
    end
end
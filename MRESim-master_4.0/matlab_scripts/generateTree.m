%Ritorna l'albero (T) che si trova alla posizione k all'interno della
%struttura Te.
%Vengono forniti: la struttura contenente tutti gli alberi (Te), il numero
%dei robot (x) e la posizione dell'albero di interesse (k).

function [T] = generateTree(Te, x, k)
    T = [];
    m = 1;
    for i = (k-1)*x+1 : k*x
        T(m,:) = Te(i,:);
        m = m+1;
    end
end
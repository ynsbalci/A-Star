function testAstar = testAstar()

% rastgele olarak �rnek olu�turma
% List = ones(100);
% for k = 1:length(List)
%     for l = 1:length(List)
%         List(k,l) = randi([1 4]);
%         if List(k, l) ~= 4
%             List(k, l) = 1;
%         end
%     end 
% end
% save('matrix.mat','List')

%�nceki kayd� y�kleme
load('matrix.mat','List')

% �rnek �al��ma alnalr�n� kendin�izde olu�rurabilrisniz
% matriste 1 olan birimler geziebilir alan ve 4 olan alanlar gezilemez alan(duvar) olaark alg�lan�r

Astar(1, 1, 100, 100, List)

end
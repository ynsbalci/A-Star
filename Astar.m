%Yunus BALCI AStar Yol Bulma Algoritams� 2017
%
%   Yol bulma fonksiyonu. Verieln Matriste ba�lang��tan hedefe en yak�n
%   mesafeyi bululr.
%
%   De�i�kenler:
%   startX, startY: ba�lang�� noktas�n�n Xve Y koordinatlar�
%   targetX, targetY: hedef noktan�n X ve Y koordinatlar�
%   List: arama yap�lacak matris(n,n boyutta)
%
%

function Astar(startX, startY, targetX, targetY, List)

    tic
    
    
    %map boyutlar�n� ayarlama
    mapWidth = size(List, 1);
    mapHeight = size(List, 1);
    
    parentX = zeros(mapWidth,mapHeight);        % her d���m i�in �st X koordinatlar�n� tutar
    parentY = zeros(mapWidth,mapHeight);        % her d���m i�in �st X koordinatlar�n� tutar
    Gcost = zeros(mapWidth,mapHeight);          % her bir d���m�n G(Ba�lang�� d���m�nden mevcut d���me kadar gelmenin maliyeti) maliyetini tutar
    Hcost = zeros(1,mapWidth*mapHeight);        % her bir d���m�n H(Mevcut d���mden hedef d���me varmak i�in tahmin edilen mesafe.) maliyetini tutar
    pathlength = 0;                             % yol uzunlu�u                

    %de�ikenlerin olu�turulmas�
    newOpenListItemID = 0;                      %
    parentXval = 0;                             % 
    parentYval = 0;                             %
    a = 0; b = 0; m = 0; u = 0; v = 0;          %
    temp = 0;                                   %
    corner = 0;                                 %
    addedGCost = 0;
    tempGCost = 0;
    path = 0;
    
    % Sabit de�i�kenler
    walkable = 1;                               %
    onOpenList = 2;                             %
    onClosedList = 3;                           %
    unwalkable = 4;                             %
    found = 1;                                  %
    nonexistent = 2;                          %
    
    % engellerin say�s�n� sayma
    NumberOfObstacles = 0;
    for k = 1:length(List)
        for l = 1:length(List)
            if List(k, l) == 4
             NumberOfObstacles = NumberOfObstacles+1;
            end
        end 
    end
    
    
    fprintf('------------------------------- \n');
    fprintf('##### Astar Yol Bulma ##### \n');
    fprintf('Ba�lang��: %d, %d \n', startX, startY);
    fprintf('Hedef: %d, %d \n \n', targetX, targetY);
    fprintf('------------------------------- \n');
    fprintf('Ba�lang��:\n');
    disp(List);
    
    % Ba�lang�� ve biti� koordinatlar�n�n ayn� olup olmad���n� kontrol etme
    if ( (startX == targetX) && (startY == targetY) )
        disp('Ba�lang�� ve Biti� noktas� ayn� olamaz');
        return;
    end
    
    %gezilebilir listesine ba�lang�� noktas�n� ekleme
    numberOfOpenListItems = 1;
    openList(1) = 1;
    openX(1) = startX;
    openY(1) = startY;
   
    
    % sonu� buluncaya kadar sonsuz bir d�g� olu�turma
    while (1)
        
        %similasyon i�in buras� a��lacak (hessaplaam s�resi uzar)
        %pause(0.0000001);
        %disp(List);
        %imagesc(List);

        % gezilebilir olup olmad���n� kontrol etme
        if (numberOfOpenListItems ~= 0)

            parentXval = openX(openList(1));
            parentYval = openY(openList(1));

            List(parentXval, parentYval) = onClosedList;
             
            
            % ilk eleman� yerle�tirme
            openList(1) = openList(numberOfOpenListItems);
            
            % gezilebilr listesinden yerle�tirlen eleman� ��karma
            numberOfOpenListItems = numberOfOpenListItems - 1;
            
            % ilk ��e hari� ge�ici bir kopya olu�turma
            tempOpenList = openList(1:numberOfOpenListItems);
            
            % gezilebilir listesini s�f�rlama
            clear openList;
            
            openList = tempOpenList;
            clear tempOpenList;
            
            v = 1;
            % maliyeet g�re uygun alan yerle�tirme
            while (1)
                u = v;

                if (2*u+1 <= numberOfOpenListItems)
                    % parent �n F maliyetini her �ocu�una g�re kontrol etme
                    if (Fcost(openList(u)) >= Fcost(openList(2*u)))
                        v = 2*u;
                    end
                    if (Fcost(openList(v)) >= Fcost(openList(2*u+1)))
                        v = 2*u+1;
                    end
                %1 tane �acu�u varsa  
                elseif (2*u <= numberOfOpenListItems)
                    % F maliyetinin oaremt �ndan k���k olup olmad���na
                    % bakma
                    if (Fcost(openList(u)) >= Fcost(openList(2*u)))
                        v = 2*u;
                    end
                end
                
                % 
                if (u ~= v)
                    temp = openList(u);
                    openList(u) = openList(v);
                    openList(v) = temp;
                else
                    break;
                end
            end

            
            % kom�ular� kontrol etme
            for a=(parentXval-1):(parentXval+1),
                for b=(parentYval-1):(parentYval+1),
                    % 
                    if ( (a > 0) && (b > 0) && (a <= mapWidth) && (b <= mapHeight) )
                        % 
                        if (List(a,b) ~= onClosedList)
                            % 
                            if (List(a,b) ~= unwalkable)
                                
                                corner = walkable;
                                % 
                                if (a == parentXval-1)
                                    if (b == parentYval-1)
                                        if ( (List(parentXval-1,parentYval) == unwalkable) || ...
                                                (List(parentXval,parentYval-1) == unwalkable) )
                                            corner = unwalkable;
                                        end
                                    elseif (b == parentYval+1)
                                        if ( (List(parentXval,parentYval+1) == unwalkable) || ...
                                                (List(parentXval-1,parentYval) == unwalkable) )
                                            corner = unwalkable;
                                        end
                                    end
                                elseif (a == parentXval+1)
                                    if (b == parentYval-1)
                                        if ( (List(parentXval,parentYval-1) == unwalkable) || ...
                                                (List(parentXval+1, parentYval) == unwalkable) )
                                            corner = unwalkable;
                                        end
                                    elseif (b == parentYval+1)
                                        if ( (List(parentXval+1,parentYval) == unwalkable) || ...
                                                (List(parentXval,parentYval+1) == unwalkable) )
                                            corner = unwalkable;
                                        end
                                    end
                                end
                                
                                % k��elerin gezilebilirlik durumunu kontrol
                                % etme
                                if (corner == walkable)
                                    %gezilebilr listesinde yoksa ekleme
                                    if (List(a,b) ~= onOpenList)
                                        % yeni gezilebilir listesi
                                        % olu�turma
                                        newOpenListItemID = newOpenListItemID + 1;
                                        m = numberOfOpenListItems + 1;
                                        openList(m) = newOpenListItemID;
                                        openX(newOpenListItemID) = a;
                                        openY(newOpenListItemID) = b;
                                        
                                        
                                        % G maliyetlerini hesaplama
                                        if ( (abs(a - parentXval) == 1) && (abs(b - parentYval) == 1) )
                                            addedGCost = 14;
                                        else
                                            addedGCost = 10;
                                        end
                                        % G maliyetlerini g�ncelleme
                                        Gcost(a,b) = Gcost(parentXval,parentYval) + addedGCost;
                                        
                                        % H ve F maliyetlerini hesaplama
                                        Hcost(openList(m)) = 10*(abs(a - targetX) + abs(b - targetY));
                                        Fcost(openList(m)) = Gcost(a,b) + Hcost(openList(m));
                                        parentX(a,b) = parentXval;
                                        parentY(a,b) = parentYval;
                                        
                                        % 
                                        while (m ~= 1)
                                            % 
                                            if (Fcost(openList(m)) <= Fcost(openList(round(m/2))))
                                                temp = openList(round(m/2));
                                                openList(round(m/2)) = openList(m);
                                                openList(m) = temp;
                                                m = round(m/2);
                                            else
                                                break;
                                            end
                                        end
                                        
                                        % gezilebilr listeynin bir
                                        % sonras�na ge�me
                                        numberOfOpenListItems = numberOfOpenListItems + 1;
                                        % mevcut d���m� gezilebir listye
                                        % atama
                                        List(a,b) = onOpenList;
                                    end
                                else
                                    %
                                end
                            end
                        else % if List(a,b) = onOpenList
                            % G maliyetini hesaplama
                            if ( (abs(a - parentXval) == 1) && (abs(b - parentYval) == 1) )
                            	addedGCost = 14;
                            else
                            	addedGCost = 10;
                            end
                            tempGcost = Gcost(parentXval,parentYval) + addedGCost;
                            
                            % bu yol daha k�sa ise
                            if (tempGcost < Gcost(a,b)),
                                parentX(a,b) = parentXval;
                                parentY(a,b) = parentYval;
                                Gcost(a,b) = tempGcost;
                                
                                % G maliyetinin de�i�tirilmesi F maliyetini de de�i�tirir, gezilebilir g�ncellenmesi ve yeniden s�ralanmas� gerekir
                                for x=1:numberOfOpenListItems,
                                    if ( (openX(openList(x)) == a) && (openY(openList(x)) == b) )
                                        % F maliyetini de��itirme
                                        Fcost(openList(x)) = Gcost(a,b) + Hcost(openList(x));
                                        
                                        %listeyi yeniden d�zenleem
                                        m = x;
                                        while (m ~= 1)
                                            % 
                                            if (Fcost(openList(m)) < Fcost(openList(round(m/2))))
                                                temp = openList(round(m/2));
                                                openList(round(m/2)) = openList(m);
                                                openList(m) = temp;
                                                m = round(m/2);
                                            else
                                                break;
                                            end
                                        end
                                        % bulunca listeden ��kma
                                        break;
                                    end
                                end
                            end    
                            
                        end
                    end
                end
            end
                                
        % gezilebilr liste bo�sa 
        else
            path = nonexistent;
            % ba�ar�s�z ise
            fprintf('Hedef yol bulunamd� \n');
            fprintf('Toplam engel say�s�: %d\n', NumberOfObstacles);
            imagesc(List);
            break;
        end
        %Hedef gezilebilir listeye eklenirse, yol bulundu
        if (List(targetX,targetY) == onOpenList)
            path = found;
            %fprintf('yol bulundu! \n');
            break;
        end
        
    end
    
    % Yol bulunduysa
    if (path == found)
        pathLength = 0;
        % yolu geri yollama
        pathX = targetX;
        pathY = targetY;
        % geri izleme
        %fprintf('En k�sa rotay� bulmak i�in geri izleme', );
        

        % ba�lang�� konumuna ula��ncaya kadar
        while(1)
            % 
            tempx = parentX(pathX,pathY);
            pathY = parentY(pathX,pathY);
            pathX = tempx;
            

            pathLength = pathLength + 1;
            
            
            % Ba�lang�� konumu ula�t�ysa, d�ng�den ��k
            if ( (pathX == startX) && (pathY == startY) )
                break;
            end
        end
        fprintf('------------------------------- \n');
        fprintf('Sonu�: \n');
        disp(List);
        fprintf('------------------------------- \n');
        fprintf('Toplam ad�m say�s�: %d\n', pathLength);
        fprintf('Toplam engel say�s�: %d\n', NumberOfObstacles);
        toc
        imagesc(List);
    end

end

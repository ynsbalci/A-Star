%Yunus BALCI AStar Yol Bulma Algoritamsý 2017
%
%   Yol bulma fonksiyonu. Verieln Matriste baþlangýçtan hedefe en yakýn
%   mesafeyi bululr.
%
%   Deðiþkenler:
%   startX, startY: baþlangýç noktasýnýn Xve Y koordinatlarý
%   targetX, targetY: hedef noktanýn X ve Y koordinatlarý
%   List: arama yapýlacak matris(n,n boyutta)
%
%

function Astar(startX, startY, targetX, targetY, List)

    tic
    
    
    %map boyutlarýný ayarlama
    mapWidth = size(List, 1);
    mapHeight = size(List, 1);
    
    parentX = zeros(mapWidth,mapHeight);        % her düðüm için üst X koordinatlarýný tutar
    parentY = zeros(mapWidth,mapHeight);        % her düðüm için üst X koordinatlarýný tutar
    Gcost = zeros(mapWidth,mapHeight);          % her bir düðümün G(Baþlangýç düðümünden mevcut düðüme kadar gelmenin maliyeti) maliyetini tutar
    Hcost = zeros(1,mapWidth*mapHeight);        % her bir düðümün H(Mevcut düðümden hedef düðüme varmak için tahmin edilen mesafe.) maliyetini tutar
    pathlength = 0;                             % yol uzunluðu                

    %deðikenlerin oluþturulmasý
    newOpenListItemID = 0;                      %
    parentXval = 0;                             % 
    parentYval = 0;                             %
    a = 0; b = 0; m = 0; u = 0; v = 0;          %
    temp = 0;                                   %
    corner = 0;                                 %
    addedGCost = 0;
    tempGCost = 0;
    path = 0;
    
    % Sabit deðiþkenler
    walkable = 1;                               %
    onOpenList = 2;                             %
    onClosedList = 3;                           %
    unwalkable = 4;                             %
    found = 1;                                  %
    nonexistent = 2;                          %
    
    % engellerin sayýsýný sayma
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
    fprintf('Baþlangýç: %d, %d \n', startX, startY);
    fprintf('Hedef: %d, %d \n \n', targetX, targetY);
    fprintf('------------------------------- \n');
    fprintf('Baþlangýç:\n');
    disp(List);
    
    % Baþlangýç ve bitiþ koordinatlarýnýn ayný olup olmadýðýný kontrol etme
    if ( (startX == targetX) && (startY == targetY) )
        disp('Baþlangýç ve Bitiþ noktasý ayný olamaz');
        return;
    end
    
    %gezilebilir listesine baþlangýç noktasýný ekleme
    numberOfOpenListItems = 1;
    openList(1) = 1;
    openX(1) = startX;
    openY(1) = startY;
   
    
    % sonuç buluncaya kadar sonsuz bir dögü oluþturma
    while (1)
        
        %similasyon için burasý açýlacak (hessaplaam süresi uzar)
        %pause(0.0000001);
        %disp(List);
        %imagesc(List);

        % gezilebilir olup olmadýðýný kontrol etme
        if (numberOfOpenListItems ~= 0)

            parentXval = openX(openList(1));
            parentYval = openY(openList(1));

            List(parentXval, parentYval) = onClosedList;
             
            
            % ilk elemaný yerleþtirme
            openList(1) = openList(numberOfOpenListItems);
            
            % gezilebilr listesinden yerleþtirlen elemaný çýkarma
            numberOfOpenListItems = numberOfOpenListItems - 1;
            
            % ilk öðe hariç geçici bir kopya oluþturma
            tempOpenList = openList(1:numberOfOpenListItems);
            
            % gezilebilir listesini sýfýrlama
            clear openList;
            
            openList = tempOpenList;
            clear tempOpenList;
            
            v = 1;
            % maliyeet göre uygun alan yerleþtirme
            while (1)
                u = v;

                if (2*u+1 <= numberOfOpenListItems)
                    % parent ýn F maliyetini her çocuðuna göre kontrol etme
                    if (Fcost(openList(u)) >= Fcost(openList(2*u)))
                        v = 2*u;
                    end
                    if (Fcost(openList(v)) >= Fcost(openList(2*u+1)))
                        v = 2*u+1;
                    end
                %1 tane çacuðu varsa  
                elseif (2*u <= numberOfOpenListItems)
                    % F maliyetinin oaremt ýndan küçük olup olmadýðýna
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

            
            % komþularý kontrol etme
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
                                
                                % köþelerin gezilebilirlik durumunu kontrol
                                % etme
                                if (corner == walkable)
                                    %gezilebilr listesinde yoksa ekleme
                                    if (List(a,b) ~= onOpenList)
                                        % yeni gezilebilir listesi
                                        % oluþturma
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
                                        % G maliyetlerini güncelleme
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
                                        % sonrasýna geçme
                                        numberOfOpenListItems = numberOfOpenListItems + 1;
                                        % mevcut düðümü gezilebir listye
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
                            
                            % bu yol daha kýsa ise
                            if (tempGcost < Gcost(a,b)),
                                parentX(a,b) = parentXval;
                                parentY(a,b) = parentYval;
                                Gcost(a,b) = tempGcost;
                                
                                % G maliyetinin deðiþtirilmesi F maliyetini de deðiþtirir, gezilebilir güncellenmesi ve yeniden sýralanmasý gerekir
                                for x=1:numberOfOpenListItems,
                                    if ( (openX(openList(x)) == a) && (openY(openList(x)) == b) )
                                        % F maliyetini deðþitirme
                                        Fcost(openList(x)) = Gcost(a,b) + Hcost(openList(x));
                                        
                                        %listeyi yeniden düzenleem
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
                                        % bulunca listeden çýkma
                                        break;
                                    end
                                end
                            end    
                            
                        end
                    end
                end
            end
                                
        % gezilebilr liste boþsa 
        else
            path = nonexistent;
            % baþarýsýz ise
            fprintf('Hedef yol bulunamdý \n');
            fprintf('Toplam engel sayýsý: %d\n', NumberOfObstacles);
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
        %fprintf('En kýsa rotayý bulmak için geri izleme', );
        

        % baþlangýç konumuna ulaþýncaya kadar
        while(1)
            % 
            tempx = parentX(pathX,pathY);
            pathY = parentY(pathX,pathY);
            pathX = tempx;
            

            pathLength = pathLength + 1;
            
            
            % Baþlangýç konumu ulaþtýysa, döngüden çýk
            if ( (pathX == startX) && (pathY == startY) )
                break;
            end
        end
        fprintf('------------------------------- \n');
        fprintf('Sonuç: \n');
        disp(List);
        fprintf('------------------------------- \n');
        fprintf('Toplam adým sayýsý: %d\n', pathLength);
        fprintf('Toplam engel sayýsý: %d\n', NumberOfObstacles);
        toc
        imagesc(List);
    end

end

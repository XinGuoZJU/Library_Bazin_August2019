function Fn_BnB_DisplayBoundsOfValidCubes(ListCubes,IndexFlag,IndicesBounds)

% inputs:
%   - ListCubes: the list of rotation cubes
%   - IndexFlag: the index of the flag in ListCubes. Used to detect the invalid cubes
%   - IndicesBounds: the indices of the lower and upper bouds in ListCubes

% select the valid cubes (their flag equals 1)
IndicesValid=find(ListCubes(:,IndexFlag)==1);
    ListCubesVALID=ListCubes(IndicesValid,:);

    % sort them according to their differences for a clearer display
    [temp,indices] = sort(ListCubesVALID(:,IndicesBounds(2))-ListCubesVALID(:,IndicesBounds(1)));
    
    % display
    figure('name','sorted diff of lower and upper bounds of the valid cubes in the list')
    plot(ListCubesVALID(indices,IndicesBounds(1)),'.-r'); % lower bound
    hold on;
    plot(ListCubesVALID(indices,IndicesBounds(2)),'.-g'); % upper bound
    hold off;
    legend('lb','ub')
    
    drawnow; 
    fprintf('Nb of valid indices/cubes: %d and number of cubes in the list:%d \n', length(IndicesValid),  size(ListCubes,1));
    
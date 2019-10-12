function my_list=Fn_RandomList_BoundedLimit(nb_points, my_low_limit, my_high_limit)

% output:
%   - my_list: 1xN

% formula obtained by linear interpolation: (0~1)-->(low~high)
my_list= rand(1,nb_points)*(my_high_limit-my_low_limit)+my_low_limit; 
%my_list=round(my_list);


function execute_path(lidar)

points = [  -1.25   -1      ;
            -1.25   1       ;
            0       2.25    ;
            1.25    1       ;
            1.25    -1      ;
            0       -2.25   ;
            0       0       ;
            0       1       ;
            0       -1      ];
A = 8;
B = 9;

order = [1 B 3 6 A 4 5 B 7]; % Order is 1 3 6 4 5 7
%order = [1 2 A 3 6 A 4 A B 5 B 7];
%order = [5 B A];
start = points(order(1),:);
route = [];
for i = 2:length(order)
    route = [route;points(order(i),:)];
end
pd_controller(start,route,lidar)
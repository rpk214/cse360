function execute_path(lidar)

points = [  -1.25   -1      ;
            -1.25   1       ;
            0       2.25    ;
            1.25    1       ;
            1.25    -1      ;
            0       -2.25   ;
            0       0       ];

order = [1 5 4 5 1 2];
start = points(order(1),:);
route = [];
for i = 2:length(order)
    route = [route;points(order(i),:)];
end
pd_controller(start,route,lidar)
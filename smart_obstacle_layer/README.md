# smart_obstacle_layer
This is a movebase costmap layer, it's developed by mwu412
It's mean to replace origianl obstacle_layer.
Original obstacle layer use ray tracing to "memorize" obstacle at costmap,
However, this strategy would lead to crowd obstacle all over costmap,
Espacially bad when localization has jump from one place to another.
smart_obstacle_layer get rid of "memorize obstacle" function, makes it perform better in the most of time.

It also include a new function that ignore laser point near AMR. Radius of the ignore circle can be specified by dynamic reconfiguration.

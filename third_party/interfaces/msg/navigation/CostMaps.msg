# 代价地图序列
# maps[0] 为当前帧膨胀后的代价图
# 如果 maps 长度 > 1，maps[i] 为 t_0 + i * prediction_dt 时刻的预测代价图

float64 prediction_dt
nav_msgs/OccupancyGrid[] maps

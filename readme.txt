1.SetLidarParameters.m
作用：设置激光雷达的扫描参数（扫描角度范围，扫描距离范围，角度增量等）

2.ReadAScan.m
作用：将激光雷达数据从极坐标转化为以机器人为中心的笛卡尔坐标（pol2cart）

3.Initialize.m
作用：第一次数据处理，把扫描数据scan从局部坐标 通过位姿pose 转换为全局地图map坐标（points和keypoints信息）

4.Transform.m
作用：把局部坐标转化为全局坐标（旋转和平移）

5.ExtractLocalMap.m
作用：在全局地图中找到当前激光数据附近一米距离的的点为局部地图

6.OccuGrid.m   
作用：根据局部地图点集创建栅格地图（返回值为翻转后的栅格地图）

7.AddKeyScan.m
作用：为地图map加入新的数据（增量式hits>1.1）

8.DiffPose.m
作用：求位姿差

9.FastMatch.m
作用：根据当前位姿的栅格地图 优化预测的下一位姿 使下一位姿的栅格地图与当前位姿的栅格地图达到最大的重合度

10.PlotMap.m
作用：绘制地图、路径、雷达扫描线(每次都清空)



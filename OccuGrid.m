% 从点集创建占用栅格地图
% Create an occupancy grid map from points
function gridmap = OccuGrid(pts, pixelSize)
%--------------------------------------------------------------------------
%输入
%   pts为当前扫描得到点集的全局坐标   
%   pixelSize表示 栅格地图一个单元的边长 对应 实际距离pixelSize米
%--------------------------------------------------------------------------
% 网格尺寸
% Grid size
minXY = min(pts) - 3 * pixelSize;%min(pts)返回x的最小值和y的最小值构成的向量(这并不一定是对应左下角,因为可能图里面没有左下角)
maxXY = max(pts) + 3 * pixelSize;% +3*pixelSize意思是 构成的栅格地图中 占用栅格最边界离地图边界留有3个栅格单元的余量
Sgrid = round((maxXY - minXY) / pixelSize) + 1;%Sgrid(1)为x轴向栅格数量,Sgrid(2)为y轴向栅格数量 +1是余量 round为四舍五入函数

N = size(pts, 1);%点集 里面 点的个数
%hits为被占用的栅格的二维坐标 (第hits(1)块,第hits(2)块)
hits = round( (pts-repmat(minXY, N, 1)) / pixelSize ) + 1;%点集里每个点的坐标 都减去它们的左下角坐标 再除单个栅格尺寸 再取整 再+1
idx = (hits(:,1)-1)*Sgrid(2) + hits(:,2);%把被占用的栅格的二维坐标转化为一维坐标
grid  = false(Sgrid(2), Sgrid(1));
%上述两行代码实现了图像的上下翻转

%将被占用的栅格幅值为正逻辑
grid(idx) = true;

gridmap.occGrid = grid;%栅格地图
gridmap.metricMap = min(bwdist(grid),10);%bwdist(grid)表示grid中0元素所在的位置靠近非零元素位置的最短距离构成的矩阵
%距离矩阵，可以用于匹配
gridmap.pixelSize = pixelSize;%栅格单元边长对应的实际长度
gridmap.topLeftCorner = minXY;%栅格地图的x最小值和y最小值构成的向量的全局坐标
% top left corner非常重要，说明图像进行了上下翻转
# coding=utf-8
import sys
from time import *
from math import *
import numpy as np
from OpenGL.GL import *
from OpenGL.GLU  import *
from OpenGL.GLUT import *
from FFD_Algorithm import *
from PyQt5.QtWidgets import *
import PyQt5.QtWidgets as QtWidgets
from vtk.qt.QVTKRenderWindowInteractor import *


# 一些控制信息
class Control(object):
    def __init__(self):
        self.show_scene = 0
        self.reset = 0
        self.move_arrow = 0
        self.addposition_x = 0
        self.addposition_y = 0
        self.addposition_z = 0
        self.arrow_position= 0
        self.lleg_move_horizontal = 0
        self.rleg_move_horizontal = 0
        self.lleg_rotate_horizontal = 0
        self.rleg_rotate_horizontal = 0
        self.lleg_move_vertical = 0
        self.rleg_move_vertical = 0
        self.lleg_rotate_vertical = 0
        self.rleg_rotate_vertical = 0

control = Control()


class FFD_Model(object):
    def __init__(self, ren=None, iren=None, filename=r"resources\cube.obj", RESIZE=1, COLOR=True, RADIUS=0.01, xl=4, yl=4, zl=4):
        # 参数初始化
        self.ren = ren
        self.iren = iren
        self.filename = filename
        self.RESIZE = RESIZE
        self.radius = RADIUS
        self.COLOR = COLOR
        self.xl = xl
        self.yl = yl
        self.zl = zl

        # 设置背景颜色
        self.ren.SetBackground(237, 237, 237)

        # 用TrackballCamera的交互方式
        InteractorStyle = vtk.vtkInteractorStyleTrackballCamera()
        self.iren.SetInteractorStyle(InteractorStyle)

        # 初始化画图
        self.Load_OBJ()
        self.Draw_Objects()

        # 根据load进来的物体大小 自适应设置球体的半径大小
        self.radius = (self.ffd.max_x - self.ffd.min_x) * self.radius

        # 画点
        self.Draw_Points()
        # 画线
        self.Draw_Line()
        # 增加监听器
        self.Add_Observer()

    # 生成坐标系坐标
    def Get_Position(self, i, j, k):
        """
        i, j, k为控制点在x轴 y轴 z轴方向分别的索引值
        x, y, z为控制点在坐标系中的坐标
        该坐标由ffd算法根据读入进来的物体的大小自动生成 保证控制点为能恰好包裹住物体的长方体
        """
        x, y, z = self.ffd.control_points_location[i][j][k]
        return x, y, z
    # 当前球的六个邻居
    def Get_Neighbor(self, i, j, k):
        """
        找到第i,j,k号球对应的所有邻居球体的索引值 即:上下左右前后六个点 通过索引值返回即可 并判断越界
        """
        n = []
        if i > 0:
            n.append((i - 1, j, k))
        if i < self.xl:
            n.append((i + 1, j, k))
        if j > 0:
            n.append((i, j - 1, k))
        if j < self.yl:
            n.append((i, j + 1, k))
        if k > 0:
            n.append((i, j, k - 1))
        if k < self.zl:
            n.append((i, j, k + 1))
        return n
    # 加载OBJ文件
    def Load_OBJ(self):
        """
        初始化，加载模型.obj格式文件
        """
        self.reader = vtk.vtkOBJReader()
        self.reader.SetFileName(self.filename)
        self.reader.Update()
        self.data = self.reader.GetOutput()
    # 着色
    def Draw_Color(self):
        if self.COLORED:
            return
        else:
            self.data.GetPointData().SetScalars(self.data_color.GetPointData().GetScalars())
            self.COLORED = True
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(self.data)

        try:
            self.ren.RemoveActor(self.actor)
            self.actor = vtk.vtkActor()
            self.actor.SetMapper(mapper)
            self.ren.AddActor(self.actor)
        except:
            pass
    # 画出图像
    def Draw_Objects(self, COLOR=False, RESIZE=1.0):
        """
        选择是否需要着色以及是否需要压缩图像
        """

        # 拷贝一份着色PolyData
        self.data_color = vtk.vtkPolyData()
        self.data_color.DeepCopy(self.data)
        self.data_color = Add_Color(self.data_color, Read_Color(self.filename))
        self.COLORED = False

        # 如果需要着色的话
        if COLOR:
            self.Draw_Color()

        self.points = self.data.GetPoints()
        vertices = [self.points.GetPoint(i) for i in range(self.points.GetNumberOfPoints())]
        self.ffd = FFD(num_x=self.xl + 1, num_y=self.yl + 1, num_z=self.zl + 1, object_points=vertices,
                       object_file=self.filename)
        self.ffd.initial_control_points()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(self.data)

        self.actor = vtk.vtkActor()
        self.actor.SetMapper(mapper)
        self.ren.AddActor(self.actor)
    # 画出控制点球体
    def Draw_Points(self):
        """
        生成控制点球体
        """
        # 初始化列表 保存每一个控制球体 列表为i*j*k维 因为有i*j*k个球体
        self.spherelist = [[[0 for zcol in range(self.zl + 1)] for col in range(self.yl + 1)] for row in
                           range(self.xl + 1)]

        for i, j, k in ((a, b, c) for a in range(self.xl + 1) for b in range(self.yl + 1) for c in range(self.zl + 1)):
            # 定义一个球状体widget
            sphereWidget = vtk.vtkSphereWidget()
            # 渲染窗口交互器实例iren是一个3D的球状体widget
            sphereWidget.SetInteractor(self.iren)
            # 从索引值i，j，k映射到真实空间中的xyz坐标
            x, y, z = self.Get_Position(i, j, k)
            # 设置球状体在真实空间中的xyz坐标
            sphereWidget.SetCenter(x, y, z)
            # 设置球状体的半径大小
            sphereWidget.SetRadius(self.radius)
            # 设置球面的颜色 仍然是通过GetProperty来获取属性并进行设置
            # sphereWidget.GetSphereProperty().SetColor(0, 1.0, 0)
            # 设置填充球状体的表面 三种基本的属性设置方式：点方式，网格方式和面方式
            sphereWidget.SetRepresentationToSurface()
            # 显示球状体
            sphereWidget.On()
            # 将球状体添加到球状体的列表中
            self.spherelist[i][j][k] = sphereWidget
    # 画出球体所在的线段
    def Draw_Line(self):
        """
        初始化画线 生成用于保存线的sourcelist, mapperlist, actorlist
        获取每个控制点球体的位置并保存在spherelocation中
        将每个控制点与其邻居结点连接起来
        """
        # 初始化列表 这些列表用于保存边与边的关系 列表为i*j*k*6维 因为有i*j*k个球体 一个球最多有6个邻居
        self.sourcelist = [[[[vtk.vtkLineSource() for nei in range(6)] for zcol in range(self.zl + 1)] for col in range(self.yl + 1)]
                              for row in range(self.xl + 1)]
        self.mapperlist = [[[[vtk.vtkPolyDataMapper() for nei in range(6)] for zcol in range(self.zl+1)] for col in range(self.yl+1)]
                           for row in range(self.xl+1)]
        self.actorlist = [[[[vtk.vtkActor() for nei in range(6)] for zcol in range(self.zl + 1)] for col in range(self.yl + 1)]
                              for row in range(self.xl + 1)]


        # 初始化列表 实时保存和更新球的坐标 列表为i*j*k维 因为有i*j*k个球体
        self.spherelocation = [[[0 for zcol in range(self.zl+1)] for col in range(self.yl+1)] for row in range(self.xl+1)]

        for i, j, k in ((a, b, c) for a in range(self.xl + 1) for b in range(self.yl + 1) for c in range(self.zl + 1)):
            # 对于一个球体i 获取球心的位置
            x1, y1, z1 = self.spherelist[i][j][k].GetCenter()
            # 在初始化时 记录球体的位置
            self.spherelocation[i][j][k] = [x1, y1, z1]
            neighbors = self.Get_Neighbor(i, j, k)
            count = 0
            for (i_neighbor, j_neighbor, k_neighbor) in neighbors:
                # 对于这个球体i的邻居j 获取球心的位置
                x2, y2, z2 = self.spherelist[i_neighbor][j_neighbor][k_neighbor].GetCenter()

                # 设置一条线的起点和终点
                self.sourcelist[i][j][k][count].SetPoint1(x1, y1, z1)
                self.sourcelist[i][j][k][count].SetPoint2(x2, y2, z2)

                # Filter的连接可以通过方法SetInputConnection()和GetOutputPort()，输出通过方法SetInputConnection()设置为vtkPolyDataMapper对象的输入
                self.mapperlist[i][j][k][count].SetInputConnection(self.sourcelist[i][j][k][count].GetOutputPort())

                # 设置定义几何信息的mapper到这个actor里
                self.actorlist[i][j][k][count].SetMapper(self.mapperlist[i][j][k][count])
                self.actorlist[i][j][k][count].GetMapper().ScalarVisibilityOff()
                self.actorlist[i][j][k][count].GetProperty().SetColor(0, 1.0, 0)

                # 把要渲染的actor加入到renderer中去。
                self.ren.AddActor(self.actorlist[i][j][k][count])
                count += 1
    # 添加监听器
    def Add_Observer(self):
        """
        对于每一个球体控制点 添加Observer监听vtkRenderWindowInteractor里的事件
        用户方法通过定义一个回调函数sphereCallback并将其作为参数传入AddObserver来定义
        该函数将GUI交互器与用户自定义的渲染交互窗口交互器的方法关联起来
        """
        for i, j, k in ((a, b, c) for a in range(self.xl + 1) for b in range(self.yl + 1) for c in range(self.zl + 1)):
            self.spherelist[i][j][k].AddObserver("InteractionEvent", self.sphereCallback)
    # 控制点的回调交互函数
    def sphereCallback(self, obj, event):
        """
        主要功能为:
        	检查控制点是否被拽动
        	对于被拽动的控制点: 去掉旧的邻居结点连线并增加新的连线
        					 去掉旧的人脸并调用ffd算法生成新的人脸
        """
        self._sphereCallback()
    # 回调Qt
    def sphereQt(self, xyz_index, xyz):
        i, j, k = xyz_index
        self.spherelist[i][j][k].SetCenter(xyz)
        self._sphereCallback()
    # 真正的回调函数
    def _sphereCallback(self):
        for i, j, k in ((a, b, c) for a in range(self.xl + 1) for b in range(self.yl + 1) for c in range(self.zl + 1)):
            # 对于一个球体i 获取它之前的位置以及现在球心的位置
            x_old, y_old, z_old = self.spherelocation[i][j][k]
            x_new, y_new, z_new = self.spherelist[i][j][k].GetCenter()

            # 只对发生改变的球体进行计算 如果球体的位置发生改变 即该控制点被拖动
            if x_new != x_old or y_new != y_old or z_new != z_old:
                # 将更新后的坐标点传给ffd算法保存下来并更新spherelocation里面保存的每一个球体的位置
                self.ffd.changed_update((i, j, k), np.array([x_new, y_new, z_new]))
                self.spherelocation[i][j][k] = [x_new, y_new, z_new]

                # 对于球体位置改变的控制点 计算得到它的邻居点，并且重新连线
                neighbors = self.Get_Neighbor(i, j, k)
                count = 0
                for (i_neighbor, j_neighbor, k_neighbor) in neighbors:
                    # 获取邻居的球心的位置
                    x2, y2, z2 = self.spherelist[i_neighbor][j_neighbor][k_neighbor].GetCenter()

                    # 设置控制点移动后的新位置与邻居结点连线的起点和终点
                    self.sourcelist[i][j][k][count].SetPoint1(x_new, y_new, z_new)
                    self.sourcelist[i][j][k][count].SetPoint2(x2, y2, z2)

                    # Filter的连接可以通过方法SetInputConnection()和GetOutputPort()，输出通过方法SetInputConnection()设置为vtkPolyDataMapper对象的输入
                    self.mapperlist[i][j][k][count].SetInputConnection(self.sourcelist[i][j][k][count].GetOutputPort())

                    # 去掉当前控制球和他的邻居球生成的旧线
                    nei_of_nei = self.Get_Neighbor(i_neighbor, j_neighbor, k_neighbor).index((i, j, k))
                    self.ren.RemoveActor(self.actorlist[i_neighbor][j_neighbor][k_neighbor][nei_of_nei])

                    # 设置定义几何信息的mapper到这个actor里
                    self.actorlist[i][j][k][count].SetMapper(self.mapperlist[i][j][k][count])
                    self.actorlist[i][j][k][count].GetMapper().ScalarVisibilityOff()

                    # 设置Actor的颜色
                    self.actorlist[i][j][k][count].GetProperty().SetColor(0, 1.0, 0)

                    # 把要渲染的actor加入到renderer中去。
                    self.ren.AddActor(self.actorlist[i][j][k][count])
                    count += 1

        # 更新控制点
        self.ffd.update_position()
        self.points = self.data.GetPoints()

        # 只对需要改变的点进行计算 并将计算后更改后的数据存入data的points数据中
        for u, v, w in self.ffd.changed.keys():
            for a, b, c in ((a, b, c) for a in range(-2, 2) for b in range(-2, 2) for c in range(-2, 2)):
                if 0 <= u + a < self.ffd.control_points_num_x and 0 <= v + b < self.ffd.control_points_num_y and 0 <= w + c < self.ffd.control_points_num_z:
                    for (id_index, x, y, z) in self.ffd.object_points[(u + a, v + b, w + c)]:
                        tmp = self.ffd.T_Local([x, y, z])
                        self.points.SetPoint(id_index, tuple([x + tmp[0], y + tmp[1], z + tmp[2]]))

        # 构造mapper
        self.ffd.changed_reset()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(self.data)

        # 去掉原始的人脸
        self.ren.RemoveActor(self.actor)
        # 添加更改后的新的人脸
        self.actor = vtk.vtkActor()
        self.actor.SetMapper(mapper)
        self.ren.AddActor(self.actor)



# 初始化Qt窗口
class Init_Qt_window(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("Final Assignment")
        MainWindow.resize(1024, 800)
        self.centralWidget = QtWidgets.QWidget(MainWindow)
        self.gridlayout    = QtWidgets.QGridLayout(self.centralWidget)
        self.vtkWidget     = QVTKRenderWindowInteractor(self.centralWidget)
        MainWindow.setCentralWidget(self.vtkWidget)

# 设置窗口菜单
class Qt_Window_Menu(QtWidgets.QMainWindow):
    # 初始化
    def __init__(self, parent=None):
        QtWidgets.QMainWindow.__init__(self, parent)
        self.ui = Init_Qt_window()
        self.ui.setupUi(self)
        self.createActions()
        self.createMenus()
        self.filename = r"resources\cylinder.obj"

    # 初始化FFD模型的参数
    def initFFD(self, dots=4):
        self.ren = vtk.vtkRenderer()
        self.ui.vtkWidget.GetRenderWindow().AddRenderer(self.ren)
        self.iren = self.ui.vtkWidget.GetRenderWindow().GetInteractor()
        self.dots = dots
        self.dot_xyz = [None, None, None]
        self.model = FFD_Model(ren=self.ren, iren=self.iren,filename=self.filename, xl=dots-1, yl=dots-1, zl=dots - 1)

    # 初始化展示几何体的参数
    def initObjects(self,texture_choice = 0):
        self.ren2 = vtk.vtkRenderer()
        self.ui.vtkWidget.GetRenderWindow().AddRenderer(self.ren2)
        self.iren2 = self.ui.vtkWidget.GetRenderWindow().GetInteractor()
        self.iren2.AddObserver("KeyPressEvent", self.keyboard_rollback)
        self.draw_objects(ren2=self.ren2, iren2=self.iren2 , texture_choice = texture_choice)

    # 展示FFD
    def showFFD(self):
        self.iren.Initialize()
        self.show()

    # 展示三维场景
    def showObjects(self):
        self.iren2.Initialize()
        self.show()

    # 画场景里需要的物体
    def draw_objects(self, ren2=None, iren2=None, texture_choice=0):
        """
        一共有15个小物体，组成射箭场景
        其中第5个物体是箭，对其进行特殊操作
        通过control.move_arrow判断是不是要进行射箭动作
        只需要对objects5进行removeactor和重新绘制即可，不需要对所有的objects重绘，
        否则不仅增加系统开销，而且动画会有卡顿。
        """

        # 参数初始化
        self.ren2 = ren2
        self.iren2 = iren2
        self.texture_choice = texture_choice

        # 设置背景颜色（与FFD窗口的保持一致）
        self.ren2.SetBackground(0, 100, 100)

        # 用TrackballCamera的交互方式
        InteractorStyle = vtk.vtkInteractorStyleTrackballCamera()
        self.iren2.SetInteractorStyle(InteractorStyle)

        # 创建13个几何体
        objects = vtk.vtkCylinderSource()   # 左腿
        objects2 = vtk.vtkCylinderSource()  # 右腿
        objects3 = vtk.vtkCubeSource()      # 身体
        objects4 = vtk.vtkCubeSource()      # 头
        objects5 = vtk.vtkArrowSource()     # 箭
        objects6 = vtk.vtkSphereSource()    # 圆盘

        objects7 = vtk.vtkCubeSource()   # 弓
        objects8 = vtk.vtkCubeSource()   # 弓
        objects9 = vtk.vtkCubeSource()   # 弓

        objects10 = vtk.vtkCubeSource()  # 右手臂
        objects11 = vtk.vtkCubeSource()  # 右手臂
        objects12 = vtk.vtkCubeSource()  # 左手臂
        objects13 = vtk.vtkCubeSource()  # 左手臂

        objects14 = vtk.vtkCubeSource()  # 墙
        objects15 = vtk.vtkCubeSource()  # 草地

        if control.move_arrow == 1:
            image_name5 = r"resources\4.png"
            objects5.Update()
            # 读取图片信息
            reader5 = vtk.vtkPNGReader()
            reader5.SetFileName(image_name5)

            # 创建纹理对象
            texture5 = vtk.vtkTexture()
            texture5.SetInputConnection(reader5.GetOutputPort())

            # 将纹理映射到几何体上去
            map_texture_to_objects5 = vtk.vtkTextureMapToSphere()
            map_texture_to_objects5.SetInputConnection(objects5.GetOutputPort())
            map_texture_to_objects5.PreventSeamOn()

            # 几何体映射
            objects_mapper5 = vtk.vtkPolyDataMapper()
            if texture_choice == 0:
                # 没有添加纹理就绑定几何体的信息
                objects_mapper5.SetInputData(objects5.GetOutput())
            else:
                # 需要添加纹理就绑定几何体+纹理的综合信息
                objects_mapper5.SetInputConnection(map_texture_to_objects5.GetOutputPort())

            # 创建执行单元actor
            self.objects_actor5 = vtk.vtkActor()
            self.objects_actor5.SetMapper(objects_mapper5)
            if texture_choice != 0:
                self.objects_actor5.SetTexture(texture5)

            # 设置几何体的位置、形状、大小、角度
            self.objects_actor5.SetScale(6.0, 3.0, 3.0)
            self.objects_actor5.SetPosition(12.00 + control.addposition_x + control.arrow_position,
                                            3.0 + control.addposition_y,
                                            0.9 + control.addposition_z)
            if texture_choice == 0:
                self.objects_actor5.GetProperty().SetColor(0, 0, 0)
            # 渲染
            self.ren2.AddActor(self.objects_actor5)
            return

        objects.Update()  # 不加看不到模型
        image_name = r"resources\8.png"
        # 读取图片信息
        reader = vtk.vtkPNGReader()
        reader.SetFileName(image_name)

        # 创建纹理对象
        texture = vtk.vtkTexture()
        texture.SetInputConnection(reader.GetOutputPort())

        # 将纹理映射到几何体上去
        map_texture_to_objects = vtk.vtkTextureMapToSphere()
        map_texture_to_objects.SetInputConnection(objects.GetOutputPort())
        map_texture_to_objects.PreventSeamOn()

        # 几何体映射
        objects_mapper = vtk.vtkPolyDataMapper()
        if texture_choice == 0:
            # 没有添加纹理就绑定几何体的信息
            objects_mapper.SetInputData(objects.GetOutput())
        else:
            # 需要添加纹理就绑定几何体+纹理的综合信息
            objects_mapper.SetInputConnection(map_texture_to_objects.GetOutputPort())

        # 创建执行单元actor
        self.objects_actor = vtk.vtkActor()
        self.objects_actor.SetMapper(objects_mapper)
        if texture_choice != 0:
            self.objects_actor.SetTexture(texture)

        # 设置几何体的位置、形状、大小、角度
        self.objects_actor.SetScale(0.8, 4, 0.8)
        self.objects_actor.RotateX(control.lleg_rotate_horizontal)
        self.objects_actor.RotateZ(control.lleg_rotate_vertical)
        self.objects_actor.SetPosition(8 + control.addposition_x+control.lleg_move_vertical, 0 + control.addposition_y, 0 + control.addposition_z+control.lleg_move_horizontal)

        if texture_choice == 0:
            self.objects_actor.GetProperty().SetColor(100, 0, 0)
        # 渲染
        self.ren2.AddActor(self.objects_actor)

        objects2.Update()
        # 读取图片信息
        reader2 = vtk.vtkPNGReader()
        reader2.SetFileName(image_name)

        # 创建纹理对象
        texture2 = vtk.vtkTexture()
        texture2.SetInputConnection(reader2.GetOutputPort())

        # 将纹理映射到几何体上去
        map_texture_to_objects2 = vtk.vtkTextureMapToSphere()
        map_texture_to_objects2.SetInputConnection(objects2.GetOutputPort())
        map_texture_to_objects2.PreventSeamOn()

        # 几何体映射
        objects_mapper2 = vtk.vtkPolyDataMapper()
        if texture_choice == 0:
            # 没有添加纹理就绑定几何体的信息
            objects_mapper2.SetInputData(objects2.GetOutput())
        else:
            # 需要添加纹理就绑定几何体+纹理的综合信息
            objects_mapper2.SetInputConnection(map_texture_to_objects2.GetOutputPort())

        # 创建执行单元actor
        self.objects_actor2 = vtk.vtkActor()
        self.objects_actor2.SetMapper(objects_mapper2)
        if texture_choice != 0:
            self.objects_actor2.SetTexture(texture2)

        # 设置几何体的位置、形状、大小、角度
        self.objects_actor2.SetScale(0.8, 4, 0.8)
        self.objects_actor2.RotateX(control.rleg_rotate_horizontal)
        self.objects_actor2.RotateZ(control.rleg_rotate_vertical)
        self.objects_actor2.SetPosition(8 + control.addposition_x+control.rleg_move_vertical, 0 + control.addposition_y,
                                        1.5 + control.addposition_z+control.rleg_move_horizontal)
        if texture_choice == 0:
            self.objects_actor2.GetProperty().SetColor(100, 0, 0)

        # 渲染
        self.ren2.AddActor(self.objects_actor2)

        objects3.Update()
        # 读取图片信息
        reader3 = vtk.vtkPNGReader()
        reader3.SetFileName(image_name)

        # 创建纹理对象
        texture3 = vtk.vtkTexture()
        texture3.SetInputConnection(reader3.GetOutputPort())

        # 将纹理映射到几何体上去
        map_texture_to_objects3 = vtk.vtkTextureMapToSphere()
        map_texture_to_objects3.SetInputConnection(objects3.GetOutputPort())
        map_texture_to_objects3.PreventSeamOn()

        # 几何体映射
        objects_mapper3 = vtk.vtkPolyDataMapper()
        if texture_choice == 0:
            # 没有添加纹理就绑定几何体的信息
            objects_mapper3.SetInputData(objects3.GetOutput())
        else:
            # 需要添加纹理就绑定几何体+纹理的综合信息
            objects_mapper3.SetInputConnection(map_texture_to_objects3.GetOutputPort())

        # 创建执行单元actor
        self.objects_actor3 = vtk.vtkActor()
        self.objects_actor3.SetMapper(objects_mapper3)
        if texture_choice != 0:
            self.objects_actor3.SetTexture(texture3)

        # 设置几何体的位置、形状、大小、角度
        self.objects_actor3.SetScale(2.6, 2.6, 2.9)
        self.objects_actor3.SetPosition(8 + control.addposition_x, 2.4 + control.addposition_y,
                                        0.8 + control.addposition_z)
        if texture_choice == 0:
            self.objects_actor3.GetProperty().SetColor(100, 0, 0)
        # 4. 渲染
        self.ren2.AddActor(self.objects_actor3)

        objects4.Update()
        # 读取图片信息
        reader4 = vtk.vtkPNGReader()
        reader4.SetFileName(image_name)

        # 创建纹理对象
        texture4 = vtk.vtkTexture()
        texture4.SetInputConnection(reader4.GetOutputPort())

        # 将纹理映射到几何体上去
        map_texture_to_objects4 = vtk.vtkTextureMapToSphere()
        map_texture_to_objects4.SetInputConnection(objects4.GetOutputPort())
        map_texture_to_objects4.PreventSeamOn()

        # 几何体映射
        objects_mapper4 = vtk.vtkPolyDataMapper()
        if texture_choice == 0:
            # 没有添加纹理就绑定几何体的信息
            objects_mapper4.SetInputData(objects4.GetOutput())
        else:
            # 需要添加纹理就绑定几何体+纹理的综合信息
            objects_mapper4.SetInputConnection(map_texture_to_objects4.GetOutputPort())

        # 创建执行单元actor
        self.objects_actor4 = vtk.vtkActor()
        self.objects_actor4.SetMapper(objects_mapper4)
        if texture_choice != 0:
            self.objects_actor4.SetTexture(texture4)

        # 设置几何体的位置、形状、大小、角度
        self.objects_actor4.SetScale(1.4, 1.6, 1.4)
        self.objects_actor4.SetPosition(8 + control.addposition_x, 4.5 + control.addposition_y,
                                        0.9 + control.addposition_z)
        if texture_choice == 0:
            self.objects_actor4.GetProperty().SetColor(100, 0, 0)
        # 渲染
        self.ren2.AddActor(self.objects_actor4)

        image_name5 = r"resources\4.png"
        objects5.Update()
        # 读取图片信息
        reader5 = vtk.vtkPNGReader()
        reader5.SetFileName(image_name5)

        # 创建纹理对象
        texture5 = vtk.vtkTexture()
        texture5.SetInputConnection(reader5.GetOutputPort())

        # 将纹理映射到几何体上去
        map_texture_to_objects5 = vtk.vtkTextureMapToSphere()
        map_texture_to_objects5.SetInputConnection(objects5.GetOutputPort())
        map_texture_to_objects5.PreventSeamOn()

        # 几何体映射
        objects_mapper5 = vtk.vtkPolyDataMapper()
        if texture_choice == 0:
            # 没有添加纹理就绑定几何体的信息
            objects_mapper5.SetInputData(objects5.GetOutput())
        else:
            # 需要添加纹理就绑定几何体+纹理的综合信息
            objects_mapper5.SetInputConnection(map_texture_to_objects5.GetOutputPort())

        # 创建执行单元actor
        self.objects_actor5 = vtk.vtkActor()
        self.objects_actor5.SetMapper(objects_mapper5)
        if texture_choice != 0:
            self.objects_actor5.SetTexture(texture5)

        # 设置几何体的位置、形状、大小、角度
        self.objects_actor5.SetScale(6.0, 3.0, 3.0)
        self.objects_actor5.SetPosition(12.00 + control.addposition_x + control.arrow_position,
                                        3.0 + control.addposition_y,
                                        0.9 + control.addposition_z)
        if texture_choice == 0:
            self.objects_actor5.GetProperty().SetColor(0, 0, 0)
        # 4. 渲染
        self.ren2.AddActor(self.objects_actor5)

        image_name6 = r"resources\3.png"
        objects6.Update()
        # 读取图片信息
        reader6 = vtk.vtkPNGReader()
        reader6.SetFileName(image_name6)

        # 创建纹理对象
        texture6 = vtk.vtkTexture()
        texture6.SetInputConnection(reader6.GetOutputPort())

        # 将纹理映射到几何体上去
        map_texture_to_objects6 = vtk.vtkTextureMapToSphere()
        map_texture_to_objects6.SetInputConnection(objects6.GetOutputPort())
        map_texture_to_objects6.PreventSeamOn()

        # 几何体映射
        objects_mapper6 = vtk.vtkPolyDataMapper()
        if texture_choice == 0:
            # 没有添加纹理就绑定几何体的信息
            objects_mapper6.SetInputData(objects6.GetOutput())
        else:
            # 需要添加纹理就绑定几何体+纹理的综合信息
            objects_mapper6.SetInputConnection(map_texture_to_objects6.GetOutputPort())

        # 创建执行单元actor
        self.objects_actor6 = vtk.vtkActor()
        self.objects_actor6.SetMapper(objects_mapper6)
        if texture_choice != 0:
            self.objects_actor6.SetTexture(texture6)

        # 设置几何体的位置、形状、大小、角度

        self.objects_actor6.RotateZ(90)
        self.objects_actor6.SetScale(6.0, 0.3, 6.0)
        self.objects_actor6.SetPosition(39, 3.0, 0.9)
        if texture_choice == 0:
            self.objects_actor6.GetProperty().SetColor(0, 100, 0)
        # 渲染
        self.ren2.AddActor(self.objects_actor6)

        image_name_arrow = r"resources\4.png"
        objects7.Update()
        # 读取图片信息
        reader7 = vtk.vtkPNGReader()
        reader7.SetFileName(image_name_arrow)

        # 创建纹理对象
        texture7 = vtk.vtkTexture()
        texture7.SetInputConnection(reader7.GetOutputPort())

        # 将纹理映射到几何体上去
        map_texture_to_objects7 = vtk.vtkTextureMapToSphere()
        map_texture_to_objects7.SetInputConnection(objects7.GetOutputPort())
        map_texture_to_objects7.PreventSeamOn()

        # 几何体映射
        objects_mapper7 = vtk.vtkPolyDataMapper()
        if texture_choice == 0:
            # 没有添加纹理就绑定几何体的信息
            objects_mapper7.SetInputData(objects7.GetOutput())
        else:
            # 需要添加纹理就绑定几何体+纹理的综合信息
            objects_mapper7.SetInputConnection(map_texture_to_objects7.GetOutputPort())

        # 创建执行单元actor
        self.objects_actor7 = vtk.vtkActor()
        self.objects_actor7.SetMapper(objects_mapper7)
        if texture_choice != 0:
            self.objects_actor7.SetTexture(texture7)

        # 设置几何体的位置、形状、大小、角度
        self.objects_actor7.SetScale(0.3, 5.0, 0.3)
        self.objects_actor7.SetPosition(11.8 + control.addposition_x, 3.0 + control.addposition_y,
                                        0.9 + control.addposition_z)
        if texture_choice == 0:
            self.objects_actor7.GetProperty().SetColor(0, 1, 10)
        # 4. 渲染
        self.ren2.AddActor(self.objects_actor7)

        objects8.Update()
        # 读取图片信息
        reader8 = vtk.vtkPNGReader()
        reader8.SetFileName(image_name_arrow)

        # 创建纹理对象
        texture8 = vtk.vtkTexture()
        texture8.SetInputConnection(reader8.GetOutputPort())

        # 将纹理映射到几何体上去
        map_texture_to_objects8 = vtk.vtkTextureMapToSphere()
        map_texture_to_objects8.SetInputConnection(objects8.GetOutputPort())
        map_texture_to_objects8.PreventSeamOn()

        # 几何体映射
        objects_mapper8 = vtk.vtkPolyDataMapper()
        if texture_choice == 0:
            # 没有添加纹理就绑定几何体的信息
            objects_mapper8.SetInputData(objects8.GetOutput())
        else:
            # 需要添加纹理就绑定几何体+纹理的综合信息
            objects_mapper8.SetInputConnection(map_texture_to_objects8.GetOutputPort())

        # 创建执行单元actor
        self.objects_actor8 = vtk.vtkActor()
        self.objects_actor8.SetMapper(objects_mapper8)
        if texture_choice != 0:
            self.objects_actor8.SetTexture(texture8)

        # 设置几何体的位置、形状、大小、角度
        self.objects_actor8.SetScale(0.3, 3.0, 0.3)
        self.objects_actor8.RotateZ(15)
        self.objects_actor8.SetPosition(12.5 + control.addposition_x, 4.0 + control.addposition_y,
                                        0.9 + control.addposition_z)
        if texture_choice == 0:
            self.objects_actor8.GetProperty().SetColor(0, 1, 10)
        # 渲染
        self.ren2.AddActor(self.objects_actor8)

        objects9.Update()
        # 读取图片信息
        reader9 = vtk.vtkPNGReader()
        reader9.SetFileName(image_name_arrow)

        # 创建纹理对象
        texture9 = vtk.vtkTexture()
        texture9.SetInputConnection(reader9.GetOutputPort())

        # 将纹理映射到几何体上去
        map_texture_to_objects9 = vtk.vtkTextureMapToSphere()
        map_texture_to_objects9.SetInputConnection(objects9.GetOutputPort())
        map_texture_to_objects9.PreventSeamOn()

        # 几何体映射
        objects_mapper9 = vtk.vtkPolyDataMapper()
        if texture_choice == 0:
            # 没有添加纹理就绑定几何体的信息
            objects_mapper9.SetInputData(objects9.GetOutput())
        else:
            # 需要添加纹理就绑定几何体+纹理的综合信息
            objects_mapper9.SetInputConnection(map_texture_to_objects9.GetOutputPort())

        # 创建执行单元actor
        self.objects_actor9 = vtk.vtkActor()
        self.objects_actor9.SetMapper(objects_mapper9)
        if texture_choice != 0:
            self.objects_actor9.SetTexture(texture9)

        self.objects_actor9.SetScale(0.3, 2.5, 0.3)
        self.objects_actor9.RotateZ(-15)
        self.objects_actor9.SetPosition(12.5 + control.addposition_x, 1.7 + control.addposition_y,
                                        0.9 + control.addposition_z)
        if texture_choice == 0:
            self.objects_actor9.GetProperty().SetColor(0, 1, 10)
        # 渲染
        self.ren2.AddActor(self.objects_actor9)

        objects10.Update()
        # 读取图片信息
        reader10 = vtk.vtkPNGReader()
        reader10.SetFileName(image_name)

        # 创建纹理对象
        texture10 = vtk.vtkTexture()
        texture10.SetInputConnection(reader10.GetOutputPort())

        # 将纹理映射到几何体上去
        map_texture_to_objects10 = vtk.vtkTextureMapToSphere()
        map_texture_to_objects10.SetInputConnection(objects10.GetOutputPort())
        map_texture_to_objects10.PreventSeamOn()

        # 几何体映射
        objects_mapper10 = vtk.vtkPolyDataMapper()
        if texture_choice == 0:
            # 没有添加纹理就绑定几何体的信息
            objects_mapper10.SetInputData(objects10.GetOutput())
        else:
            # 需要添加纹理就绑定几何体+纹理的综合信息
            objects_mapper10.SetInputConnection(map_texture_to_objects10.GetOutputPort())

        # 创建执行单元actor
        self.objects_actor10 = vtk.vtkActor()
        self.objects_actor10.SetMapper(objects_mapper10)
        if texture_choice != 0:
            self.objects_actor10.SetTexture(texture10)

        # 设置几何体的位置、形状、大小、角度
        self.objects_actor10.SetScale(0.3, 4.0, 0.3)
        self.objects_actor10.RotateX(90)
        self.objects_actor10.RotateZ(45)
        self.objects_actor10.SetPosition(6.5 + control.addposition_x, 3.0 + control.addposition_y,
                                         2.8 + control.addposition_z)
        if texture_choice == 0:
            self.objects_actor10.GetProperty().SetColor(100, 0, 0)
        # 渲染
        self.ren2.AddActor(self.objects_actor10)

        objects11.Update()
        # 读取图片信息
        reader11 = vtk.vtkPNGReader()
        reader11.SetFileName(image_name)

        # 创建纹理对象
        texture11 = vtk.vtkTexture()
        texture11.SetInputConnection(reader11.GetOutputPort())

        # 将纹理映射到几何体上去
        map_texture_to_objects11 = vtk.vtkTextureMapToSphere()
        map_texture_to_objects11.SetInputConnection(objects11.GetOutputPort())
        map_texture_to_objects11.PreventSeamOn()

        # 几何体映射
        objects_mapper11 = vtk.vtkPolyDataMapper()
        if texture_choice == 0:
            # 没有添加纹理就绑定几何体的信息
            objects_mapper11.SetInputData(objects11.GetOutput())
        else:
            # 需要添加纹理就绑定几何体+纹理的综合信息
            objects_mapper11.SetInputConnection(map_texture_to_objects11.GetOutputPort())

        # 创建执行单元actor
        self.objects_actor11 = vtk.vtkActor()
        self.objects_actor11.SetMapper(objects_mapper11)
        if texture_choice != 0:
            self.objects_actor11.SetTexture(texture11)

        # 设置几何体的位置、形状、大小、角度
        self.objects_actor11.SetScale(0.3, 2.5, 0.3)
        self.objects_actor11.RotateX(90)
        self.objects_actor11.RotateZ(45)
        self.objects_actor11.SetPosition(8.5 + control.addposition_x, 3.0 + control.addposition_y,
                                         -1 + control.addposition_z)
        if texture_choice == 0:
            self.objects_actor11.GetProperty().SetColor(100, 0, 0)
        # 渲染
        self.ren2.AddActor(self.objects_actor11)

        objects12.Update()
        # 读取图片信息
        reader12 = vtk.vtkPNGReader()
        reader12.SetFileName(image_name)

        # 创建纹理对象
        texture12 = vtk.vtkTexture()
        texture12.SetInputConnection(reader12.GetOutputPort())

        # 将纹理映射到几何体上去
        map_texture_to_objects12 = vtk.vtkTextureMapToSphere()
        map_texture_to_objects12.SetInputConnection(objects12.GetOutputPort())
        map_texture_to_objects12.PreventSeamOn()

        # 几何体映射
        objects_mapper12 = vtk.vtkPolyDataMapper()
        if texture_choice == 0:
            # 没有添加纹理就绑定几何体的信息
            objects_mapper12.SetInputData(objects12.GetOutput())
        else:
            # 需要添加纹理就绑定几何体+纹理的综合信息
            objects_mapper12.SetInputConnection(map_texture_to_objects12.GetOutputPort())

        # 创建执行单元actor
        self.objects_actor12 = vtk.vtkActor()
        self.objects_actor12.SetMapper(objects_mapper12)
        if texture_choice != 0:
            self.objects_actor12.SetTexture(texture12)

        # 设置几何体的位置、形状、大小、角度
        self.objects_actor12.SetScale(0.3, 7.8, 0.3)
        self.objects_actor12.RotateX(90)
        self.objects_actor12.RotateZ(65)
        self.objects_actor12.SetPosition(8.5 + control.addposition_x, 3.0 + control.addposition_y,
                                         2.8 + control.addposition_z)
        if texture_choice == 0:
            self.objects_actor12.GetProperty().SetColor(100, 0, 0)
        # 渲染
        self.ren2.AddActor(self.objects_actor12)

        objects13.Update()
        # 读取图片信息
        reader13 = vtk.vtkPNGReader()
        reader13.SetFileName(image_name)

        # 创建纹理对象
        texture13 = vtk.vtkTexture()
        texture13.SetInputConnection(reader13.GetOutputPort())

        # 将纹理映射到几何体上去
        map_texture_to_objects13 = vtk.vtkTextureMapToSphere()
        map_texture_to_objects13.SetInputConnection(objects13.GetOutputPort())
        map_texture_to_objects13.PreventSeamOn()

        # 几何体映射
        objects_mapper13 = vtk.vtkPolyDataMapper()
        if texture_choice == 0:
            # 没有添加纹理就绑定几何体的信息
            objects_mapper13.SetInputData(objects13.GetOutput())
        else:
            # 需要添加纹理就绑定几何体+纹理的综合信息
            objects_mapper13.SetInputConnection(map_texture_to_objects13.GetOutputPort())

        # 创建执行单元actor
        self.objects_actor13 = vtk.vtkActor()
        self.objects_actor13.SetMapper(objects_mapper13)
        if texture_choice != 0:
            self.objects_actor13.SetTexture(texture13)

        # 设置几何体的位置、形状、大小、角度
        self.objects_actor13.SetScale(0.3, 3.5, 0.3)
        self.objects_actor13.RotateX(90)
        self.objects_actor13.RotateZ(135)
        self.objects_actor13.SetPosition(10.7 + control.addposition_x, 3.0 + control.addposition_y,
                                         -0.6 + control.addposition_z)
        if texture_choice == 0:
            self.objects_actor13.GetProperty().SetColor(100, 0, 0)
        # 渲染
        self.ren2.AddActor(self.objects_actor13)

        image_name_grass = r"resources\5.png"
        objects14.Update()
        # 读取图片信息
        reader14 = vtk.vtkPNGReader()
        reader14.SetFileName(image_name_grass)

        # 创建纹理对象
        texture14 = vtk.vtkTexture()
        texture14.SetInputConnection(reader14.GetOutputPort())

        # 将纹理映射到几何体上去
        map_texture_to_objects14 = vtk.vtkTextureMapToSphere()
        map_texture_to_objects14.SetInputConnection(objects14.GetOutputPort())
        map_texture_to_objects14.PreventSeamOn()

        # 几何体映射
        objects_mapper14 = vtk.vtkPolyDataMapper()
        if texture_choice == 0:
            # 没有添加纹理就绑定几何体的信息
            objects_mapper14.SetInputData(objects14.GetOutput())
        else:
            # 需要添加纹理就绑定几何体+纹理的综合信息
            objects_mapper14.SetInputConnection(map_texture_to_objects14.GetOutputPort())

        # 创建执行单元actor
        self.objects_actor14 = vtk.vtkActor()
        self.objects_actor14.SetMapper(objects_mapper14)
        if texture_choice != 0:
            self.objects_actor14.SetTexture(texture14)

        # 设置几何体的位置、形状、大小、角度
        self.objects_actor14.RotateZ(90)
        self.objects_actor14.SetScale(10, 0.3, 30)
        self.objects_actor14.SetPosition(39.3, 3.0, 0.9)
        if texture_choice == 0:
            self.objects_actor14.GetProperty().SetColor(0, 0, 0)
        # 渲染
        self.ren2.AddActor(self.objects_actor14)

        objects15.Update()
        # 读取图片信息
        reader15 = vtk.vtkPNGReader()
        reader15.SetFileName(image_name_grass)

        # 创建纹理对象
        texture15 = vtk.vtkTexture()
        texture15.SetInputConnection(reader15.GetOutputPort())

        # 将纹理映射到几何体上去
        map_texture_to_objects15 = vtk.vtkTextureMapToSphere()
        map_texture_to_objects15.SetInputConnection(objects15.GetOutputPort())
        map_texture_to_objects15.PreventSeamOn()

        # 几何体映射
        objects_mapper15 = vtk.vtkPolyDataMapper()
        if texture_choice == 0:
            # 没有添加纹理就绑定几何体的信息
            objects_mapper15.SetInputData(objects15.GetOutput())
        else:
            # 需要添加纹理就绑定几何体+纹理的综合信息
            objects_mapper15.SetInputConnection(map_texture_to_objects15.GetOutputPort())

        # 创建执行单元actor
        self.objects_actor15 = vtk.vtkActor()
        self.objects_actor15.SetMapper(objects_mapper15)
        if texture_choice != 0:
            self.objects_actor15.SetTexture(texture15)

        # 设置几何体的位置、形状、大小、角度
        self.objects_actor15.SetScale(34, 0.3, 30)
        self.objects_actor15.SetPosition(22, -2, 1)
        if texture_choice == 0:
            self.objects_actor15.GetProperty().SetColor(0, 100, 0)
        # 渲染
        self.ren2.AddActor(self.objects_actor15)

    # 用键盘移动小人
    def keyboard_rollback(self, obj, event):
            key = obj.GetKeySym()
            # 小人往左移动，清楚垂直移动信息，更新水平移动信息
            if key == 'Left':
                control.lleg_move_vertical = 0
                control.lleg_rotate_vertical = 0
                control.rleg_move_vertical = 0
                control.rleg_rotate_vertical = 0

                # 小人两腿摆开、微移
                control.move_arrow = 0
                control.addposition_z -= 1.0
                control.lleg_move_horizontal -= 0.4
                control.lleg_rotate_horizontal = 24
                control.rleg_move_horizontal += 0.4
                control.rleg_rotate_horizontal = -24
                self.ren2.RemoveAllViewProps()
                self.draw_objects(ren2=self.ren2, iren2=self.iren2, texture_choice=self.texture_choice)
                self.showObjects()

                # 小人两腿收拢，完成一个左移动作。
                control.move_arrow = 0
                control.addposition_z -= 1.0
                control.lleg_move_horizontal += 0.4
                control.lleg_rotate_horizontal = 0
                control.rleg_move_horizontal -= 0.4
                control.rleg_rotate_horizontal = 0

                self.ren2.RemoveAllViewProps()
                self.draw_objects(ren2=self.ren2, iren2=self.iren2, texture_choice=self.texture_choice)
                self.showObjects()

                return

            # 小人往右移动，与向左类似
            if key == 'Right':
                control.lleg_move_vertical = 0
                control.lleg_rotate_vertical = 0
                control.rleg_move_vertical = 0
                control.rleg_rotate_vertical = 0

                control.move_arrow = 0
                control.addposition_z += 1.0
                control.lleg_move_horizontal -= 0.4
                control.lleg_rotate_horizontal = 24
                control.rleg_move_horizontal += 0.4
                control.rleg_rotate_horizontal = -24

                self.ren2.RemoveAllViewProps()
                self.draw_objects(ren2=self.ren2, iren2=self.iren2, texture_choice=self.texture_choice)
                self.showObjects()
                control.move_arrow = 0
                control.addposition_z += 1.0
                control.lleg_move_horizontal += 0.4
                control.lleg_rotate_horizontal = 0
                control.rleg_move_horizontal -= 0.4
                control.rleg_rotate_horizontal = 0

                self.ren2.RemoveAllViewProps()
                self.draw_objects(ren2=self.ren2, iren2=self.iren2, texture_choice=self.texture_choice)
                self.showObjects()

                return

            # 小人往后移动
            if key == 'Down':
                # 清楚水平的移动信息
                control.lleg_move_horizontal = 0
                control.lleg_rotate_horizontal = 0
                control.rleg_move_horizontal = 0
                control.rleg_rotate_horizontal = 0

                # 设置左腿的移动、旋转，右腿的移动、旋转（反方向）
                control.move_arrow = 0
                control.addposition_x -= 1.0
                control.lleg_move_vertical += 0.8
                control.lleg_rotate_vertical = 30
                control.rleg_move_vertical -= 0.8
                control.rleg_rotate_vertical = -30
                self.ren2.RemoveAllViewProps()
                self.draw_objects(ren2=self.ren2, iren2=self.iren2, texture_choice=self.texture_choice)
                self.showObjects()

                # 设计为上面的代码是腿摆开移动，下面的代码是两腿收拢，形成一个逼真的效果。
                control.move_arrow = 0
                control.addposition_x -= 1.0
                control.lleg_move_vertical -= 0.8
                control.lleg_rotate_vertical = 0
                control.rleg_move_vertical += 0.8
                control.rleg_rotate_vertical = 0

                self.ren2.RemoveAllViewProps()
                self.draw_objects(ren2=self.ren2, iren2=self.iren2, texture_choice=self.texture_choice)
                self.showObjects()

                return

            # 小人往前移动，与向后类似。
            if key == 'Up':
                control.lleg_move_horizontal = 0
                control.lleg_rotate_horizontal = 0
                control.rleg_move_horizontal = 0
                control.rleg_rotate_horizontal = 0

                control.move_arrow = 0
                control.addposition_x += 1.0
                control.lleg_move_vertical += 0.8
                control.lleg_rotate_vertical = 30
                control.rleg_move_vertical -= 0.8
                control.rleg_rotate_vertical = -30
                self.ren2.RemoveAllViewProps()
                self.draw_objects(ren2=self.ren2, iren2=self.iren2, texture_choice=self.texture_choice)
                self.showObjects()

                control.move_arrow = 0
                control.addposition_x += 1.0
                control.lleg_move_vertical -= 0.8
                control.lleg_rotate_vertical = 0
                control.rleg_move_vertical += 0.8
                control.rleg_rotate_vertical = 0

                self.ren2.RemoveAllViewProps()
                self.draw_objects(ren2=self.ren2, iren2=self.iren2, texture_choice=self.texture_choice)
                self.showObjects()

                return

            # 射箭
            if key == 's':
                # 射箭标志置位
                control.move_arrow = 1
                # 箭开始移动
                while True:
                    control.arrow_position += 0.7
                    # 箭已经射到靶子上，则停止移动，重绘在弓上
                    if control.arrow_position + control.addposition_x > 30:
                        control.arrow_position = 0
                        self.ren2.RemoveActor(self.objects_actor5)
                        self.draw_objects(ren2=self.ren2, iren2=self.iren2, texture_choice=self.texture_choice)
                        self.showObjects()
                        control.move_arrow = 0
                        break
                    # 射箭过程中，不断去除之前的箭，重绘一个向前移动了一点点的箭
                    self.ren2.RemoveActor(self.objects_actor5)
                    self.draw_objects(ren2=self.ren2, iren2=self.iren2, texture_choice=self.texture_choice)
                    self.showObjects()

    # 为菜单的每个功能都创建一个函数
    def createActions(self):
        self.Load_Obj_Action = QAction('Load .OBJ',   self,  triggered=self.Load_Obj)
        self.Load_Ffd_Action = QAction('Load .FFD',   self,  triggered=self.Load_Ffd)
        self.start_FFD       = QAction('Start FFD',   self,  triggered=self.start_FFD)
        self.Scene           = QAction('Scene', self, triggered=self.Scene)
        self.Add_Texture     = QAction('Add Texture', self,  triggered=self.Add_Texture)
        self.Delete_Texture  = QAction('Delete Texture', self, triggered=self.Delete_Texture)
        self.Dots_Action     = QAction("Dots Number", self,  triggered=self.Change_Dots)
        self.Exit_Action     = QAction('Exit'     ,   self,  triggered=self.Exit)
        self.Save_Obj_Action = QAction('Save .OBJ',   self,  triggered=self.Save_Obj)
        self.Save_Ffd_Action = QAction('Save .FFD',   self,  triggered=self.Save_Ffd)
    # 创建菜单
    def createMenus(self):
        self.menuBar().setNativeMenuBar(False)
        self.File_Menu    = self.menuBar().addMenu('File')
        self.FFD_Menu     = self.menuBar().addMenu('FFD')
        self.Objects_Menu = self.menuBar().addMenu('3D objects')
        self.Txeture_Menu = self.menuBar().addMenu('Texture')


        self.File_Menu.addAction(self.Load_Obj_Action)
        self.File_Menu.addAction(self.Load_Ffd_Action)
        self.File_Menu.addAction(self.Save_Obj_Action)
        self.File_Menu.addAction(self.Save_Ffd_Action)
        self.File_Menu.addAction(self.Exit_Action)

        self.FFD_Menu.addAction(self.start_FFD)
        self.FFD_Menu.addAction(self.Dots_Action)

        self.Objects_Menu.addAction(self.Scene)


        self.Txeture_Menu.addAction(self.Add_Texture)
        self.Txeture_Menu.addAction(self.Delete_Texture)

#*******************************************************************
#      以下是所有的菜单函数
#*******************************************************************
    # 加载obj文件
    def Load_Obj(self):
        filename, ok = QFileDialog.getOpenFileName(self, 'Load .OBJ', '')
        if ok:
            self.filename = filename
            self.initFFD()
            self.showFFD()
            print("Load OBJ Success.")
    # 加载ffd文件，用self.model.sphereQt函数依次设置点位移
    def Load_Ffd(self):
        filename, ok = QFileDialog.getOpenFileName(self, 'Load .FFD', '')
        if ok:
            self.model.ffd.load_ffd(filename)
            for x in range(len(self.model.ffd.control_points)):
                for y in range(len(self.model.ffd.control_points[x])):
                    for z in range(len(self.model.ffd.control_points[x][y])):
                        x_loc_new, y_loc_new, z_loc_new = self.model.ffd.new_control_points_location[x][y][z]
                        x_loc_old, y_loc_old, z_loc_old = self.model.ffd.control_points_location[x][y][z]
                        if (x_loc_old != x_loc_new) or (y_loc_old != y_loc_new) or (z_loc_old != z_loc_new):
                            self.model.sphereQt( (x, y, z), self.model.ffd.new_control_points_location[x][y][z])

            print("Load FFD Success.")
        return
    # 保存 obj文件
    def Save_Obj(self):
        filename, ok = QFileDialog.getSaveFileName(self, 'Save .OBJ', '')
        if ok:
            self.model.ffd.save_ffd(filename)
            print("Save obj File Success.")
            return
    # 保存 ffd文件
    def Save_Ffd(self):
        filename, ok = QFileDialog.getSaveFileName(self, 'Save .FFD', '')
        if ok:
            self.model.ffd.save_ffd(filename)
            print("Save fdd File Success.")
            return
    # 设置点的数量
    def Change_Dots(self):
        DOTS, ok = QInputDialog.getInt(self, "DOTS SETTING", "Set the number of dots by edge: ", 5, 2, 8, 1)
        if ok:
            print("The dots number has been changed.")
            self.initFFD(dots=DOTS)
            self.showFFD()

    # 展示FFD
    def start_FFD(self):
        control.show_scene = 0
        self.initFFD()
        self.showFFD()

    # 展示一个射箭场景
    def Scene(self):
        control.show_scene = 1
        self.initObjects()
        self.showObjects()

    # 添加纹理
    def Add_Texture(self):
        if control.show_scene != 1:
            return
        # 清楚之前的所有actor，否则不仅会出现重叠，而且会爆栈
        self.ren2.RemoveAllViewProps()
        self.draw_objects(ren2=self.ren2, iren2=self.iren2 , texture_choice = 1)
        self.showObjects()

    # 删除纹理
    def Delete_Texture(self):
        if control.show_scene != 1:
            return
        # 清楚之前的所有actor，否则不仅会出现重叠，而且会爆栈
        self.ren2.RemoveAllViewProps()
        self.draw_objects(ren2=self.ren2, iren2=self.iren2 , texture_choice = 0)
        self.showObjects()

    # 退出
    def Exit(self):
        exit(0)


# 自由变形启动
def Free_Deformation_start():
    app = QApplication(sys.argv)
    window = Qt_Window_Menu()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    Free_Deformation_start()

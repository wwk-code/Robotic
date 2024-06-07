# 项目说明

  所有我已完成的lab及其相关文档都会实时推送至这个项目仓库，在Ubuntu20.04系统中拉取这个项目并且执行其中的关键 py 文件即可，每个lab所关联的文件都会以单独的文件夹形式存放，同时每个lab子文件夹下都会包含对应lab的详细教程(README.md文档),搭环境跑项目时要注意跟着README.md文档一步一步走



###  摄像头问题解决方案

0. 首先点击Vmware上方菜单栏中的"虚拟机->可移动设备->*** Camera ，选择连接至虚拟机"

接下来基本上就配置成功了，选择再运行一下程序看看行了没

1.使用 lsusb 命令 检查 USB 设备看看Camera连接虚拟机成功了没有(可能需要设置一下Vmware的USB设置)

2. 给权限:  sudo usermod -aG video $USER
3. 重新加载视频驱动程序 ① sudo modprobe -r uvcvideo ② sudo modprobe uvcvideo
4. 使用 v4l2-ctl --list-devices，看看有没有 video0 ,有的话基本上成功了

查看哪个进行正在使用摄像头: sudo lsof /dev/video0
然后杀死进程: kill -9 pid




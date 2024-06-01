# lab1

### 1. 从0搭建 Robotic Lab1 开发环境 - 操作流程

``` C++
1. 使用Vmware搭建Ubuntu20.04系统,VMware虚拟网络编辑器设置中使用NAT静态IP(非DHCP模式),然后在 /etc/netplan/01-network-manager-all.yaml 中设置如下内容:
network:
    version: 2
    renderer: NetworkManager
    ethernets:
      ens33:
        dhcp4: no
        addresses:
          - 192.168.52.120/24 (要看Vmware虚拟网络编辑器中NAT下的对应Ip地址然后做对应修改)
        optional: true
        gateway4: 192.168.52.2  (看Vmware虚拟网络编辑器的网关地址(一样即可))
        nameservers:
          addresses:
            - 8.8.8.8
            - 114.114.114.114
然后给apt确定好源,下载好 ping、SSH，保证可以ping通google和vscode SSH远程登录，同时可以配置git的http和https代理                
2. 安装基础开发环境： 通过apt下载Python、G++、GCC,Pip再安装numpy
3. 根据Tutorial1安装Eigen库，根据Tutorial2文档搭建好对应的ROS环境，跑出其小海龟程序
4. 设置Vmware的USB为3.0协议，按照上方的摄像头配置做好配置，然后跑通我的lab1
```



### 2. 运行项目

``` C++
老师发的资料是本目录下的 "Robotics_Lab1_基于颜色特征的目标识别与追踪实验.pdf"，但此lab比较简单，主要注意以下几个事项:
① lab1的所有相关代码都在 Robotics_Lab1_Code 文件夹下，在已经搭建好lab1的开发环境的背景下，最终运行项目结果给老师看只需要在该目录下执行命令:
python(或 python3) hist.py 即可。  例:  python3 hist.py
```



### 3. 给老师演示时的注意事项

```C++
1. 理论上来说第二次实验应该是演示lab2或者lab3的，但我第一次去的时候没有演示完就走了，如果老师还记得这个情况，并且要你演示lab1,就直接按照上述命令运行给它看结果，如果他要你解释项目运行情况，注意以下几点:
① 了解lab1的主题: lab1是一个比较简单的使用openCV去处理一个纯颜色图片的灰度直方图，并通过窗口输出这个灰度直方图(向老师传达你了解这个主题的信息之后，他基本上就不会再问了，尽量含糊一点就行)
② 如果他要你具体解释其中的函数 (基本上没可能)，那就看我的报告: "SA23225161_吴维康_Lab1.docx"，照着念就行
```


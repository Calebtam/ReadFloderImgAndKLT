<!-- 先创建一个包 -->
catkin_create_pkg --rosdistro ROSDISTRO dynamic_tutorials rospy roscpp dynamic_reconfigure



<!-- 当然，这些修改的参数要在其他类中的方法使用，还应该提供一个参数更新的接口： -->
search_.configUpdate(min_frontier_size_, potential_scale_, gain_scale_, information_scale_, information_r_);

 <!-- 最后，可以使用dynparam dump将调好的参数写到文件中： -->
rosrun dynamic_reconfigure dynparam dump /your_node dump.yaml

<!-- 也可以使用dynparam load将参数文件再次加载： -->
rosrun dynamic_reconfigure dynparam load /node dump.yaml







[原文](https://blog.csdn.net/qq_52785580/article/details/128693760)
#### 文章目录

[文章目录](https://blog.csdn.net/qq_52785580/article/details/128693760#%E6%96%87%E7%AB%A0%E7%9B%AE%E5%BD%95)

[前言](https://blog.csdn.net/qq_52785580/article/details/128693760#%E5%89%8D%E8%A8%80)

[一、编写.cfg文件](https://blog.csdn.net/qq_52785580/article/details/128693760#%E4%B8%80%E3%80%81%E7%BC%96%E5%86%99.cfg%E6%96%87%E4%BB%B6)

[二、为节点配置dynamic\_reconfigure](https://blog.csdn.net/qq_52785580/article/details/128693760#%E4%BA%8C%E3%80%81%E4%B8%BA%E8%8A%82%E7%82%B9%E9%85%8D%E7%BD%AEdynamic_reconfigure)

[总结](https://blog.csdn.net/qq_52785580/article/details/128693760#%E6%80%BB%E7%BB%93)

___

## 前言

dynamic\_reconfigure配置是ROS中为了方便用户对程序中的参数进行实时调整而推出的工具，配置好自己的dynamic\_reconfigure文件后，可以很方便的使用ROS提供的rqt\_reconfigure工具对程序的参数进行合理调整，以获得最优的性能。

例如：[move\_base](https://so.csdn.net/so/search?q=move_base&spm=1001.2101.3001.7020)中就针对costmap、planner等设置了很多动态调整的参数，可以方便用户在使用过程中调整得到合适的参数。

又例如：如果我们开发了一个PID控制器程序，如果这时能通过rqt\_reconfigure工具，对PID参数进行合理的调整，然后直接将获得的参数写进程序，就能使工作更加高效。

## 一、编写.[cfg文件](https://so.csdn.net/so/search?q=cfg%E6%96%87%E4%BB%B6&spm=1001.2101.3001.7020)

**1.在功能包文件夹下，创建一个config文件夹（其他名字也行，比如cfg）**

![](https://img-blog.csdnimg.cn/081bf991d9834f03a251ed638054f314.png)

在其中创建一个后缀为.cfg的文件

![](https://img-blog.csdnimg.cn/10240fe4716b475bb9c084637ff38139.png)

**2.在其中写入内容**

```python
from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()

type_enum = gen.enum([gen.const("centroid", str_t, "centroid", "Use Centroid"),

gen.const("initial", str_t, "initial", "Use Initial"),

gen.const("middle", str_t, "middle", "Use Middle")],

"An enum to set frontier type")

gen.add("timeout", double_t, 0, "Explore Timeout(s)", 30.0, 10.0, 100.0)

gen.add("min_frontier_size", double_t, 0, "Min Frontier Size(m)", 0.75, 0.5, 2)

gen.add("visualize", bool_t, 0, "Is Visualize?", True)

gen.add("frontier_type", str_t, 0, "Select Frontier Type", "centroid", edit_method=type_enum)

gen.add("potential_scale", double_t, 0, "Distance Weight", 3.0)

gen.add("gain_scale", double_t, 0, "Gain Weight", 1.0)

gen.add("information_scale", double_t, 0, "Information Weight", 1.2)

gen.add("information_r", double_t, 0, "Information Radius", 3.0)

exit(gen.generate("explore_server", "explore_server", "Explore"))
```

进行逐行解析：

```python
from dynamic_reconfigure.parameter_generator_catkin import *
```

说明这是一个python文件，使用/usr/bin/env下的python解析，并引入dynamic\_reconfigure头文件

```python
gen = ParameterGenerator()
```

创建一个参数生成器，然后可以使用add函数添加动态配置的参数，add函数的参数如下：

-   **name** - 一个字符串，指定应存储此参数的名称
    
-   **paramtype** - 定义存储值的类型，可以是 int\_t、double\_t、str\_t 或 bool\_t 中的任何一个
    
-   **level** - 稍后将传递给动态重新配置回调的位掩码。当回调被调用时，所有已更改参数的级别值将被或运算在一起，并将结果值传递给回调。
    
-   **description** - 描述参数的字符串
    
-   **default** - 指定默认值
    
-   **min** - 指定最小值（可选，不适用于字符串和布尔值）
    
-   **max** - 指定最大值（可选，不适用于字符串和布尔值）
    

例如：

```python
gen.add("timeout", double_t, 0, "Explore Timeout(s)", 30.0, 10.0, 100.0)
```

我想动态设置程序超时时间（timeout），该变量为double类型，位掩码为0，描述是"Explore Timeout(s)"，默认值为30.0，最小值为：10.0，最大值为：100.0，这个配置在rqt\_reconfigure中的显示如下：

![](https://img-blog.csdnimg.cn/23de4b9baa7e415186de293a959ea776.png)

 布尔类型的值没有最大最小值，它在程序中的显示如下：

![](https://img-blog.csdnimg.cn/76b2d967d2ee4c059f55e8c24d0ae9e3.png)

```python
type_enum = gen.enum([gen.const("centroid", str_t, "centroid", "Use Centroid"),

gen.const("initial", str_t, "initial", "Use Initial"),

gen.const("middle", str_t, "middle", "Use Middle")],

"An enum to set frontier type")
```

这里我们可以使用gen.num创建一个枚举类型的变量，让用户可以通过下拉列表选择值，gen.num中有两个参数，一个是gen.const组成的列表，gen.const参数结构和gen.add一样，第二个参数就是枚举类型的描述。

```python
gen.add("frontier_type", str_t, 0, "Select Frontier Type", "centroid", edit_method=type_enum)
```

然后可以通过edit\_method=type\_enum参数将枚举类型添加进去，最后的显示效果如下：

![](https://img-blog.csdnimg.cn/d2a3a2e56c4a4182a3e80ba315919ec0.png)

最后，使用gen.generate生成config文件，并使用exit退出

```python
exit(gen.generate("explore_server", "explore_server", "Explore"))
```

 gen.generate第一个参数是config的“命名空间”（功能包名），第二个参数是可以在其中运行的节点的名称（仅用于生成文档），第三个参数是生成的文件将获得的名称前缀。

具体来说，第一个参数是我们使用生成的Config文件的命名空间（功能包名）

![](https://img-blog.csdnimg.cn/a451c29fc6014aec8424e05812b0b59e.png)

第二个参数是运行时的节点名

![](https://img-blog.csdnimg.cn/9f6e9df9638141afa284d328eef946df.png)

第三个参数是生成配置头文件的前缀，我们需要包含我们生成的配置头文件

![](https://img-blog.csdnimg.cn/6ad62291d5c14fa58483e72b6a425f83.png)

 Explore就是头文件的前缀，可以在devel/include/功能包名 看到这个生成的头文件（这和msgs生成类似）

![](https://img-blog.csdnimg.cn/ecd06b1e2d294eb3a7ead2639cfcb055.png)

**3.CmakeLists.txt和package.xml文件修改** 

首先需要在find\_package和catkin\_package函数中添加dynamic\_reconfigure包的依赖

![](https://img-blog.csdnimg.cn/e20939189ed24f47a2b52f2f8950d3e9.png)

然后在generate\_dynamic\_reconfigure\_options函数中添加.cfg文件，这与生成.msg类似，config/是你定义的文件夹名

![](https://img-blog.csdnimg.cn/e7a64e2026a94e00b97928e8ba9d3490.png)

最后需要在add\_dependencies中添加依赖项${PROJECT\_NAME}\_gencfg

![](https://img-blog.csdnimg.cn/e6af80de7c8c42559738ff39a85a6d74.png) 在package.xml文件中添加dynamic\_reconfigure的依赖

![](https://img-blog.csdnimg.cn/fafd240688b94f0780be387101800a64.png)

## 二、为节点配置dynamic\_reconfigure

### 

配置好.cfg文件后，就可以在我们自己编写的节点中使用生成的配置文件了。

首先引入头文件：

![](https://img-blog.csdnimg.cn/6ad62291d5c14fa58483e72b6a425f83.png)

第一个是dynamic\_reconfigure服务器的头文件，用于创建服务器，第二个是我们自己写的.cfg文件生成的头文件，用于使用我们定义的参数。

```cpp
dynamic_reconfigure::Server<ExploreConfig> configServer;

dynamic_reconfigure::Server<ExploreConfig>::CallbackType cb;
```

然后，我们可以创建两个变量，第一个是动态配置参数服务器，第二个是回调函数，也就是我们使用rqt\_reconfigure调参时进入的函数，需要我们自己编写。

```cpp
void Explore_Server::configCb(ExploreConfig& config, uint32_t level)

{

ROS_INFO("Dynamic Config Start");

timeout = config.timeout;

min_frontier_size_ = config.min_frontier_size;

visualize_ = config.visualize;

frontier_type_ = config.frontier_type;

potential_scale_ = config.potential_scale;

gain_scale_ = config.gain_scale;

information_scale_ = config.information_scale;

information_r_ = config.information_r;

}
```

然后可以在程序中编写自己的回调函数，我这里进行了赋值操作，当然也可以进行其他操作，这里configCb之前的Explore\_Server::是我程序中的命名空间，非必需，大家根据自己程序编写。

```cpp
cb = boost::bind(&Explore_Server::configCb, this, _1, _2);

configServer.setCallback(cb);
```

然后使用bind函数将回调函数绑定后赋值给cb变量，然后使用setCallback函数为服务器设置回调函数。

当然，这些修改的参数要在其他类中的方法使用，还应该提供一个参数更新的接口：

```cpp
search_.configUpdate(min_frontier_size_, potential_scale_, gain_scale_, information_scale_, information_r_);
```

 最后，可以使用dynparam dump将调好的参数写到文件中：

```bash
rosrun dynamic_reconfigure dynparam dump /your_node dump.yaml
```

也可以使用dynparam load将参数文件再次加载：

```bash
rosrun dynamic_reconfigure dynparam load /node dump.yaml
```

___

## 总结

到此为止就实现了一个动态参数配置的节点和我们自定义个.cfg文件，在运行程序时，我们就可以使用rqt\_configure工具，实时的动态调节程序的参数，帮助我们高效便捷的获得最优程序参数。
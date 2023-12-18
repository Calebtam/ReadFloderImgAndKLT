import matplotlib.pyplot as plt
import numpy as np
import time  #引入time模块

t1 = time.gmtime() # gmtime()用来获取当前时间
t2 =  time.gmtime()
print(t1 == t2) #输出True
print(t1 is t2) #输出False

'''
x = np.linspace(0, 20, 100)  # Create a list of evenly-spaced numbers over the range
plt.plot(x, np.sin(x))       # Plot the sine of each x point
plt.show()                   # Display the plot


qweqwdada
'''
# c++
# https://c.biancheng.net/view/2199.html

# python 
# https://c.biancheng.net/view/4360.html

num = 10
print(type(num))

# Python 3.x 允许使用下划线_作为数字（包括整数和小数）的分隔符。
print(type(8_888_888_888_888_888_888_888))
print(type(12 + 0.2j))
print(2.1E5)

print("引文双引号是\"，中文双引号是“")
print('引文双引号是"，中文双引号是“' + "  I'm a great coder!")
# 加上r前缀，就变成了原始字符串
print(r'D:\Program Files\Python 3.8\python.exe')


# bytes 只是简单地记录内存中的原始数据
b5 = "C语言中文网8岁了".encode('UTF-8')
print("b5: ", b5)
print("str1: ", b5.decode('UTF-8'))

# input str = input(tipmsg)
# str 表示一个字符串类型的变量，input 会将读取到的字符串放入 str 中。
# ipmsg 表示提示信息，它会显示在控制台上，告诉用户应该输入什么样的内容；如果不写 tipmsg，就不会有任何提示信息。
# 使用 Python 内置函数将字符串转换成想要的类型，比如：
# int(string) 将字符串转换成 int 类型；
# float(string) 将字符串转换成 float 类型；
# bool(string) 将字符串转换成 bool 类型。

user_name = 'Charlie'
user_age = 8
print("读者名：" ,user_name,"年龄：",user_age,sep='|')

# file 参数指定 print() 函数的输出目标，file 参数的默认值为 sys.stdout，该默认值代表了系统标准输出，也就是屏幕，因此 print() 函数默认输出到屏幕。实际上，完全可以通过改变该参数让 print() 函数输出到特定文件中，例如如下代码：
# 上面程序中，open() 函数用于打开 demo.txt 文件，接连 2 个 print 函数会将这 2 段字符串依次写入此文件，最后调用 close() 函数关闭文件，教程后续章节还会详细介绍关于文件操作的内容。
# print() 函数的 flush 参数用于控制输出缓存，该参数一般保持为 False 即可，这样可以获得较好的性能。

# f = open("demo.txt","w")#打开文件以便写入
# print('沧海月明珠有泪',file=f)
# print('蓝田日暖玉生烟',file=f)
# f.close()

n = 123456
# %09d 表示最小宽度为9，左边补0
print("n(09): %09d" % n)
# %+9d 表示最小宽度为9，带上符号
print("n(+9): %+9d" % n)
f = 140.5
# %-+010f 表示最小宽度为10，左对齐，带上符号
print("f(-+0): %-+010f" % f)
s = "Hello"
# %-10s 表示最小宽度为10，左对齐
print("s(-10): %-10s." % s)
f = 3.141592653
# 最小宽度为8，小数点后保留3位
print("%8.3f" % f)
# 最小宽度为8，小数点后保留3位，左边补0
print("%08.3f" % f)
# 最小宽度为8，小数点后保留3位，左边补0，带符号
print("%+08.3f" % f)

#使用\t排版
str1 = '网站\t\t域名\t\t\t年龄\t\t价值'
str2 = 'C语言中文网\tc.biancheng.net\t\t8\t\t500W'
str3 = '百度\t\twww.baidu.com\t\t20\t\t500000W'
print(str1)
print(str2)
print(str3)

# age = int(input("请输入年龄："))
# height = int(input("请输入身高："))
# if age>=18 and age<=30 and height >=170 and height <= 185 :
#     print("恭喜，你符合报考飞行员的条件")
# else:
#     print("抱歉，你不符合报考飞行员的条件")
    
a = 20
b = 10
c = 30
d = 40
# if (a > b) a
# else if(b > c) b
# else if (c > d) c
# else d
e = a if a > b else b if b > c else c if c>d else d
print(e)

# 序列
str="C语言中文网"
print(str[0],"==",str[-6])
# 隔 1 个字符取一个字符，区间是整个字符串
print(str[::2])
# 检查字符‘c’是否包含在字符串“c.biancheng.net”中
print('C'in str)
# 字符长度
print(len(str))
# 找出最小的字符
print(min(str))
# 对字符串中的元素进行排序
print(sorted(str))

# list 
type( ["http://c.biancheng.net/python/", 1, [2,3,4] , 3.0] )

#将字典转换成列表
dict1 = {'a':100, 'b':42, 'c':9}
list3 = list(dict1)
print(list3)

url = list("http://c.biancheng.net/shell/")
# 使用索引访问列表中的某个元素
print(url[3])  #使用正数索引
print(url[-4])  #使用负数索引
#使用切片访问列表中的一组元素
print(url[9: 18])  #使用正数切片
print(url[9: 18: 3])  #指定步长
print(url[-6: -1])  #使用负数切片
# 删除列表
del url

l = ['Python', 'C++', 'Java']
#追加元素
l.append('PHP')
print(l)
#追加列表，整个列表也被当成一个元素
l.append(['Ruby', 'SQL'])
print(l)
# extend() 不会把列表或者元祖视为一个整体，而是把它们包含的元素逐个添加到列表中。
# 追加元组，元祖被拆分成多个元素
t = ('JavaScript', 'C#', 'Go')
l.extend(t)
print(l)
#插入字符串，整个字符串被当成一个元素
l.insert(0, "http://c.biancheng.net")
print(l)
# 删除
del l[2]
print(l)
# pop 如果不写 index 参数，默认会删除列表中的最后一个元素
l.pop(3)
print(l)
l.remove('Ruby')
# clear
l.clear()
 
# 改
nums = [40, 36, 89, 2, 36, 100, 7]
#在4个位置插入元素
nums[4: 4] = [-77, -52.5, 999]
print(nums)

# 查
nums = [40, 36, 89, 2, 36, 100, 7, -20.5, -999, 1000, 36]
#检索列表中的所有元素    返回index
print( nums.index(2) )
# 检索3~7之间的元素     返回index
print( nums.index(100, 3, 7) )
# 检索4之后的元素       返回index
print( nums.index(7, 4) )

print("36出现了%d次" % nums.count(36))

# python 
# https://c.biancheng.net/view/4360.html

# remove()：根据元素值进行删除
# height=float(input("输入身高： (m)")) #输入身高
# weight=float(input("输入体重： (kg)")) #输入体重
# bmi=weight/(height*height)       #计算BMI指数
# #判断身材是否合理
# if bmi<18.5:
#     #下面 2 行同属于 if 分支语句中包含的代码，因此属于同一作用域
#     print("BMI指数为："+str(bmi)) #输出BMI指数
#     print("体重过轻")
# if bmi>=18.5 and bmi<24.9:
#     print("BMI指数为："+str(bmi)) #输出BMI指数
#     print("正常范围，注意保持")
# if bmi>=24.9 and bmi<29.9:
#     print("BMI指数为："+str(bmi)) #输出BMI指数
#     print("体重过重")
# if bmi>=29.9:
#     print("BMI指数为："+str(bmi)) #输出BMI指数
#     print("肥胖")
# if bmi<14.5:
#     print("bmi < 14.5 " + "真实值为 " + str(bmi))

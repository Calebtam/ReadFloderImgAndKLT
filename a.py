import matplotlib.pyplot as plt
import numpy as np
'''
x = np.linspace(0, 20, 100)  # Create a list of evenly-spaced numbers over the range
plt.plot(x, np.sin(x))       # Plot the sine of each x point
plt.show()                   # Display the plot


qweqwdada
'''
# c++
# https://c.biancheng.net/view/2199.html

# python 
# https://c.biancheng.net/view/2175.html

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

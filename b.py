import cv2
import os
import math
import numpy as np
import matplotlib.pyplot as plt

def compare_images(image_paths):
    # Load images
    images = [cv2.imread(image_path) for image_path in image_paths]

    # Calculate contrast
    gray_images = [cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) for image in images]
    contrast = [cv2.compareHist(gray_images[i], gray_images[i+1], cv2.HISTCMP_CORREL) for i in range(len(images)-1)]
    
    # Calculate sharpness
    laplacians = [cv2.Laplacian(gray_image, cv2.CV_64F).var() for gray_image in gray_images]
    sharpness = [abs(laplacians[i] - laplacians[i+1]) for i in range(len(images)-1)]
    
    # Calculate gradient mean
    gradients = []
    for gray_image in gray_images:
        sobelx = cv2.Sobel(gray_image, cv2.CV_64F, 1, 0, ksize=5)
        sobely = cv2.Sobel(gray_image, cv2.CV_64F, 0, 1, ksize=5)
        gradients.append(cv2.mean(cv2.magnitude(sobelx, sobely))[0])
    gradient_mean = [abs(gradients[i] - gradients[i+1]) for i in range(len(images)-1)]

    # Print results
    for i in range(len(images)-1):
        print(f"Comparison between {image_paths[i]} and {image_paths[i+1]}:")   
        print(f"Contrast: {contrast[i]}")
        print(f"Sharpness: {sharpness[i]}")
        print(f"Gradient Mean: {gradient_mean[i]}")
        print()

def statistic_image(img):
    # 统计图像的直方图
    hist = cv2.calcHist([img], [0, 1, 2], None, [8, 8, 8], [0, 256, 0, 256, 0, 256])
    
    # 计算对比度
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    hist = cv2.calcHist([gray], [0], None, [256], [0, 256])
    contrast = hist.std() / hist.mean()

    # 计算清晰度
    laplacian = cv2.Laplacian(gray, cv2.CV_64F).var()
    sharpness = laplacian / img.size

    # 计算梯度
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=5)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=5)
    gradient = cv2.addWeighted(cv2.convertScaleAbs(sobelx), 0.5, cv2.convertScaleAbs(sobely), 0.5, 0)
    gradient1 = cv2.mean(gradient)

    # 输出结果
    print('对比度：', contrast)
    print('清晰度：', sharpness)
    print('梯度：', gradient1)
    # plt.imshow(hist, interpolation="nearest")
    plt.plot(hist)
    plt.xlim([0, 256])
    plt.show()

def compare_images_histogram(img1, img2):
    # 将图片转换为HSV颜色空间
    img1_hsv = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
    img2_hsv = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)

    # 计算图片的直方图
    hist1 = cv2.calcHist([img1_hsv], [0, 1], None, [180, 256], [0, 180, 0, 256])
    hist2 = cv2.calcHist([img2_hsv], [0, 1], None, [180, 256], [0, 180, 0, 256])
    print('hist1', hist1.shape)
    print('hist2', hist2.shape)
    
    # 归一化直方图
    cv2.normalize(hist1, hist1, 0, 1, cv2.NORM_MINMAX, -1)
    cv2.normalize(hist2, hist2, 0, 1, cv2.NORM_MINMAX, -1)

    # 计算直方图的差异
    similarity = cv2.compareHist(hist1, hist2, cv2.HISTCMP_CORREL)

    return similarity

## 遍历一个文件夹下的所有图像 
def bianli_pics(path):

    img_folder = path
    img_list = [os.path.join(nm) for nm in os.listdir(img_folder) if nm[-3:] in ['jpg', 'png', 'gif']]
    img_list.sort(key=lambda x: int(x[-5:-4]))
    print(img_list) #将所有图像遍历并存入一个列表
    ## ['test_14.jpg', 'test_15.jpg', 'test_9.jpg', 'test_17.jpg', 'test_16.jpg']
    image_paths = []

    for i in img_list:
          
        imgpath=os.path.join(path,i)
        image_paths.append(imgpath)
        # print(imgpath)
        ## ./input/test_14.jpg
		## ./input/test_15.jpg
        # image = cv2.imread(imgpath) ## 逐个读取
        # plt.imshow(image) 
        # plt.show()
        # statistic_image(image)
    compare_images(image_paths)
    
if __name__ == '__main__':
    base_path = r'./L'
    bianli_pics(base_path)
    # files = os.listdir(base_path)
    # files.remove('.DS_Store') ## Mac系统中可能会存在.DS_Store，提前将其删除
    # files.sort(key=lambda x: int(x.split('.')[0])) ## 使用切片将图片名称单独切开
    # for path in files:
    #     full_path = os.path.join(base_path, path)
    #     # print(full_path)
    #     with open(full_path) as fp:
    #         data = fp.read()
    #         print(data)
            
    # # 加载图像
    # img_copy = cv2.imread('D://corn.png')
    # img_rgb = cv2.cvtColor(img_copy, cv2.COLOR_BGR2RGB)
    # width = 64
    # height = 128
    # # img_copy = img[320:320 + height, 570:570 + width][:, :, ::-1]
    # # gray_copy = cv2.cvtColor(img_copy, cv2.COLOR_BGR2GRAY)
    # gray_copy = cv2.cvtColor(img_copy, cv2.COLOR_BGR2GRAY)
    # # 显示原图像
    # plt.figure(figsize=(6.4, 2.0 * 3.2))
    # plt.subplot(1, 2, 1)
    # plt.imshow(img_rgb)

    # # HOG特征提取
    # hog = Hog_descriptor(gray_copy, cell_size=8, bin_size=9)
    # hog_vector, hog_image = hog.extract()
    # print('hog_vector', hog_vector.shape)
    # print('hog_image', hog_image.shape)

    # # 绘制特征图
    # plt.subplot(1, 2, 2)
    # plt.imshow(hog_image, cmap=plt.cm.gray)
    # plt.show()

# 示例用法
# img1_path = 'image1.jpg'
# img2_path = 'image2.jpg'
# similarity = compare_images_histogram(img1_path, img2_path)
# print('相似度：', similarity)
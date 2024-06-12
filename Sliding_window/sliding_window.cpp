#include <iostream>
#include <vector>
#include <queue>

class SlidingWindow {
public:
    //      size_ : size of sliding window  
    //      ratio : sta == 2 in all windowSize ratio
    //      sta_thr : stable pose when the sta threshold
    SlidingWindow(int size_, double ratio_, double sta_thr_) : windowSize(size_), step(0), sum_sta(0), sum_yaw(0), sum_x_dis(0), sum_y_dis(0), ratio(ratio_), sta_thr(sta_thr_) {
        // 初始化队列和变量
        wait_period = 0;
        wait_count = 0;
    }

    // step = 0 : 等待数据
    // step = 1 : sta数据足够
    // step = 2 : sta数据足够且状态均值大于1.5                    告诉外面要停车
    // step = 3 : sta均值大于1.5 且 有效数据量足够                告诉可以获取位姿   
    // step = 4 : 停下来求解位姿不符合要求，只有一次后面就会进入2     重新往前走3-5cm  
    int addValue(int val, double yaw_, double x_dis_, double y_dis_) {
        // 添加新的值到滑动窗口
        bool sta_pop = false;
        q_sta.push(val);
        sum_sta += val;
        
        // 数据堆处理
        if (q_sta.size() > windowSize) {
            sum_sta -= q_sta.front();
            q_sta.pop();
            sta_pop = true;

            if(step == 0){ 
                // statistic sta data， and judge stop the robot
                step = 1;
            }
            // step = 1;  dont need to judge again

            // only when the car was stoped, we want to statistic the yaw x_dis and y_dis
            if(step == 2)
            {
                if(val >= 2)
                {
                    q_yaw.push(yaw_);
                    q_x_dis.push(x_dis_);
                    q_y_dis.push(y_dis_);
                    sum_yaw += yaw_;
                    sum_x_dis += x_dis_;
                    sum_y_dis += y_dis_;
                } 

                if (sta_pop) {
                    // 如果窗口大小超过限制，移除最旧的值
                    if(val >= 2){
                        sum_yaw -= q_yaw.front();
                        sum_x_dis -= q_x_dis.front();
                        sum_y_dis -= q_y_dis.front();
                        q_yaw.pop();
                        q_x_dis.pop();
                        q_y_dis.pop();
                    }
                }
            }
        }
        else{
            // wait sta data
            return 0;
        }

        if(step == 1)
        {    
            if(q_sta.size() >= windowSize)
            {
                // jump to next, stop the robot, clear old and wait new data
                if(getStaAverage() > sta_thr)
                {
                    // i want stop the robot
                    step = 2;
                    wait_count = 0;

                    // if i was stopped, i want to clear the yaw x_dis and y_dis
                    std::queue<double> empty;
                    std::swap(q_yaw, empty);
                    std::swap(q_x_dis, empty);
                    std::swap(q_y_dis, empty);
                }
                // else if(getAverage() < 0.5)
                // {
                //     step = 1;
                // }
            }
            else 
                step = 0;   // wait sta data
        }
        if(step == 2)
        {
            // sta dont keep
            // wait 2 * windowsize, we cant get the robot pose, 
            if(getStaAverage() < (1.0 + 1e-6) && wait_count >= (2*windowSize))
            {
                step = 4;
                std::queue<double> empty;
                std::swap(q_yaw, empty);
                std::swap(q_x_dis, empty);
                std::swap(q_y_dis, empty);
            }
            else if(check())
            {
                // you can get the yaw x_dis and y_dis
                step = 3;
                wait_count = 0;
            }
            else
            {
                // dont enough data, we will to get more, wait moment
                step = 2;
            }
        }
        // return wait sta can get the robot pose
        if(step == 4)
        {
            // we will move little froward and jump back to wait sta suitable 
            wait_period++;
            if(wait_period > 3)
            {
                // cant get the robot pose after three times move， robot callback error
                std::queue<int> emptyi;
                std::swap(q_sta, emptyi);
                std::queue<double> empty;
                std::swap(q_yaw, empty);
                std::swap(q_x_dis, empty);
                std::swap(q_y_dis, empty);
                step = 0;
                // return -1;
            }
            else
            {   
                // equ step = 1;
                wait_count = 0;
                if(q_sta.size() >= windowSize)
                {
                    if(getStaAverage() > sta_thr)
                    {
                        // i want stop the robot
                        step = 2;
                        wait_count = 0;

                        // if i was stopped, i want to clear the yaw x_dis and y_dis
                        std::queue<double> empty;
                        std::swap(q_yaw, empty);
                        std::swap(q_x_dis, empty);
                        std::swap(q_y_dis, empty);
                    }
                }
                else 
                    step = 0;   // wait sta data
            }
        }
        return step;
    }


    bool getAverage(double &yaw_, double &x_dis_, double &y_dis_) {
        if(step == 3)
        {
            yaw_ = sum_yaw / q_yaw.size();
            x_dis_ = sum_x_dis / q_x_dis.size();
            y_dis_ = sum_y_dis / q_y_dis.size();
            return true;
        }
        else
        {
            // std::cout << "Not enough values to calculate y_dis average." << std::endl;
            std::cout << " 3 dont equ step: " << step << std::endl;
            return false;
        }
    }

    // double getVariance() {
    //     // 计算方差
    //     double avg = getStaAverage();
    //     double variance = 0.0;

    //     std::queue<int> tempsta = q_sta;
    //     while (!tempsta.empty()) {
    //         int val = tempsta.front();
    //         tempsta.pop();
    //         variance += (val - avg) * (val - avg);
    //     }

    //     return variance / q_sta.size();
    // }

private:

    double getStaAverage() {
        // 计算均值
        if(q_sta.size() >= windowSize)
            return static_cast<double>(sum_sta) / q_sta.size();
        else
        {
            std::cout << "Not enough values to calculate average." << std::endl;
            return 0.0;
        }
    }
    bool check() {
        // 足够的有效值
        if(q_yaw.size() >= ratio * q_sta.size()
            &&q_x_dis.size() >= ratio * q_sta.size()
            &&q_y_dis.size() >= ratio * q_sta.size())
            return true;
        else
        {
            std::cout << "Not enough values to calculate yaw average." << std::endl;
            return false;
        }
    }

    int windowSize;
    std::queue<int> q_sta;
    std::queue<double> q_yaw;
    std::queue<double> q_x_dis;
    std::queue<double> q_y_dis;
    int sum_sta;
    double sum_yaw;
    double sum_x_dis;
    double sum_y_dis;
    double ratio;
    double sta_thr;
    int wait_period;
    int wait_count;
    int step;
};

int main() {
    // 创建一个窗口大小为 5 的滑动窗口
    SlidingWindow sw(100, 0.6, 1.2);

    // 添加一些示例值
    int i = 0;
    while(i < 50)
    {
        sw.addValue(2, 19, 18);
        sw.addValue(1, 19, 18);
        sw.addValue(2, 9, 8);
        // 获取均值和方差
        std::cout << "Average: " << sw.getAverage() << std::endl;
        if(sw.getAverage() > 1.6)
        {
            std::cout << "yaw: " << sw.getyawAverage() << std::endl;
            std::cout << "y_dis: " << sw.getyDisAverage() << std::endl;
        }
        std::cout << std::endl;
    }
    // // 获取均值和方差
    // std::cout << "Average: " << sw.getAverage() << std::endl;
    // std::cout << "Variance: " << sw.getVariance() << std::endl;

    return 0;
}

#include<iostream>
#include<cstdio>
#include<cstring>
#include<vector>
#include<cmath>
using namespace std;

const int MaxN = 101;
// int wx_[MaxN], wy_[MaxN]; //顶标
// int MatchY[MaxN];
int* x2y_;
int* y2x_;
bool* visited_x_;
bool* visited_y_;
double* wx_;
double* wy_;
// int* x2y_;
// int* y2x_;



// int visited_x_[MaxN], visited_y_[MaxN];
// int Slack[MaxN];
int num_x_, num_y_;

const double epsilon = 1e-10;
int w[MaxN][MaxN];
#define cost_matrix w

double min_d_;

bool FindExtendedPath(int& x_index)
{
    visited_x_[x_index] = true;
    for (int i=0; i<num_y_; ++i) {
        if (!visited_y_[i]) {
            double d = wx_[x_index] + wy_[i] - cost_matrix[x_index][i];
//                std::cout<<"wx["<<x_index<<"] : "<<wx_[x_index]<<std::endl;
//                std::cout<<"wy["<<i<<"] : "<<wy_[i]<<std::endl;
//                std::cout<<"cost_matrix : "<<cost_matrix[x_index,i)<<std::endl;
//                std::cout<<"diff : "<<d<<std::endl;
            if (d < epsilon) {
//                    std::cout<<"coming!"<<std::endl;
                visited_y_[i] = true;
//                    std::cout<<"y2x_["<<i<<"] : "<<y2x_[i]<<std::endl;
                if (y2x_[i] == -1 || FindExtendedPath(y2x_[i])) {
//                        std::cout<<"find one!"<<std::endl;
                    x2y_[x_index] = i;
                    y2x_[i] = x_index;
                    return true;
                }
            } else {
                if (min_d_ > d) min_d_ = d;
            }
        }
    }
    return false;
}
void GetMatch() {
//        std::cout<<"cost matrix : \n"<<cost_matrix<<std::endl;
//     if (cost_matrix.rows==0 || cost_matrix.rows>cost_matrix.cols) {
// //            std::cout<<"Cost matrix is not qualified!"<<std::endl;
//         return;
//     }
    // num_x_ = cost_matrix.rows;
    // num_y_ = cost_matrix.cols;
    double inf_cost_ = 0.0;
    for (int i=0; i<num_x_; ++i) {
        for (int j=0; j<num_y_; ++j) {
            if (inf_cost_ < cost_matrix[i][j])
                inf_cost_ = cost_matrix[i][j];
        }
    }
    inf_cost_ += 1;

    x2y_ = new int[num_x_];
    y2x_ = new int[num_y_];
    memset(x2y_,-1,sizeof(int)*num_x_);
    memset(y2x_,-1,sizeof(int)*num_y_);
    
    wx_ = new double[num_x_];
    wy_ = new double[num_y_];
    memset(wx_,0.0,sizeof(double)*num_x_);
    memset(wy_,0.0,sizeof(double)*num_y_);

    for (int i=0; i<num_x_; ++i) {
        for (int j=0; j<num_y_; ++j) {
            if (wx_[i] < cost_matrix[i][j]) wx_[i] = cost_matrix[i][j];
        }
    }

    visited_x_ = new bool[num_x_];
    visited_y_ = new bool[num_y_];
    for (int i=0; i<num_x_; ++i) {
        while (true) {
            memset(visited_x_,false,sizeof(bool)*num_x_);
            memset(visited_y_,false,sizeof(bool)*num_y_);
            min_d_ = inf_cost_;
            if (FindExtendedPath(i)) break;

//                std::cout<<"min_d_ : "<<min_d_<<std::endl;
            for (int j=0; j<num_x_; ++j) {
//                    std::cout<<"x"<<j<<" cost : "<<wx_[j];
                if (visited_x_[j]) wx_[j] -= min_d_;
//                    std::cout<<" -> "<<wx_[j]<<std::endl;
            }
            for (int j=0; j<num_y_; ++j) {
//                    std::cout<<"y"<<j<<" cost : "<<wy_[j];
                if (visited_y_[j]) wy_[j] += min_d_;
//                    std::cout<<" -> "<<wy_[j]<<std::endl;
            }
        }
    }

    std::vector<std::pair<int,int>> couples;
    couples.clear();
//        std::cout<<"dispatch result : \n"<<std::endl;
    int result = 0;
    for (int i=0; i<num_x_; ++i) {
        std::cout<<i<<"->"<<x2y_[i]<<std::endl;
        couples.emplace_back(i,x2y_[i]);

        result += cost_matrix[i][x2y_[i]];
        std::cout << i << " -> " << x2y_[i] << " cost " << cost_matrix[i][x2y_[i]] << std::endl;
    }
    std::cout << result << std::endl;
    // delete []visited_x_;
    // delete []visited_y_;
    // delete []x2y_;
    // delete []y2x_;
    // delete []wx_;
    // delete []wy_;
    // SetPointerToNULL();
}


int main()
{
    // while (true)
    // {
        num_x_ = 4;
        num_y_ = 4;
        w[0][0] = 2;  w[0][1] = 5;   w[0][2] = 1;  w[0][3] = 2; 
        w[1][0] = 8;   w[1][1] = 3;   w[1][2] = 9;  w[1][3] = 3;  
        w[2][0] = 4;  w[2][1] = 9;  w[2][2] = 2; w[2][3] = 4; 
        w[3][0] = 3;   w[3][1] = 5;  w[3][2] = 6; w[3][3] = 5; 
        GetMatch();
    //     printf("%d\n", KM());
    //     break;
    // }
    return 0;
}
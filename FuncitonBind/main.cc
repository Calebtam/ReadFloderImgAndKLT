//
// Created by zyn on 2023/3/8.
//

#include <functional>
#include <iostream>
#include <unistd.h>
#include <signal.h>
using namespace std;
void fun(int x, int y, int z){
  cout << "x = " << x << ", y = " << y << ", z = " << z << endl;
}

void signal_callback_handler(int signum) {
  cout << "Caught signal " << signum << endl;
  //终止程序
  exit(signum);   /// not adding this code, can not terminate this program
}

int main (){
  std::function<void(int,int,int)> bf; ///  = std::bind(fun, placeholders::_1, 20, placeholders::_2);
  //注册信号和信号处理程序
  signal(SIGINT, signal_callback_handler);
  if(bf)
    cout << "bind success !!" << std::endl;
  while(true)
  {
    sleep(10);
  }
  return 0;
}



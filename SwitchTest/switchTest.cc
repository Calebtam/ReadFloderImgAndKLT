//
// Created by zyn on 2022/11/15.
//
#include <iostream>
using namespace std;
int main(){
  int a = 1;
  switch (a)
  {
  case 1:
  case 2:
    cout << 2 << endl;
    break;
  case 3: cout << 3 << endl;
  default:
    cout << "exit !" << endl;
  }
  return 0;
}
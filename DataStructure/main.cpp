//
// Created by zyn on 2023/6/21.
//

#include <deque>
#include <iostream>
using namespace std;
int main(){
  deque<int> a;
  a.push_back(1);
  a.push_back(2);
  a.push_back(3);
  cout << "## end element: " << a[a.size()-1] << endl;
  a.pop_front();
  a.push_back(4);
  cout << "## end element: " << a[a.size()-1] << endl;
  cout << "## start element: " << a[0] << endl;
  return 1;
}
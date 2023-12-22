//
// Created by zyn on 2023/5/26.
//

#include <iostream>
using namespace std;
int main()
{
  double timestamp_inI = 649272.8;
  uint64_t timestamp = timestamp_inI*10e9;
  cout << "timestamp: " << timestamp << "," << timestamp_inI*10e9 << endl;
  timestamp = 166789787989;
  double imgTimeStamp = timestamp/1e6;
  uint64_t reTime = imgTimeStamp * 1e6;
  cout << "timestamp: " << reTime << endl;
  return 0;
}
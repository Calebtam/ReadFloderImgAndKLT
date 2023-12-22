//
// Created by zyn on 2023/3/8.
//

#include<iostream>

#include <rbs.h>

#include "type.h"
using namespace std;

bool ok;
/// in order to capture the "ctr + c" quit command
void SignalHandler(int signo, siginfo_t *info, void *ctext)
{
  ok = rbs::SignalHandler(signo, info);
}

int main(){

  ok = true;
  rbs::SignalHandlerInit(SignalHandler);
  rbs::ProcMsgRecv<ResetMessage> reset_recv("VslamReset",2);
  ResetMessage receiveMsg;
  receiveMsg.m_resetFlag = false;
  while( ok ){
    if(reset_recv.Get(receiveMsg,1000)){
      std::cout << "Receive Success: " << receiveMsg.m_resetFlag << std::endl;
      receiveMsg.m_resetFlag = false;
    }
    usleep(100);
  }

  return 1;
}
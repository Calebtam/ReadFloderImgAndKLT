//
// Created by zyn on 2023/3/8.
//

#include<iostream>

#include <rbs.h>

#include "type.h"

using namespace std;
bool ok;

void SignalHandler(int signo, siginfo_t *info, void *ctext)
{
  ok = rbs::SignalHandler(signo, info);
}

int main(){
  ok = true;

  rbs::SignalHandlerInit(SignalHandler);
  rbs::ProcMsgSend<ResetMessage> reset_send("VslamReset");
  ResetMessage m_resetMessage;
  m_resetMessage.m_resetFlag = true;
  while( ok ){
    reset_send.Pub(m_resetMessage);
    std::cout << "Sending !!" << std::endl;
    if(m_resetMessage.m_resetFlag)
      m_resetMessage.m_resetFlag = false;
    else
      m_resetMessage.m_resetFlag = true;
    usleep(100);
    break;
  }
  return 1;
}
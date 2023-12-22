//
// Created by zyn on 2023/3/8.
//

#ifndef TEST_TYPE_H
#define TEST_TYPE_H
#include <unistd.h>
struct ResetMessage {
  bool m_resetFlag = false;
  static const size_t MaxSize()
  {
    return sizeof(ResetMessage);
  }
};
#endif // TEST_TYPE_H

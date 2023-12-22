//
// Created by zyn on 2023/3/8.
//

#include <stdlib.h>

int main(){

//  system("gnome-terminal -- bash "
//         "-c 'curl -H \"content-Type: application/json\" -H \"Content-Length: 0\" "
//         "-X POST http://localhost:8080/pause'");
//
  system("gnome-terminal -- bash "
         "-c 'curl -H \"content-Type: application/json\" -H \"Content-Length: 0\" "
         "-X POST http://10.10.8.198:8080/pause'");
//  system("curl -s -v -- \"https://www.baidu.com\"");
  system("curl -H \"content-Type: application/json\" -H \"Content-Length: 0\" -X POST http://10.10.8.198:8080/pause");
return 1;
}
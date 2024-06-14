
#!/bin/bash

# 默认值
down_ip="10.8.190.85"
down_user="root"
# down_path="/userdata/mins/build_pitti_divergence/lib/"
down_path="/userdata/mins/build_pitti_divergence/thirdparty/rosbag/"
pass1='123'

remote_ip="192.168.3.5"
remote_user="root"
remote_path="/root/fescue_ws/install/loc_fusion_alg/lib/"
pass2='123'

# file_to_transfer="libob_mower_loc.so"
file_to_transfer="libRosbagStorage.so"
ssh_key="123"

# 解析参数
while getopts ":i:u:p:f:k:" opt; do
  case $opt in
    i) remote_ip="$OPTARG"
    ;;
    u) remote_user="$OPTARG"
    ;;
    p) remote_path="$OPTARG"
    ;;
    f) file_to_transfer="$OPTARG"
    ;;
    k) ssh_key="$OPTARG"
    ;;
    \?) echo "无效的选项: -$OPTARG" >&2
    ;;
  esac
done

# 检查是否提供了必需的参数
if [[ -z "$down_ip" || -z "$down_user" || -z "$down_path" || -z "$pass1" ]]; then
#   echo "使用方法: $0 -i 目标IP -u 用户名 -p 目标路径 -f 文件路径 -k SSH密钥路径"
  exit 1
fi

if [[ -z "$remote_ip" || -z "$remote_user" || -z "$remote_path" || -z "$file_to_transfer" || -z "$ssh_key" ]]; then
  echo "使用方法: $0 -i 目标IP -u 用户名 -p 目标路径 -f 文件路径 -k SSH密钥路径"
  exit 1
fi

# # 使用 scp 命令传输文件到目标主机，同时使用 SSH 密钥认证
# scp -i "$ssh_key" "$file_to_transfer" "$remote_user@$remote_ip:$remote_path$file_to_transfer"

# # 使用 scp 命令传输文件到目标主机，同时使用 SSH 密钥认证
# scp -i "$ssh_key" "$file_to_transfer" "$remote_user@$remote_ip:$remote_path"

# # 打印信息提示
# echo "文件上传完成"

echo "================================="
# down_path="/userdata/mins/build_pitti_divergence/thirdparty/rosbag/"
# file_to_transfer="libRosbagStorage.so"


# expect << END
#     set timeout 20
#     spawn scp $down_user@$down_ip:$down_path$file_to_transfer ./$file_to_transfer
#     expect {
#         "yes/no" {send "yes\r";exp_continue}
#         "*assword" {send "$pass1\n"}
#     }
#     expect eof
# END

# sleep $[ 1 ]
# echo "等待后继续"

# #  spawn scp ./lib/libob_aruco_api.so root@192.168.3.22:/root/fescue_ws/install/perception_alg/lib/libob_aruco_api.so
# #  spawn scp ./lib/libob_aruco_api.so root@192.168.10.2:/root/fescue_ws/install/perception_alg/lib/libob_aruco_api.so

# expect << END
#      set timeout 20

#     spawn scp ./$file_to_transfer $remote_user@$remote_ip:$remote_path$file_to_transfer

#      expect {
#          "yes/no" {send "yes\r";exp_continue}
#          "*assword" {send "$pass2\n"}
#      }
#      expect eof
# END
echo "================================="
# down_path="/userdata/mins/build_pitti_divergence/thirdparty/rosbag/"
# file_to_transfer="libRosTime.so"

# expect << END
#     set timeout 20
#     spawn scp $down_user@$down_ip:$down_path$file_to_transfer ./$file_to_transfer
#     expect {
#         "yes/no" {send "yes\r";exp_continue}
#         "*assword" {send "$pass1\n"}
#     }
#     expect eof
# END

# sleep $[ 1 ]
# echo "等待后继续"

# #  spawn scp ./lib/libob_aruco_api.so root@192.168.3.22:/root/fescue_ws/install/perception_alg/lib/libob_aruco_api.so
# #  spawn scp ./lib/libob_aruco_api.so root@192.168.10.2:/root/fescue_ws/install/perception_alg/lib/libob_aruco_api.so

# expect << END
#      set timeout 20

#     spawn scp ./$file_to_transfer $remote_user@$remote_ip:$remote_path$file_to_transfer

#      expect {
#          "yes/no" {send "yes\r";exp_continue}
#          "*assword" {send "$pass2\n"}
#      }
#      expect eof
# END

echo "================================="

down_path="/userdata/mins/build_pitti_divergence/lib/"
file_to_transfer="libob_mower_loc.so"

if [[ -z "$remote_ip" || -z "$remote_user" || -z "$remote_path" || -z "$file_to_transfer" || -z "$ssh_key" ]]; then
  echo "使用方法: $0 -i 目标IP -u 用户名 -p 目标路径 -f 文件路径 -k SSH密钥路径"
  exit 1
fi

expect << END
    set timeout 20
    spawn scp $down_user@$down_ip:$down_path$file_to_transfer ./$file_to_transfer
    expect {
        "yes/no" {send "yes\r";exp_continue}
        "*assword" {send "$pass1\n"}
    }
    expect eof
END

sleep $[ 1 ]
echo "等待后继续"

#  spawn scp ./lib/libob_aruco_api.so root@192.168.3.22:/root/fescue_ws/install/perception_alg/lib/libob_aruco_api.so
#  spawn scp ./lib/libob_aruco_api.so root@192.168.10.2:/root/fescue_ws/install/perception_alg/lib/libob_aruco_api.so

expect << END
     set timeout 20

    spawn scp ./$file_to_transfer $remote_user@$remote_ip:$remote_path$file_to_transfer

     expect {
         "yes/no" {send "yes\r";exp_continue}
         "*assword" {send "$pass2\n"}
     }
     expect eof
END


echo "===============Done=================="

#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import socket 
import time

def setGripperWidth(width, force, ip="192.168.83.2", port=30003):
    #定义了UR机器人的地址和端口
    target_ip = (ip, port)
    #建立一个socket对象
    sk = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #建立连接
    sk.connect(target_ip)
    #这是发送给UR机器人的脚本指令
    tcp_command = "def rg2ProgOpen():\n"
    tcp_command += "\ttextmsg(\"inside RG2 function called\")\n"

    tcp_command += '\ttarget_width={}\n'.format(width)
    tcp_command += "\ttarget_force={}\n".format(force)
    tcp_command += "\tpayload=1.0\n"
    tcp_command += "\tset_payload1=False\n"
    tcp_command += "\tdepth_compensation=False\n"
    tcp_command += "\tslave=False\n"

    tcp_command += "\ttimeout = 0\n"
    tcp_command += "\twhile get_digital_in(9) == False:\n"
    tcp_command += "\t\ttextmsg(\"inside while\")\n"
    tcp_command += "\t\tif timeout > 400:\n"
    tcp_command += "\t\t\tbreak\n"
    tcp_command += "\t\tend\n"
    tcp_command += "\t\ttimeout = timeout+1\n"
    tcp_command += "\t\tsync()\n"
    tcp_command += "\tend\n"
    tcp_command += "\ttextmsg(\"outside while\")\n"

    tcp_command += "\tdef bit(input):\n"
    tcp_command += "\t\tmsb=65536\n"
    tcp_command += "\t\tlocal i=0\n"
    tcp_command += "\t\tlocal output=0\n"
    tcp_command += "\t\twhile i<17:\n"
    tcp_command += "\t\t\tset_digital_out(8,True)\n"
    tcp_command += "\t\t\tif input>=msb:\n"
    tcp_command += "\t\t\t\tinput=input-msb\n"
    tcp_command += "\t\t\t\tset_digital_out(9,False)\n"
    tcp_command += "\t\t\telse:\n"
    tcp_command += "\t\t\t\tset_digital_out(9,True)\n"
    tcp_command += "\t\t\tend\n"
    tcp_command += "\t\t\tif get_digital_in(8):\n"
    tcp_command += "\t\t\t\tout=1\n"
    tcp_command += "\t\t\tend\n"
    tcp_command += "\t\t\tsync()\n"
    tcp_command += "\t\t\tset_digital_out(8,False)\n"
    tcp_command += "\t\t\tsync()\n"
    tcp_command += "\t\t\tinput=input*2\n"
    tcp_command += "\t\t\toutput=output*2\n"
    tcp_command += "\t\t\ti=i+1\n"
    tcp_command += "\t\tend\n"
    tcp_command += "\t\treturn output\n"
    tcp_command += "\tend\n"
    tcp_command += "\ttextmsg(\"outside bit definition\")\n"

    tcp_command += "\ttarget_width=target_width+0.0\n"
    tcp_command += "\tif target_force>40:\n"
    tcp_command += "\t\ttarget_force=40\n"
    tcp_command += "\tend\n"

    tcp_command += "\tif target_force<4:\n"
    tcp_command += "\t\ttarget_force=4\n"
    tcp_command += "\tend\n"
    tcp_command += "\tif target_width>110:\n"
    tcp_command += "\t\ttarget_width=110\n"
    tcp_command += "\tend\n"
    tcp_command += "\tif target_width<0:\n"
    tcp_command += "\t\ttarget_width=0\n"
    tcp_command += "\tend\n"
    tcp_command += "\trg_data=floor(target_width)*4\n"
    tcp_command += "\trg_data=rg_data+floor(target_force/2)*4*111\n"
    tcp_command += "\tif slave:\n"
    tcp_command += "\t\trg_data=rg_data+16384\n"
    tcp_command += "\tend\n"

    tcp_command += "\ttextmsg(\"about to call bit\")\n"
    tcp_command += "\tbit(rg_data)\n"
    tcp_command += "\ttextmsg(\"called bit\")\n"

    tcp_command += "\tif depth_compensation:\n"
    tcp_command += "\t\tfinger_length = 55.0/1000\n"
    tcp_command += "\t\tfinger_heigth_disp = 5.0/1000\n"
    tcp_command += "\t\tcenter_displacement = 7.5/1000\n"

    tcp_command += "\t\tstart_pose = get_forward_kin()\n"
    tcp_command += "\t\tset_analog_inputrange(2, 1)\n"
    tcp_command += "\t\tzscale = (get_analog_in(2)-0.026)/2.976\n"
    tcp_command += "\t\tzangle = zscale*1.57079633-0.087266462\n"
    tcp_command += "\t\tzwidth = 5+110*sin(zangle)\n"

    tcp_command += "\t\tstart_depth = cos(zangle)*finger_length\n"

    tcp_command += "\t\tsync()\n"
    tcp_command += "\t\tsync()\n"
    tcp_command += "\t\ttimeout = 0\n"

    tcp_command += "\t\twhile get_digital_in(9) == True:\n"
    tcp_command += "\t\t\ttimeout=timeout+1\n"
    tcp_command += "\t\t\tsync()\n"
    tcp_command += "\t\t\tif timeout > 20:\n"
    tcp_command += "\t\t\t\tbreak\n"
    tcp_command += "\t\t\tend\n"
    tcp_command += "\t\tend\n"
    tcp_command += "\t\ttimeout = 0\n"
    tcp_command += "\t\twhile get_digital_in(9) == False:\n"
    tcp_command += "\t\t\tzscale = (get_analog_in(2)-0.026)/2.976\n"
    tcp_command += "\t\t\tzangle = zscale*1.57079633-0.087266462\n"
    tcp_command += "\t\t\tzwidth = 5+110*sin(zangle)\n"
    tcp_command += "\t\t\tmeasure_depth = cos(zangle)*finger_length\n"
    tcp_command += "\t\t\tcompensation_depth = (measure_depth - start_depth)\n"
    tcp_command += "\t\t\ttarget_pose = pose_trans(start_pose,p[0,0,-compensation_depth,0,0,0])\n"
    tcp_command += "\t\t\tif timeout > 400:\n"
    tcp_command += "\t\t\t\tbreak\n"
    tcp_command += "\t\t\tend\n"
    tcp_command += "\t\t\ttimeout=timeout+1\n"
    tcp_command += "\t\t\tservoj(get_inverse_kin(target_pose),0,0,0.008,0.033,1700)\n"
    tcp_command += "\t\tend\n"
    tcp_command += "\t\tnspeed = norm(get_actual_tcp_speed())\n"
    tcp_command += "\t\twhile nspeed > 0.001:\n"
    tcp_command += "\t\t\tservoj(get_inverse_kin(target_pose),0,0,0.008,0.033,1700)\n"
    tcp_command += "\t\t\tnspeed = norm(get_actual_tcp_speed())\n"
    tcp_command += "\t\tend\n"
    tcp_command += "\tend\n"
    tcp_command += "\tif depth_compensation==False:\n"
    tcp_command += "\t\ttimeout = 0\n"
    tcp_command += "\t\twhile get_digital_in(9) == True:\n"
    tcp_command += "\t\t\ttimeout = timeout+1\n"
    tcp_command += "\t\t\tsync()\n"
    tcp_command += "\t\t\tif timeout > 20:\n"
    tcp_command += "\t\t\t\tbreak\n"
    tcp_command += "\t\t\tend\n"
    tcp_command += "\t\tend\n"
    tcp_command += "\t\ttimeout = 0\n"
    tcp_command += "\t\twhile get_digital_in(9) == False:\n"
    tcp_command += "\t\t\ttimeout = timeout+1\n"
    tcp_command += "\t\t\tsync()\n"
    tcp_command += "\t\t\tif timeout > 400:\n"
    tcp_command += "\t\t\t\tbreak\n"
    tcp_command += "\t\t\tend\n"
    tcp_command += "\t\tend\n"
    tcp_command += "\tend\n"
    tcp_command += "\tif set_payload1:\n"
    tcp_command += "\t\tif slave:\n"
    tcp_command += "\t\t\tif get_analog_in(3) < 2:\n"
    tcp_command += "\t\t\t\tzslam=0\n"
    tcp_command += "\t\t\telse:\n"
    tcp_command += "\t\t\t\tzslam=payload\n"
    tcp_command += "\t\t\tend\n"
    tcp_command += "\t\telse:\n"
    tcp_command += "\t\t\tif get_digital_in(8) == False:\n"
    tcp_command += "\t\t\t\tzmasm=0\n"
    tcp_command += "\t\t\telse:\n"
    tcp_command += "\t\t\t\tzmasm=payload\n"
    tcp_command += "\t\t\tend\n"
    tcp_command += "\t\tend\n"
    tcp_command += "\t\tzsysm=0.0\n"
    tcp_command += "\t\tzload=zmasm+zslam+zsysm\n"
    tcp_command += "\t\tset_payload(zload)\n"
    tcp_command += "\tend\n"

    tcp_command += "end\n"

    sk.send(str.encode(tcp_command))  # 利用字符串的encode方法编码成bytes，默认为utf-8类型
    sk.close()
    time.sleep(1)
    return
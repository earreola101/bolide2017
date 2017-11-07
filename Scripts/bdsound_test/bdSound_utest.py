import subprocess
import sys,string,os
import serial
import numpy as np

from timeit import default_timer
from time import sleep

#Script Version 0.1.0
#Created By Edward Arreola,earreola@logitech.com
#=======================================================================
#This is a simple script that can access the bdsound prototyp board.
#The script works on version 0.1 of the bdsound TABLE/TV HUB Ver.0.1
#Script can assemble a control packet and control the LED.
#Install python version 2.7
#Usage:
# Step#1 make sure the comport is configured correctly.
#=======================================================================

#configure below
comport="COM6"
timeout=100;
pbaudrate=115200
success=1

#bdsound board commands
CMD_SET_LED_MICPOD=0x0E01
CMD_SET_MUTE_MICPOD=0x0E02
CMD_INFO_MUTE_MICPOD=0x0E03
CMD_GET_LED_MICPOD=0x0E04
CMD_SET_POST_FILER_EN=0x0E05
CMD_SET_LOOPBACK_MICPOD=0x0E06
MICPOD_LED_ON=1
MICPOD_LED_OFF=0
ON=1
OFF=0

MAX_DATA_LEN=1024
HEADER_BYTES=2
HEADER_0=0x55
HEADER_1=0xAA
LEN_BYTES=2
CRC_BYTES=1
PACKET_OVERHEAD=(HEADER_BYTES+LEN_BYTES+CRC_BYTES)
COMMAND_OVERHEAD=10
MAX_PACKET_LEN=1024
MAX_MESSAGE_LEN=(MAX_PACKET_LEN-PACKET_OVERHEAD)
RX_QUEUE_LEN=(MAX_PACKET_LEN*4)

class tdm_command_t:
    command=0;
    sequence=0;
    sender=0;
    receiver=0;
    length=0;
    data=bytearray(MAX_DATA_LEN)

def crcCalc(pDst,packetLen):
    checksum = 0
    x=0
    for x in range(0,len(pDst)-1):
     checksum^=pDst[x]
     
    return checksum

#Set
def Set(commandstate,cmdSate):
    pcomport=comport
    ser = serial.Serial(port=pcomport,\
    baudrate=pbaudrate,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS);
    
    tdm_cmd=tdm_command_t();
    tdm_cmd.command=commandstate;
    tdm_cmd.sequence=0;
    tdm_cmd.sender=0x77;
    tdm_cmd.receiver=0x21;
    tdm_cmd.length=2;
    M_STATE=cmdSate;

    packetLen=tdm_cmd.length+PACKET_OVERHEAD+COMMAND_OVERHEAD;
    commandLen=tdm_cmd.length+COMMAND_OVERHEAD;
    cmdh = int((tdm_cmd.command >> 8) & 0xff);
    cmdl = int(tdm_cmd.command & 0xff);
    
    packet = bytearray(MAX_DATA_LEN);
    packet.append(HEADER_0);                  #(0x55)#
    packet.append(HEADER_1);                  #(0xaa)#
    packet.append((packetLen>>8)&0xFF);       #(0x00)#
    packet.append(packetLen&0xFF);            #(0x11)#
    packet.append(cmdl);                      #(0x01)#
    packet.append(cmdh);                      #(0x0e)#
    packet.append(tdm_cmd.sender);            #(0x77)#
    packet.append(tdm_cmd.receiver);          #(0x21)#
    packet.append(0x00);                      #(0x00)#
    packet.append(0x00);                      #(0x00)#
    packet.append(0x00);                      #(0x00)#
    packet.append(0x00);                      #(0x00)#
    packet.append(tdm_cmd.length);            #(0x02)#
    packet.append(0x00);                      #(0x00)#
    packet.append(M_STATE);                   #(0x00)#
    packet.append(0x00);                      #(0x00)#
    packet.append(crcCalc(packet,packetLen)); #(0xb5)#
    
    ser.write(packet);
    ser.close();
    del packet;
    return

#==================================LED_TEST
Set(CMD_SET_LED_MICPOD,MICPOD_LED_ON);
sleep(0.5)
Set(CMD_SET_LED_MICPOD,MICPOD_LED_OFF);
sleep(0.5)
Set(CMD_SET_LED_MICPOD,MICPOD_LED_ON);
sleep(0.5)
Set(CMD_SET_LED_MICPOD,MICPOD_LED_OFF);
sleep(0.5)
#==================================LED_TEST

print("done")

sys.exit(success)

# Protocol Description for deehli

This document will quickly summarize the protocol used when communicating from the computer to the board.

## Medium and data types

The transmission is done over serial/USB, commands are described as a string encoded in ascii.

The generic syntax is "{CMD_ID},{PARAM1},{PARAM2},..." 

## Commands

Here is a description of the supported commands:

| cmd id | cmd name | parameters | types |
| - | - | - | - |
| 1 | servo set             | id,servo_id,taget_angle(deg) | u8,u8,float |
| 2 | servo goto            | id,servo_id,taget_angle(deg),max_speed(degps) | u8,u8,float,float |
| 3 | servo group goto 1by1 | id,servo1_id,taget_angle1(deg),max_speed1(degps),servo2_id,target_angle2(deg),max_speed2(degps),... | u8,u8,float,float,u8,float,float,... |
| 4 | servo group goto sync | id,servo1_id,taget_angle1(deg),max_speed1(degps),servo2_id,target_angle2(deg),max_speed2(degps),... | u8,u8,float,float,u8,float,float,... |


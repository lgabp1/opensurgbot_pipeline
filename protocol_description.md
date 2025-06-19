# Protocol Description for deehli

This document will quickly summarize the protocol used when communicating from the computer to the board.

## Medium and data types

The transmission is done over serial/USB, commands are described as a string encoded in ascii.

The generic syntax is "{CMD_ID},{PARAM1},{PARAM2},..." 

## Commands

Here is a description of the supported commands:

**Servo**

| cmd id | cmd name | parameters | types |
| - | - | - | - |
| 1 | servo set             | cmdid,servo_id,taget_angle(deg) | u8,u8,float |
| 2 | servo goto            | cmdid,servo_id,taget_angle(deg),max_speed(degps) | u8,u8,float,float |
| 3 | servo group goto 1by1 | cmdid,servo1_id,taget_angle1(deg),max_speed1(degps),servo2_id,target_angle2(deg),max_speed2(degps),... | u8,u8,float,float,u8,float,float,... |
| 4 | servo group goto sync | cmdid,servo1_id,taget_angle1(deg),max_speed1(degps),servo2_id,target_angle2(deg),max_speed2(degps),... | u8,u8,float,float,u8,float,float,... |

**ZDT linear actuator**

| cmd id | cmd name | parameters | types |
| - | - | - | - |
| 10 | ZDT enable control | cmdid, do_enable                            | u8,bool |
| 11 | ZDT pos control    | cmdid,dir,npulses,speed,accellvl,isrelative | u8,bool,u32,u16,u8,bool |
| 18 | ZDT trigger homing | cmdid                                       | u8 |
| 19 | ZDT set homing 0   | cmdid                                       | u8 |

note: "bool" means 0:False, 1:True in the string message
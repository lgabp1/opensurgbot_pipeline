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
| 1 | servo set             | cmdid,servo_id,target_angle(deg) | u8,u8,float |
| 2 | servo goto            | cmdid,servo_id,target_angle(deg),max_speed(degps) | u8,u8,float,float |
| 3 | servo group goto 1by1 | cmdid,servo1_id,target_angle1(deg),max_speed1(degps),servo2_id,target_angle2(deg),max_speed2(degps),... | u8,u8,float,float,u8,float,float,... |
| 4 | servo group goto sync | cmdid,servo1_id,target_angle1(deg),max_speed1(degps),servo2_id,target_angle2(deg),max_speed2(degps),... | u8,u8,float,float,u8,float,float,... |

**ZDT linear actuator**

| cmd id | cmd name | parameters | types |
| - | - | - | - |
| 10 | ZDT enable control | cmdid, do_enable                            | u8,bool |
| 11 | ZDT pos control    | cmdid,dir,npulses,speed,accellvl,isrelative | u8,bool,u32,u16,u8,bool |
| 18 | ZDT trigger homing | cmdid                                       | u8 |
| 19 | ZDT set homing 0   | cmdid                                       | u8 |

**Other**
| cmd id | cmd name | parameters | types |
| 101 | deehli drive all blocking | cmdid,target_angle1(deg),target_angle2(deg),target_angle3(deg),target_angle4(deg),max_speeds(degps),zdt_dir,npulses,speed,accellvl,reached_timeout(ms) | u8,float,float,float,float,float,bool,u32,u16,u8,u32 |

note: "bool" means 0:False, 1:True in the string message

## Receptions from board
| msg_id | name | parameters | types |
| - | - | - | - |
| 255 | setup success | cmdid | u8 |
| 254 | setup error | cmdid | u8 |
| 250 | cmd ack | cmdid | u8 |
| 249 | cmd success | cmdid | u8 |
| 248 | cmd error   | cmdid,error_code | u8,u8 | 

Note: error_codes are 0:invalid_command 1:invalid_params 9:processing_error 10:timeout

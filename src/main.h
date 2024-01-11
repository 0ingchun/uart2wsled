# pragma once

#ifndef _MAIN_H_
#define _MAIN_H_

typedef struct {
    char deviceType[10];
    char packetType[10];
    bool peg_top;
    bool super_cap;
    bool auto_aim;
    bool aid_aim;
    bool dont_know;
} CommunicationData;



#endif
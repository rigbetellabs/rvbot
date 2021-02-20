#ifndef _ROS_rvbot_bringup_MotorSpeed_h
#define _ROS_rvbot_bringup_MotorSpeed_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rvbot_bringup
{

  class MotorSpeed : public ros::Msg
  {
    public:
      typedef float _lspeed_type;
      _lspeed_type lspeed;
      typedef float _rspeed_type;
      _rspeed_type rspeed;

    MotorSpeed():
      lspeed(0),
      rspeed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_lspeed;
      u_lspeed.real = this->lspeed;
      *(outbuffer + offset + 0) = (u_lspeed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lspeed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lspeed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lspeed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lspeed);
      union {
        float real;
        uint32_t base;
      } u_rspeed;
      u_rspeed.real = this->rspeed;
      *(outbuffer + offset + 0) = (u_rspeed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rspeed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rspeed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rspeed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rspeed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_lspeed;
      u_lspeed.base = 0;
      u_lspeed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_lspeed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_lspeed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_lspeed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->lspeed = u_lspeed.real;
      offset += sizeof(this->lspeed);
      union {
        float real;
        uint32_t base;
      } u_rspeed;
      u_rspeed.base = 0;
      u_rspeed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rspeed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rspeed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rspeed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rspeed = u_rspeed.real;
      offset += sizeof(this->rspeed);
     return offset;
    }

    const char * getType(){ return "rvbot_bringup/MotorSpeed"; };
    const char * getMD5(){ return "3be884aba89f83e32f3db8c63ed039bf"; };

  };

}
#endif

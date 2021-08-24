#include "Plane.h"

#define rc_pitch 2  
#define rc_roll 1  
#define rc_yaw  4
#define rc_throttle 3 //previously 7
//#define rc_flap  8 previously 6

uint16_t i_loop = 0;
uint16_t pitch, roll, yaw, flap, throttle;
float pitch_s, roll_s, yaw_s, flap_s, throttle_s, FL, RL, FR, RR, M1, M2;

float Plane::scale_pwm(uint16_t n)  {return ((float)(n-1500)/500);}
float Plane::scale_pwm_rc(uint16_t n)  {return ((float)(n-25-1500)/500);}

float Plane::mixer_limit(float n)
{
  if (n < -1.0) n = -1.0;
  else if (n > 1.0) n = 1.0;
  return n;
}

void Plane::Electro_loop()
{
    SRV_Channels::get_output_pwm(SRV_Channel::k_elevator, pitch);
    pitch_s = scale_pwm(pitch);
    
    SRV_Channels::get_output_pwm(SRV_Channel::k_aileron, roll);
    roll_s = scale_pwm(roll);

    SRV_Channels::get_output_pwm(SRV_Channel::k_rudder, yaw);
    yaw_s = scale_pwm(yaw);

    SRV_Channels::get_output_pwm(SRV_Channel::k_throttle, throttle);
    throttle_s = scale_pwm(throttle);
    
    //SRV_Channels::get_output_pwm(SRV_Channel::k_flap, flap);
    //flap_s = scale_pwm(flap);
    
    //rc().get_pwm(rc_pitch, pitch);
    //pitch_s = scale_pwm_rc(pitch);

    //rc().get_pwm(rc_roll, roll);
    //roll_s = scale_pwm_rc(roll);

    //rc().get_pwm(rc_yaw, yaw);
    //yaw_s = scale_pwm_rc(yaw);

    //rc().get_pwm(rc_throttle, throttle);
    //throttle_s = scale_pwm_rc(throttle);

    //rc().get_pwm(rc_flap, flap);
    //flap_s = scale_pwm_rc(flap);

    //pitch_s = 0;
    //roll_s = 0;
    //yaw_s = 0;
    //throttle_s = 0;
    flap_s = 0;

    FL = mixer_limit(flap_s-(1.0*pitch_s)-(1.0*roll_s)); //+(0.5*yaw_s))
    RL = mixer_limit(flap_s+(1.0*pitch_s)-(1.0*roll_s)); //-(0.5*yaw_s))
    FR = mixer_limit(flap_s-(1.0*pitch_s)+(1.0*roll_s)); //-(0.5*yaw_s))
    RR = mixer_limit(flap_s+(1.0*pitch_s)+(1.0*roll_s)); //+(0.5*yaw_s))
    
    SRV_Channels::set_output_norm(SRV_Channel::k_scripting2, FL);
    SRV_Channels::set_output_norm(SRV_Channel::k_scripting3, RL);
    SRV_Channels::set_output_norm(SRV_Channel::k_scripting5, FR);
    SRV_Channels::set_output_norm(SRV_Channel::k_scripting6, RR);

    if(arming.is_armed())
    {
      if (throttle_s > 0)
      {
        M1 = mixer_limit((0.75*throttle_s)+(0.25*yaw_s));
        M2 = mixer_limit((0.75*throttle_s)-(0.25*yaw_s));
      }
      else if (throttle_s < 0)
      {
        M1 = mixer_limit((0.5*throttle_s)+(0.25*yaw_s));
        M2 = mixer_limit((0.5*throttle_s)-(0.25*yaw_s));
      }
      SRV_Channels::set_output_norm(SRV_Channel::k_scripting1, M1);
      SRV_Channels::set_output_norm(SRV_Channel::k_scripting4, M2);
    }
    else
    {
      SRV_Channels::set_output_pwm(SRV_Channel::k_scripting1, 0);
      SRV_Channels::set_output_pwm(SRV_Channel::k_scripting4, 0);
    }

    if (i_loop % 512 == 0)
    {
      GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Pitch = %f, Roll = %f, Yaw = %f, Flap = %f, Throttle = %f", pitch_s, roll_s, yaw_s, flap_s, throttle_s);
      GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FL = %f, RL = %f, FR = %f, RR = %f", FL, RL, FR, RR);
    }

    i_loop++;
}

#ifndef RW_INTERFACE_H
#define RW_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

void RW_Command_SetTorqueNm(float torque_nm);
float RW_Command_GetTorqueNm(void);
void RW_Command_SetWheelSpeedRadS(float omega_rad_s);
float RW_Command_GetWheelSpeedRadS(void);

#ifdef __cplusplus
}
#endif

#endif

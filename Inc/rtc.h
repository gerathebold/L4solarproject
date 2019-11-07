/*
 * rtc.h
 *
 *  Created on: 7 nov. 2019
 *      Author: Gerardo
 */

#ifndef RTC_H_
#define RTC_H_

void Configure_RTC_Clock(void);
void RTC_wakeup_init_from_standby_or_shutdown( int delay );
void RTC_wakeup_init_from_stop( int delay );
#endif /* RTC_H_ */

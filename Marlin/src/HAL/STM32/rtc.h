
#pragma once

void rtc_init(int force);
void rtc_init_nohal(int force);

void rtc_set_time(uint8_t hour, uint8_t minute, uint8_t second);
void rtc_set_date(uint8_t year, uint8_t month, uint8_t day, uint8_t weekday);
void rtc_get_date_time(RTC_DateTypeDef* date, RTC_TimeTypeDef* time);
#if ENABLED(SDSUPPORT)
void rtc_dateTimeCallback(uint16_t *date, uint16_t *time);
#endif

void rtc_print_datetime();
uint32_t rtc_read_status_reg();

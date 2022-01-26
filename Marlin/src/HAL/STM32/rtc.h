
#pragma once

void rtc_init(int force);
void rtc_init2();

void rtc_set_time(uint8_t hour, uint8_t minute, uint8_t second);
void rtc_set_date(uint8_t year, uint8_t month, uint8_t day);
void rtc_print_datetime();

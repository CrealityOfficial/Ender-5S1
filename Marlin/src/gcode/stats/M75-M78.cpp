/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../gcode.h"
#include "../../feature/runout.h"
#include "../../module/printcounter.h"
#include "../../lcd/marlinui.h"
#if ENABLED(RTS_AVAILABLE)
  #include "../../lcd/dwin/lcd_rts.h"
  #include "../../lcd/dwin/preview.h"
  #include "../../module/temperature.h"
  #include "../../module/planner.h"
#endif

#include "../../MarlinCore.h" // for startOrResumeJob

/**
 * M75: Start print timer
 */
void GcodeSuite::M75() {
  startOrResumeJob();
}

/**
 * M76: Pause print timer
 */
void GcodeSuite::M76() {
  print_job_timer.pause();
}

/**
 * M77: Stop print timer
 */
void GcodeSuite::M77() {
  print_job_timer.stop();
}

#if ENABLED(PRINTCOUNTER)

/**
 * M78: Show print statistics
 */
void GcodeSuite::M78() {
  if (parser.intval('S') == 78) {  // "M78 S78" will reset the statistics
    print_job_timer.initStats();
    ui.reset_status();
    return;
  }

  #if HAS_SERVICE_INTERVALS
    if (parser.seenval('R')) {
      print_job_timer.resetServiceInterval(parser.value_int());
      ui.reset_status();
      return;
    }
  #endif

  print_job_timer.showStats();
}

#endif // PRINTCOUNTER
// 云打印状态标志
int32_t g_couldPrintState = COULD_PRINT_STATE_OFF;
/**
 * M79: cloud print statistics
 */
void GcodeSuite::M79()
{
  if (parser.seenval('S'))
  {
    const int16_t cloudvalue = parser.value_celsius();
    switch (cloudvalue)
    {
      case 0:
        // 0:cloud connect
        #if ENABLED(RTS_AVAILABLE)
          rtscheck.RTS_SndData(1, WIFI_CONNECTED_DISPLAY_ICON_VP);
        #endif
        break;

      case 1:
        // 1:cloud print satrt
        #if ENABLED(RTS_AVAILABLE)
          if(PoweroffContinue == false)
          {
            Update_Time_Value = 0;
            print_job_timer.start();
            rtscheck.RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
            change_page_font = 10;
            g_couldPrintState = COULD_PRINT_STATE_RUNNING;
            rtscheck.RTS_SndData(1, DEFAULT_PRINT_MODEL_VP);
          }
        #endif
        break;

      case 2:
        // 2:cloud print pause
        #if ENABLED(RTS_AVAILABLE)
          if(PoweroffContinue == false) // 云打印
          {
            Update_Time_Value = 0;
            print_job_timer.pause();
            if(runout.filament_ran_out == false)
            {
              rtscheck.RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
              change_page_font = 12;
            }
            g_couldPrintState = COULD_PRINT_STATE_PAUSE;
          } else if(PoweroffContinue == true) { // 本地打印
              if(!wait_for_heatup) { // 没有循环加热
                rtscheck.RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
                waitway = 1;
                pause_e = current_position[E_AXIS] - 3;
                if(!temphot) {
                  temphot = thermalManager.temp_hotend[0].target;
                }
                card.pauseSDPrint();
                print_job_timer.pause();
                pause_action_flag = true;
                Update_Time_Value = 0;
                planner.synchronize();
                sdcard_pause_check = false;
              } else { // 如果在循环加热就发继续打印
                 SERIAL_ECHOLN("M79 S3");
                break;
              }
          }
        #endif
        break;

      case 3:
        // 3:cloud print resume
        #if ENABLED(RTS_AVAILABLE)
          if(PoweroffContinue == false) { // 云打印
            Update_Time_Value = 0;
            print_job_timer.start();
            rtscheck.RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
            change_page_font = 10;
            if (g_cloudPLRStatusValue == CLOUD_PLR_CHOOSE_STATE) { // 存在断电续打
              g_cloudPLRStatusValue = CLOUD_PLR_YES_STATE;
            }
            g_couldPrintState = COULD_PRINT_STATE_RUNNING;
          } else if (PoweroffContinue == true) { // 本地打印
            if(CardReader::flag.mounted) {
              if(CLOUD_PLR_NULL_STATE == g_cloudPLRStatusValue) { // 无断电续打
                rtscheck.RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
                change_page_font = 10;
                #if ENABLED(HAS_RESUME_CONTINUE)
                  if(wait_for_user) {
                    wait_for_user = false;
                  }
                  else
                #endif
                  {
                    memset(commandbuf, 0, sizeof(commandbuf));
                    sprintf_P(commandbuf, PSTR("M109 S%i"), temphot);
                    queue.enqueue_one_now(commandbuf);

                    card.startOrResumeFilePrinting();
                    print_job_timer.start();
                    Update_Time_Value = 0;
                    sdcard_pause_check = true;
                  }
              } else if(g_cloudPLRStatusValue == CLOUD_PLR_CHOOSE_STATE) { // 有断电续打
                g_cloudPLRStatusValue = CLOUD_PLR_YES_STATE;
                PoweroffContinue = true;
                power_off_type_yes = 1;
                Update_Time_Value = 0;
                rtscheck.RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
                change_page_font = 10;
                // recovery.resume();
                queue.enqueue_now_P(PSTR("M1000"));
                sdcard_pause_check = true;
                rtscheck.RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
              }
            }
          }
        #endif
        break;

      case 4:
        // 4:cloud print stop
        #if ENABLED(RTS_AVAILABLE)
          if (PoweroffContinue == false) {
            Update_Time_Value = 0;
            rtscheck.RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
            change_page_font = 1;
            rtscheck.RTS_SndData(0, PRINT_PROCESS_VP);
            RefreshBrightnessAtPrint(0);
            rtscheck.RTS_SndData(0, PRINT_PROCESS_ICON_VP);
            print_job_timer.stop();
            g_cloudPLRStatusValue = CLOUD_PLR_NULL_STATE;
          } else if(PoweroffContinue == true) {
            if (27 == change_page_font) { // 有本地断电续打
              Update_Time_Value = RTS_UPDATE_VALUE;
              rtscheck.RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
              change_page_font = 1;
              rtscheck.RTS_SndData(0, PRINT_TIME_HOUR_VP);
              rtscheck.RTS_SndData(0, PRINT_TIME_MIN_VP);
              Update_Time_Value = 0;
              rtscheck.RTS_SDcard_Stop();
              g_cloudPLRStatusValue = CLOUD_PLR_NULL_STATE;
              break;
            }
            rtscheck.RTS_SndData(0, PRINT_TIME_HOUR_VP);
            rtscheck.RTS_SndData(0, PRINT_TIME_MIN_VP);
            Update_Time_Value = 0;
            temphot = 0;
            rtscheck.RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
            waitway = 7;
            rtscheck.RTS_SDcard_Stop();
            gcode.process_subcommands_now_P(PSTR("G28 XY"));
            gcode.process_subcommands_now_P(PSTR("M84"));
            g_cloudPLRStatusValue = CLOUD_PLR_NULL_STATE;
          }
        #endif
        break;

      case 5:
        // 5:cloud print complete
        #if ENABLED(RTS_AVAILABLE)
          if (PoweroffContinue == false) {
            Update_Time_Value = 0;
            print_job_timer.stop();
            rtscheck.RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
            change_page_font = 1;
            rtscheck.RTS_SndData(0, PRINT_PROCESS_VP);
            RefreshBrightnessAtPrint(0);
            rtscheck.RTS_SndData(0, PRINT_PROCESS_ICON_VP);
          }
        #endif
        break;

      case 6:
        // 6:cloud print power continue
        #if ENABLED(RTS_AVAILABLE)
          if (PoweroffContinue == false) {
            rtscheck.RTS_SndData(ExchangePageBase + 27, ExchangepageAddr);
            change_page_font = 27;
            power_off_type_yes = 1;
            g_cloudPLRStatusValue = CLOUD_PLR_CHOOSE_STATE;
          }
        #endif
        break;
      default:
        break;
    }
  }

  if (parser.seenval('T'))
  {
    const int16_t feedratevalue = parser.value_celsius();
    #if ENABLED(RTS_AVAILABLE)
      rtscheck.RTS_SndData(feedratevalue, PRINT_PROCESS_VP);
      RefreshBrightnessAtPrint(feedratevalue);
      rtscheck.RTS_SndData(feedratevalue, PRINT_PROCESS_ICON_VP);
    #endif
  }

  if (parser.seenval('C'))
  {
    millis_t print_time_update = parser.value_celsius();
    #if ENABLED(RTS_AVAILABLE)
      rtscheck.RTS_SndData(print_time_update / 3600, PRINT_TIME_HOUR_VP);
      rtscheck.RTS_SndData((print_time_update % 3600) / 60, PRINT_TIME_MIN_VP);
    #endif
  }

  if (parser.seenval('D'))
  {
    millis_t next_remain_time_update = parser.value_celsius();
    #if ENABLED(RTS_AVAILABLE)
      rtscheck.RTS_SndData(next_remain_time_update / 3600, PRINT_REMAIN_TIME_HOUR_VP);
      rtscheck.RTS_SndData((next_remain_time_update % 3600) / 60, PRINT_REMAIN_TIME_MIN_VP);
    #endif
  }
}

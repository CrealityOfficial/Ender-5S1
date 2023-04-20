#include "lcd_rts.h"
#include <wstring.h>
#include <stdio.h>
#include <string.h>
#include <Arduino.h>
//#include <libmaple/usart.h>
#include "../../MarlinCore.h"
#include "../../inc/MarlinConfig.h"
#include "../../core/serial.h"
#include "../../core/macros.h"

#include "../fontutils.h"
#include "../marlinui.h"
#include "../../sd/cardreader.h"
#include "../../feature/powerloss.h"
#include "../../feature/runout.h"
#include "../../feature/babystep.h"
#include "../../module/temperature.h"
#include "../../module/printcounter.h"
#include "../../module/motion.h"
#include "../../module/planner.h"
#include "../../gcode/queue.h"
#include "../../gcode/gcode.h"
#include "../../module/probe.h"
#include "preview.h"
#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  #include "../../feature/bedlevel/bedlevel.h"
  #include "../../feature/bedlevel/abl/abl.h"
#endif

#if ENABLED(EEPROM_SETTINGS)
  #include "../../HAL/shared/eeprom_api.h"
  #include "../../module/settings.h"
#endif

#if ENABLED(RTS_AVAILABLE)

#ifdef LCD_SERIAL_PORT
  #define LCDSERIAL LCD_SERIAL
#elif SERIAL_PORT_2
  #define LCDSERIAL MYSERIAL2
#endif

#if ENABLED(HAS_MENU_RESET_WIFI)
  unsigned char WIFI_STATE = INITIAL;
#endif

char errorway = 0;
char errornum = 0;
char home_errornum = 0;
char error_sd_num = 0;

#if ENABLED(BABYSTEPPING)
  float zprobe_zoffset;
  float last_zoffset = 0.0;
#endif

int power_off_type_yes = 0;

int8_t g_cloudPLRStatusValue = 0; // 云打印状态标志位 


const float manual_feedrate_mm_m[] = {50 * 60, 50 * 60, 4 * 60, 60};
constexpr float default_max_feedrate[]        = DEFAULT_MAX_FEEDRATE;
constexpr float default_max_acceleration[]    = DEFAULT_MAX_ACCELERATION;
constexpr float default_max_jerk[]            = { DEFAULT_XJERK, DEFAULT_YJERK, DEFAULT_ZJERK, DEFAULT_EJERK };
constexpr float default_axis_steps_per_unit[] = DEFAULT_AXIS_STEPS_PER_UNIT;

float default_nozzle_ptemp = DEFAULT_Kp;
float default_nozzle_itemp = DEFAULT_Ki;
float default_nozzle_dtemp = DEFAULT_Kd;

float default_hotbed_ptemp = DEFAULT_bedKp;
float default_hotbed_itemp = DEFAULT_bedKi;
float default_hotbed_dtemp = DEFAULT_bedKd;

int startprogress = 0;
CRec CardRecbuf;
int16_t temphot = 0;
int16_t tempbed = 0;
float temp_bed_display = 0;
uint8_t afterprobe_fan0_speed = 0;
float pause_e = 0;
bool sdcard_pause_check = true;
bool pause_action_flag = false;
bool print_preheat_check = false;
bool probe_offset_flag = false;

float ChangeFilamentTemp = 200;
int heatway = 0;
millis_t next_rts_update_ms      = 0;
millis_t next_shutdown_update_ms = 0;
millis_t next_wifireset_update_ms = 0;
unsigned int count_ms = 0;
unsigned int wifiresetcount_ms = 0;
unsigned long count_lcd_down = 0;
bool flag_lcd_down = false;

int last_target_temperature[4] = {0};
int last_target_temperature_bed;
char waitway = 0;
int change_page_font = 1;
unsigned char Percentrecord = 0;
// represents to update file list
bool CardUpdate = false;

uint8_t fileCnt = 0;
uint8_t file_current_page = 1;
uint8_t file_total_page = 1;
uint8_t page_total_file = 0;

extern CardReader card;

RTSSHOW rtscheck;

uint8_t lang = 2;
// represents SD-card status, true means SD is available, false means opposite.
bool lcd_sd_status;

char Checkfilenum = 0;
char cmdbuf[20] = {0};
float FilamentLOAD = 10;
float FilamentUnLOAD = 10;

// 1 for 10mm, 2 for 1mm, 3 for 0.1mm
unsigned char AxisUnitMode;
float axis_unit = 10.0;
bool LEDStatus = false;

int Update_Time_Value = 0;

bool PoweroffContinue = false;
char commandbuf[30];

uint16_t remain_time = 0;

bool home_flag = false;
bool G29_flag = false;
bool G29_finished=false;
float z_values_value[4][4]={0};
bool leveing_flag=false;
bool flag_counter_wifireset = false;
HMI_LCD_Flag_t HMI_lcd_flag{0};
static bool temp_remove_card_flag=false,temp_cutting_line_flag=false;/*,temp_wifi_print_flag=false*/; /*0 沒有中断， 1 断料暂停  2 拔卡暂停*/

BedNozzleHeightCalSt st_bedNozzleHeightCal={0};//平台和喷嘴高度测量结构体
float bedNozzleHeightCalZ=0;//平台喷嘴Z轴测量变量
bool g_heaterLoadTempAdd = false; // 进退料温度加热
bool g_uiXYAxisEnable = false; // 轴移动界面，YX轴电机使能标志位
bool g_uiZAxisEnable = false; // 轴移动界面，Z轴电机使能标志位
bool g_uiZOffsetHomeOkFlag = false;
bool g_uiAutoPIDFlag =false;  // 进入自动PID界面的标志位
int16_t g_autoPIDHeaterTempTarget = 300;
int16_t g_autoPIDHotBedTempTarget = 110;
bool g_uiAutoPIDRuningFlag = false;  // 进入自动PID界面的标志位
int8_t g_uiAutoPIDRuningDiff = 0; // 0：没有启动自动PID。1：喷嘴PID调节中；2：热床PID调节中
int16_t g_uiCurveDataCnt = 0;
/**
 * 设置两个坐标（z下限位和bltouch建立的坐标系）的差值
 * @Author Creality
 * @Time   2021-12-13
 */
void RTSSHOW::SetzCoordinateOffset(float _offset)
{
    rtscheck.zCoordinateOffset = _offset;
}

//拔卡监测
 static void Remove_card_window_check(void)
 {
    static bool lSDCardStatus = false;
    /* sd card inserted */
      if(!lSDCardStatus && IS_SD_INSERTED())  //有卡
      {
        lSDCardStatus = IS_SD_INSERTED();        
      }
      /* sd card removed */
      else if(lSDCardStatus && !IS_SD_INSERTED())  //拔卡
      {
          lSDCardStatus = IS_SD_INSERTED();
          /* remove SD card when printing */
          if(IS_SD_PRINTING()) //正在打印  
          {
              HMI_lcd_flag.remove_card_flag=true;
              temp_remove_card_flag=true;
         
              #if ENABLED(POWER_LOSS_RECOVERY)
                if (recovery.enabled) recovery.save(true, false);//rock_20211016
              #endif
                    // #if ENABLED(RTS_AVAILABLE)
                //  SERIAL_ECHO_MSG("ROCK_MOVE_CARD2222\r\n");
                rtscheck.RTS_SndData(ExchangePageBase + 47, ExchangepageAddr);
                change_page_font = 47;
              queue.inject_P(PSTR("M25"));//M25: Pause the SD print in progress.              
          }
      }
      /* refresh sd card status */
      else
      {
          lSDCardStatus = IS_SD_INSERTED();
      }
}

inline void RTS_line_to_current(AxisEnum axis)
{
  if (!planner.is_full())
  {
    planner.buffer_line(current_position, MMM_TO_MMS(manual_feedrate_mm_m[(int8_t)axis]), active_extruder);
  }
}




static void RTS_line_to_filelist()
{
  for(int j = 0;j < 4;j ++)
  {
    // clean filename Icon
    for(int i = 0;i < TEXTBYTELEN;i ++)
    {
      rtscheck.RTS_SndData(0, CardRecbuf.addr[j] + i);
    }
  }
  memset(&CardRecbuf, 0, sizeof(CardRecbuf));

  int num = 0;
  for(uint16_t i = (file_current_page - 1) * 4; i < (file_current_page * 4);i ++)
  {
    card.selectFileByIndex(fileCnt - 1 - i);
    char *pointFilename = card.longFilename;
    int filenamelen = strlen(card.longFilename);
    int j = 1;
    while((strncmp(&pointFilename[j], ".gcode", 6) && strncmp(&pointFilename[j], ".GCODE", 6)) && ((j ++) < filenamelen));

    if (j >= TEXTBYTELEN)
    {
      strncpy(&card.longFilename[TEXTBYTELEN - 3], "..", 2);
      card.longFilename[TEXTBYTELEN - 1] = '\0';
      j = TEXTBYTELEN - 1;
    }
    strncpy(CardRecbuf.Cardshowfilename[num], card.longFilename, j);

    strcpy(CardRecbuf.Cardfilename[num], card.filename);
    CardRecbuf.addr[num] = FILE1_TEXT_VP + (num * 20);
    rtscheck.RTS_SndData(CardRecbuf.Cardshowfilename[num], CardRecbuf.addr[num]);
    CardRecbuf.Filesum = (++num);
  }
  page_total_file = CardRecbuf.Filesum;
  CardRecbuf.Filesum = ((file_total_page - 1) * 4) + page_total_file;
}

RTSSHOW::RTSSHOW(void)
{
  recdat.head[0] = snddat.head[0] = FHONE;
  recdat.head[1] = snddat.head[1] = FHTWO;
  memset(databuf, 0, sizeof(databuf));
}

void RTSSHOW::RTS_SDCardInit(void)
{
  if(RTS_SD_Detected())//如果检测到有卡插入
  {
    card.mount();
  }  
  SERIAL_ECHOLNPAIR(" CardReader::flag.mounted=: ",CardReader::flag.mounted);
  if(CardReader::flag.mounted)  //挂载成功
  {
    fileCnt = card.get_num_Files();
    card.getWorkDirName();
    if(card.filename[0] != '/')
    {
      card.cdup();
    }
    HAL_watchdog_refresh(); //解决拔插卡重启的问题，Rock_20220915

    if(fileCnt > 4)
    {
      file_total_page = (fileCnt / 4) + 1;
      if(file_total_page>5)file_total_page=5;  //rock_20221412
    }
    else
    {
      file_total_page = 1;
    }
    RTS_SndData(file_total_page, PRINT_COUNT_PAGE_DATA_VP);
    file_current_page = 1;
    RTS_SndData(file_current_page, PRINT_CURRENT_PAGE_DATA_VP);
    RTS_line_to_filelist(); //rock_20220915
    CardRecbuf.selecFlag = false;
    //rock_20220707
    if(PoweroffContinue == true) /*|| print_job_timer.isRunning())*/
    {
      return;
    }
    else
    {
      for(int j = 0;j < 20;j ++)
      {
        // clean print file
        RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
      }
    }
    lcd_sd_status = IS_SD_INSERTED();
  }
  else
  {
    if(PoweroffContinue == true)
    {
      return;
    }
    else
    {
      // clean filename Icon
      for(int j = 0;j < MaxFileNumber;j ++)
      {
        // clean filename Icon
        for(int i = 0;i < TEXTBYTELEN;i ++)
        {
          RTS_SndData(0, CardRecbuf.addr[j] + i);
        }
      }
      memset(&CardRecbuf, 0, sizeof(CardRecbuf));
    }
  }
}

bool RTSSHOW::RTS_SD_Detected(void)
{
  static bool last;
  static bool state;
  static bool flag_stable;
  static uint32_t stable_point_time;

  bool tmp = IS_SD_INSERTED();

  if(tmp != last)
  {
    flag_stable = false;
  }
  else
  {
    if(!flag_stable)
    {
      flag_stable = true;
      stable_point_time = millis();
    }
  }

  if(flag_stable)
  {
    if((millis() - stable_point_time) > 30)
    {
      state = tmp;
    }
  }

  last = tmp;

  return state;
}

void RTSSHOW::RTS_SDCardUpate(void)
{
  const bool sd_status = RTS_SD_Detected();
  
  if (sd_status != lcd_sd_status)
  {
    if (sd_status)
    {
      //SD card power on
      RTS_SDCardInit();
    }
    else
    {
      
      if(PoweroffContinue)//((PoweroffContinue == true) || print_job_timer.isRunning())
      {
        return;
      }
      else
      {
        card.release();
        for(int i = 0;i < CardRecbuf.Filesum;i ++)
        {
          for(int j = 0;j < 20;j ++)
          {
            RTS_SndData(0, CardRecbuf.addr[i] + j);
          }
          RTS_SndData((unsigned long)0xFFFF, FilenameNature + (i + 1) * 16);
        }
        for(int j = 0;j < 20;j ++)
        {
          // clean screen.
          RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
          RTS_SndData(0, SELECT_FILE_TEXT_VP + j);
        }
        memset(&CardRecbuf, 0, sizeof(CardRecbuf));
        RTS_SndData(1, PRINT_COUNT_PAGE_DATA_VP);
        file_total_page = 1;
        RTS_SndData(1, PRINT_CURRENT_PAGE_DATA_VP);
        file_current_page = 1;
      }
    }
    lcd_sd_status = sd_status;
  }


  // represents to update file list
  if(CardUpdate && lcd_sd_status && RTS_SD_Detected())
  {
    RTS_line_to_filelist();
    for(uint16_t i = 0;i < 5;i ++)
    {
      delay(1);
      RTS_SndData((unsigned long)0xFFFF, FilenameNature + (i + 1) * 16);
    }
    HAL_watchdog_refresh();
    CardUpdate = false;
  }
}

void RTSSHOW::RTS_Init(void)
{
  AxisUnitMode = 1;
  lang = language_change_font;
  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
    bool zig = true;
    int8_t inStart, inStop, inInc, showcount;
    showcount = 0;
    settings.load();
    st_bedNozzleHeightCal.zCoordinateOffset=bedNozzleHeightCalZ;//caixiaoliang add 20210807
    for (int y = 0; y < GRID_MAX_POINTS_Y; y++)
    {
      // away from origin
      if (zig)
      {
        inStart = 0;
        inStop = GRID_MAX_POINTS_X;
        inInc = 1;
      }
      else
      {
        // towards origin
        inStart = GRID_MAX_POINTS_X - 1;
        inStop = -1;
        inInc = -1;
      }
      zig ^= true;
      for (int x = inStart; x != inStop; x += inInc)
      {
        RTS_SndData(z_values[x][y] * 1000, AUTO_BED_LEVEL_1POINT_VP + showcount * 2);
        showcount++;
      }
    }
    queue.enqueue_now_P(PSTR("M420 S1"));
  #endif
  last_zoffset = zprobe_zoffset = probe.offset.z;
  RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);

  for(int i = 0;i < 9;i ++)
  {
    RTS_SndData(0, LANGUAGE_CHINESE_TITLE_VP + i);
  }
  RTS_SndData(1, LANGUAGE_CHINESE_TITLE_VP + (language_change_font - 1));
  languagedisplayUpdate();
  delay(500);

  last_target_temperature[0] = thermalManager.temp_hotend[0].target;
  last_target_temperature_bed = thermalManager.temp_bed.target;
  feedrate_percentage = 100;
  RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);

  /***************turn off motor*****************/
  RTS_SndData(1, MOTOR_FREE_ICON_VP);

  /***************transmit temperature to screen*****************/
  RTS_SndData(0, HEAD_SET_TEMP_VP);
  RTS_SndData(0, BED_SET_TEMP_VP);
  RTS_SndData(thermalManager.temp_hotend[0].celsius, HEAD_CURRENT_TEMP_VP);
  delay(20);
      #if ENABLED(BED_TEMP_COMP)
          float bedTempDisp = thermalManager.temp_bed.celsius; 
          if (thermalManager.degTargetBed() > 65 && thermalManager.degTargetBed() <= 127)
          {
              if (bedTempDisp > 65 && bedTempDisp <= 86)
                  bedTempDisp -= (5 * bedTempDisp) / thermalManager.degTargetBed();
              else if (bedTempDisp > 86 && bedTempDisp <= 127)
                  bedTempDisp -= (7 * bedTempDisp) / thermalManager.degTargetBed();
          }
          RTS_SndData(bedTempDisp, BED_CURRENT_TEMP_VP);
          #else 
          RTS_SndData(thermalManager.temp_bed.celsius, BED_CURRENT_TEMP_VP);
          #endif
 
  delay(20);
  RTS_SndData(ui.material_preset[0].hotend_temp, PREHEAT_PLA_SET_NOZZLE_TEMP_VP);
  RTS_SndData(ui.material_preset[0].bed_temp, PREHEAT_PLA_SET_BED_TEMP_VP);
  RTS_SndData(ui.material_preset[1].hotend_temp, PREHEAT_ABS_SET_NOZZLE_TEMP_VP);
  RTS_SndData(ui.material_preset[1].bed_temp, PREHEAT_ABS_SET_BED_TEMP_VP);

  RTS_SndData(planner.settings.max_feedrate_mm_s[0], MAX_VELOCITY_XAXIS_DATA_VP);
  RTS_SndData(planner.settings.max_feedrate_mm_s[1], MAX_VELOCITY_YAXIS_DATA_VP);
  RTS_SndData(planner.settings.max_feedrate_mm_s[2], MAX_VELOCITY_ZAXIS_DATA_VP);
  RTS_SndData(planner.settings.max_feedrate_mm_s[3], MAX_VELOCITY_EAXIS_DATA_VP);

  RTS_SndData(planner.settings.max_acceleration_mm_per_s2[0], MAX_ACCEL_XAXIS_DATA_VP);
  RTS_SndData(planner.settings.max_acceleration_mm_per_s2[1], MAX_ACCEL_YAXIS_DATA_VP);
  RTS_SndData(planner.settings.max_acceleration_mm_per_s2[2], MAX_ACCEL_ZAXIS_DATA_VP);
  RTS_SndData(planner.settings.max_acceleration_mm_per_s2[3], MAX_ACCEL_EAXIS_DATA_VP);

  RTS_SndData(planner.max_jerk.x * 100, MAX_JERK_XAXIS_DATA_VP);
  RTS_SndData(planner.max_jerk.y * 100, MAX_JERK_YAXIS_DATA_VP);
  RTS_SndData(planner.max_jerk.z * 100, MAX_JERK_ZAXIS_DATA_VP);
  RTS_SndData(planner.max_jerk.e * 100, MAX_JERK_EAXIS_DATA_VP);

  RTS_SndData(planner.settings.axis_steps_per_mm[0] * 10, MAX_STEPSMM_XAXIS_DATA_VP);
  RTS_SndData(planner.settings.axis_steps_per_mm[1] * 10, MAX_STEPSMM_YAXIS_DATA_VP);
  RTS_SndData(planner.settings.axis_steps_per_mm[2] * 10, MAX_STEPSMM_ZAXIS_DATA_VP);
  RTS_SndData(planner.settings.axis_steps_per_mm[3] * 10, MAX_STEPSMM_EAXIS_DATA_VP);

  RTS_SndData(PID_PARAM(Kp, 0) * 100, NOZZLE_TEMP_P_DATA_VP);
  RTS_SndData(unscalePID_i(PID_PARAM(Ki, 0)) * 100, NOZZLE_TEMP_I_DATA_VP);
  RTS_SndData(unscalePID_d(PID_PARAM(Kd, 0)) * 100, NOZZLE_TEMP_D_DATA_VP);
  RTS_SndData(thermalManager.temp_bed.pid.Kp * 100, HOTBED_TEMP_P_DATA_VP);
  RTS_SndData(unscalePID_i(thermalManager.temp_bed.pid.Ki) * 100, HOTBED_TEMP_I_DATA_VP);
  RTS_SndData(unscalePID_d(thermalManager.temp_bed.pid.Kd) * 10, HOTBED_TEMP_D_DATA_VP);
  RTS_SndData(StartSoundSet, 0);
  RTS_SndData(1, DEFAULT_PRINT_MODEL_VP);
  RTS_SndData(0, DOWNLOAD_PREVIEW_VP);
  /***************transmit Fan speed to screen*****************/
  // turn off fans
  thermalManager.set_fan_speed(0, 0);
  RTS_SndData(1, PRINTER_FANOPEN_TITLE_VP);
  RTS_SndData(0, PRINTER_LEDOPEN_TITLE_VP);
  LEDStatus = false;
  delay(5);

  /*********transmit SD card filename to screen***************/
  RTS_SDCardInit();

  /***************transmit Printer information to screen*****************/
  RTS_SndData(MACHINE_TYPE, MACHINE_TYPE_ABOUT_TEXT_VP);
  RTS_SndData(FIRMWARE_VERSION, FIREWARE_VERSION_ABOUT_TEXT_VP);
  // RTS_SndData(SCREEN_VERSION, PRINTER_DISPLAY_VERSION_TEXT_VP);
  RTS_SndData(HARDWARE_VERSION, HARDWARE_VERSION_ABOUT_TEXT_VP);
  RTS_SndData(PRINT_SIZE, PRINTER_PRINTSIZE_TEXT_VP);
  gcodePicDispalyOnOff(DEFAULT_PRINT_MODEL_VP, true);

  delay(5);
  if(1 == lang)
  {
    RTS_SndData(CORP_WEBSITE_C, WEBSITE_ABOUT_TEXT_VP);
  }
  else
  {
    RTS_SndData(CORP_WEBSITE_E, WEBSITE_ABOUT_TEXT_VP);
  }

  if(wifi_enable_flag)
  {
    RTS_SndData(0, ADV_SETTING_WIFI_ICON_VP);
    RTS_SndData(1, WIFI_CONNECTED_DISPLAY_ICON_VP);
  }
  else
  {
    RTS_SndData(1, ADV_SETTING_WIFI_ICON_VP);
    RTS_SndData(0, WIFI_CONNECTED_DISPLAY_ICON_VP);
  }

  if(recovery.enabled)
  {
    RTS_SndData(0, POWERCONTINUE_CONTROL_ICON_VP);
  }
  else
  {
    RTS_SndData(1, POWERCONTINUE_CONTROL_ICON_VP);
  }

  if(runout.enabled)
  {
    RTS_SndData(0, FILAMENT_CONTROL_ICON_VP);
  }
  else
  {
    RTS_SndData(1, FILAMENT_CONTROL_ICON_VP);
  }

  if (g_soundSetOffOn == 2) {
    RTS_SndData(DC_SOUND_SET_OFF, DC_SOUND_SET_DDR);
    RTS_SndData(1, SOUND_SETTING_OFF_ON_VP);
  } else {
    RTS_SndData(DC_SOUND_SET_ON, DC_SOUND_SET_DDR);
    RTS_SndData(0, SOUND_SETTING_OFF_ON_VP);
  }
  /**************************some info init*******************************/
  RTS_SndData(0, PRINT_PROCESS_ICON_VP);
  RTS_SndData(1, PREHAEAT_NOZZLE_ICON_VP);
  RTS_SndData(1, PREHAEAT_HOTBED_ICON_VP);

  rtscheck.RTS_SndData(ExchangePageBase, ExchangepageAddr);
  change_page_font = 0;

  for(startprogress = 0; startprogress <= 80; startprogress++)
  {
    rtscheck.RTS_SndData(startprogress, START_PROCESS_ICON_VP);
    delay(100);
    HAL_watchdog_refresh();
  }
  delay(200);
  HAL_watchdog_refresh();

  Update_Time_Value = RTS_UPDATE_VALUE;
}

int RTSSHOW::RTS_RecData(void)
{
  static int recnum = 0;
  #define DWINBUF_MAX 256
  static bool recvflag = false;
  static uint8_t databuf[DWINBUF_MAX];

  // Parse data frame
  if((LCDSERIAL.available() > 0) && (recnum < (signed)sizeof(databuf)))
  {
    databuf[recnum++] = LCDSERIAL.read();

    // #define RECV_DEBUG
    #if defined(RECV_DEBUG)
      char buf[16];
      sprintf_P(buf, PSTR("%02x "), databuf[recnum - 1]);
      serialprintPGM(buf);
    #endif

    // Verify the frame head
    if((recnum == 1) && (databuf[0] != 0x5A))
    {
      recnum = 0;
    }
    else if((recnum == 2) && (databuf[1] != 0xA5))
    {
      // Verify the frame head
      recnum = 0;
    }
    else if((recnum == 4) && (databuf[3] != 0x83))
    {
      // Parse only the read command 0x83 and filter out the reply command 0x82
      recnum = 0;
    }
    else if((recnum >= 3) && (databuf[2] == (recnum - 3)))
    {
      // Complete resolution
      recvflag = true;

      #if defined(RECV_DEBUG)
        serialprintPGM("\n");
        SERIAL_ECHO_MSG("dwin rx ok");
      #endif
    }
    else if((recnum >= 3) && ((recnum - 3) > databuf[2]))
    {
      // The actual received data bytes were larger than the frame data bytes, parsing failed
      recnum = 0;
    }
  }

  if(!recvflag)
  {
    return -1;
  }
  else
  {
    recvflag = false;
  }

  // receive nothing
  if(recnum < 1)
  {
    return -1;
  }
  else if((recdat.head[0] == databuf[0]) && (recdat.head[1] == databuf[1]) && (recnum > 2))
  {
    recdat.len = databuf[2];
    recdat.command = databuf[3];
    // response for writing byte
    if((recdat.len == 0x03) && ((recdat.command == 0x82) || (recdat.command == 0x80)) && (databuf[4] == 0x4F) && (databuf[5] == 0x4B))
    {
      memset(databuf, 0, sizeof(databuf));
      recnum = 0;
      return -1;
    }
    else if(recdat.command == 0x83)
    {
      // response for reading the data from the variate
      recdat.addr = databuf[4];
      recdat.addr = (recdat.addr << 8) | databuf[5];
      recdat.bytelen = databuf[6];
      for(unsigned int i = 0; i < recdat.bytelen; i += 2)
      {
        recdat.data[i / 2] = databuf[7 + i];
        recdat.data[i / 2] = (recdat.data[i / 2] << 8) | databuf[8 + i];
      }
    }
    else if(recdat.command == 0x81)
    {
      // response for reading the page from the register
      recdat.addr = databuf[4];
      recdat.bytelen = databuf[5];
      for(unsigned int i = 0; i < recdat.bytelen; i ++)
      {
        recdat.data[i] = databuf[6 + i];
        // recdat.data[i] = (recdat.data[i] << 8 )| databuf[7 + i];
      }
    }
  }
  else
  {
    memset(databuf, 0, sizeof(databuf));
    recnum = 0;
    // receive the wrong data
    return -1;
  }
  memset(databuf, 0, sizeof(databuf));
  recnum = 0;
  return 2;
}

void RTSSHOW::RTS_SndData(void)
{
  if((snddat.head[0] == FHONE) && (snddat.head[1] == FHTWO) && (snddat.len >= 3))
  {
    databuf[0] = snddat.head[0];
    databuf[1] = snddat.head[1];
    databuf[2] = snddat.len;
    databuf[3] = snddat.command;

    // to write data to the register
    if(snddat.command == 0x80)
    {
      databuf[4] = snddat.addr;
      for(int i = 0;i <(snddat.len - 2);i ++)
      {
        databuf[5 + i] = snddat.data[i];
      }
    }
    else if((snddat.len == 3) && (snddat.command == 0x81))
    {
      // to read data from the register
      databuf[4] = snddat.addr;
      databuf[5] = snddat.bytelen;
    }
    else if(snddat.command == 0x82)
    {
      // to write data to the variate
      databuf[4] = snddat.addr >> 8;
      databuf[5] = snddat.addr & 0xFF;
      for(int i =0;i <(snddat.len - 3);i += 2)
      {
        databuf[6 + i] = snddat.data[i/2] >> 8;
        databuf[7 + i] = snddat.data[i/2] & 0xFF;
      }
    }
    else if((snddat.len == 4) && (snddat.command == 0x83))
    {
      // to read data from the variate
      databuf[4] = snddat.addr >> 8;
      databuf[5] = snddat.addr & 0xFF;
      databuf[6] = snddat.bytelen;
    }
    // usart_tx(MYSERIAL1.c_dev(), databuf, snddat.len + 3);
    // MYSERIAL1.flush();
    for(int i = 0;i < (snddat.len + 3); i ++)
    {
      LCDSERIAL.write(databuf[i]);
      delayMicroseconds(1);
    }

    memset(&snddat, 0, sizeof(snddat));
    memset(databuf, 0, sizeof(databuf));
    snddat.head[0] = FHONE;
    snddat.head[1] = FHTWO;
  }
}

void RTSSHOW::RTS_SndData(const String &s, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
{
  if(s.length() < 1)
  {
    return;
  }
  RTS_SndData(s.c_str(), addr, cmd);
}

void RTSSHOW::RTS_SndData(const char *str, unsigned long addr, unsigned char cmd/*= VarAddr_W*/)
{
  int len = strlen(str);
  if(len > 0)
  {
    databuf[0] = FHONE;
    databuf[1] = FHTWO;
    databuf[2] = 3 + len;
    databuf[3] = cmd;
    databuf[4] = addr >> 8;
    databuf[5] = addr & 0x00FF;
    for(int i = 0;i < len;i ++)
    {
      databuf[6 + i] = str[i];
    }

    for(int i = 0;i < (len + 6);i ++)
    {
      LCDSERIAL.write(databuf[i]);
      delayMicroseconds(1);
    }
    memset(databuf, 0, sizeof(databuf));
  }
}
void RTSSHOW::RTS_SendCurveData(uint8_t channel, uint16_t *vaule, uint8_t size)
{

    databuf[0] = FHONE;
    databuf[1] = FHTWO;
    databuf[2] = 9 + size * 2;
    databuf[3] = VarAddr_W;
    databuf[4] = 0x03;
    databuf[5] = 0x10;
    databuf[6] = 0x5A;
    databuf[7] = 0xA5;
    databuf[8] = 0x01;
    databuf[9] = 0x00;
    databuf[10] = channel;
    databuf[11] = size;

    for (int i = 0,j = 0; j < size; j++) {
      databuf[i + 12] = vaule[j] >> 8;
      i++;
      databuf[i +12] = vaule[j] & 0x00FF;
      i++;
      if (i >= SizeofDatabuf)break;
    }
    for(int i = 0;i < (size * 2 + 12);i ++)
    {
      LCDSERIAL.write(databuf[i]);
      delayMicroseconds(1);
    }
    memset(databuf, 0, sizeof(databuf));
}
void RTSSHOW::RTS_SndData(char c, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
{
  snddat.command = cmd;
  snddat.addr = addr;
  snddat.data[0] = (unsigned long)c;
  snddat.data[0] = snddat.data[0] << 8;
  snddat.len = 5;
  RTS_SndData();
}

void RTSSHOW::RTS_SndData(unsigned char *str, unsigned long addr, unsigned char cmd) { RTS_SndData((char *)str, addr, cmd); }

void RTSSHOW::RTS_SndData(int n, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
{
  if (cmd == VarAddr_W)
  {
    if (n > 0xFFFF)
    {
      snddat.data[0] = n >> 16;
      snddat.data[1] = n & 0xFFFF;
      snddat.len = 7;
    }
    else
    {
      snddat.data[0] = n;
      snddat.len = 5;
    }
  }
  else if (cmd == RegAddr_W)
  {
    snddat.data[0] = n;
    snddat.len = 3;
  }
  else if (cmd == VarAddr_R)
  {
    snddat.bytelen = n;
    snddat.len = 4;
  }
  snddat.command = cmd;
  snddat.addr = addr;
  RTS_SndData();
}

void RTSSHOW::RTS_SndData(unsigned int n, unsigned long addr, unsigned char cmd) { RTS_SndData((int)n, addr, cmd); }

void RTSSHOW::RTS_SndData(float n, unsigned long addr, unsigned char cmd) { RTS_SndData((int)n, addr, cmd); }

void RTSSHOW::RTS_SndData(long n, unsigned long addr, unsigned char cmd) { RTS_SndData((unsigned long)n, addr, cmd); }

void RTSSHOW::RTS_SndData(unsigned long n, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
{
  if (cmd == VarAddr_W)
  {
    if (n > 0xFFFF)
    {
      snddat.data[0] = n >> 16;
      snddat.data[1] = n & 0xFFFF;
      snddat.len = 7;
    }
    else
    {
      snddat.data[0] = n;
      snddat.len = 5;
    }
  }
  else if (cmd == VarAddr_R)
  {
    snddat.bytelen = n;
    snddat.len = 4;
  }
  snddat.command = cmd;
  snddat.addr = addr;
  RTS_SndData();
}

void RTSSHOW::RTS_SDcard_Stop(void)
{
  if(PoweroffContinue == true)
  {
    planner.synchronize();
    // card.endFilePrint();
    card.flag.abort_sd_printing = true;
    queue.clear();
    quickstop_stepper();
    print_job_timer.stop();
    #if DISABLED(SD_ABORT_NO_COOLDOWN)
      thermalManager.disable_all_heaters();
    #endif
    print_job_timer.reset();
    wait_for_heatup = wait_for_user = false;
    PoweroffContinue = false;
    sd_printing_autopause = false;
    if(CardReader::flag.mounted)
    {
      #if ENABLED(SDSUPPORT) && ENABLED(POWER_LOSS_RECOVERY)
        card.removeJobRecoveryFile();
      #endif
    }
    CardRecbuf.recordcount = -1;
  }
  SERIAL_ECHOLN("M79 S4");   // 4:cloud print stop

  // shut down the stepper motor.
  // queue.enqueue_now_P(PSTR("M84"));
  RTS_SndData(1, MOTOR_FREE_ICON_VP);

  RTS_SndData(0, PRINT_PROCESS_ICON_VP);
  RTS_SndData(0, PRINT_PROCESS_VP);
  RefreshBrightnessAtPrint(0);
  delay(2);
  for(int j = 0;j < 20;j ++)
  {
    // clean screen.
    RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
    // clean filename
    RTS_SndData(0, SELECT_FILE_TEXT_VP + j);
  }
  // waitway = 0;
  // RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
  // change_page_font = 1;
}

void RTSSHOW::RTS_HandleData(void)
{
  int Checkkey = -1;
  // for waiting
  if(waitway > 0)
  {
    memset(&recdat, 0, sizeof(recdat));
    recdat.head[0] = FHONE;
    recdat.head[1] = FHTWO;
    return;
  }
  for(int i = 0;Addrbuf[i] != 0;i ++)
  {
    if(recdat.addr == Addrbuf[i])
    {
      Checkkey = i;
      break;
    }
  }

  if(Checkkey < 0)
  {
    memset(&recdat, 0, sizeof(recdat));
    recdat.head[0] = FHONE;
    recdat.head[1] = FHTWO;
    return;
  }

  switch(Checkkey)
  {
    case MainEnterKey:
      if(recdat.data[0] == 1)
      {
        CardUpdate = true;
        CardRecbuf.recordcount = -1;
        if (true == g_heaterLoadTempAdd) break;
        for (int j = 0; j < 20; j ++)
        {
          // clean screen.
          RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
          // clean filename
          RTS_SndData(0, SELECT_FILE_TEXT_VP + j);
        }
        for(int j = 0; j < 20; j ++)
        {
          RTS_SndData(0, FILE1_SELECT_ICON_VP + j);
        }
        RTS_SDCardUpate();

        RTS_SndData(file_total_page, PRINT_COUNT_PAGE_DATA_VP);
        file_current_page = 1;
        RTS_SndData(file_current_page, PRINT_CURRENT_PAGE_DATA_VP);

        RTS_SndData(ExchangePageBase + 2, ExchangepageAddr);
        change_page_font = 2;
        // if(CardReader::flag.mounted)  //IS_SD_INSERTED()
        if(IS_SD_INSERTED())
        {
          RTS_line_to_filelist();
        }
      }
      else if(recdat.data[0] == 2)
      {
        AxisUnitMode = 3;
        axis_unit = 0.1;
        RTS_SndData(10 * current_position[X_AXIS], AXIS_X_COORD_VP);
        RTS_SndData(10 * current_position[Y_AXIS], AXIS_Y_COORD_VP);
        RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);
        RTS_SndData(ExchangePageBase + 16, ExchangepageAddr);
        change_page_font = 16;
      }
      else if(recdat.data[0] == 3)
      {
        if (true == g_heaterLoadTempAdd) break;
        RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
        change_page_font = 21;
      }
      else if(recdat.data[0] == 4)
      {
        leveing_flag=true; //进入调平页面
        gcode.process_subcommands_now_P(PSTR("M420 S0"));//Rock_20220817 进入调平页面先关掉自动补偿

        RTS_SndData(ExchangePageBase + 25, ExchangepageAddr);
        change_page_font = 25;
        planner.synchronize();
        queue.enqueue_now_P(PSTR("G28"));
        queue.enqueue_now_P(PSTR("M420 S0"));//Rock_20220817 进入调平页面先关掉自动补偿          
        queue.enqueue_now_P(PSTR("G1 X110 Y110 F3000"));
        queue.enqueue_now_P(PSTR("G1 F600 Z0"));
        g_uiZOffsetHomeOkFlag = true;        
      
      }
      else if(recdat.data[0] == 5)
      {
        queue.clear();
        quickstop_stepper();
        print_job_timer.stop();
        RTS_SndData(1, MOTOR_FREE_ICON_VP);
        RTS_SndData(0, PRINT_PROCESS_ICON_VP);
        RTS_SndData(0, PRINT_PROCESS_VP);
        RefreshBrightnessAtPrint(0);

        RTS_SndData(0, PRINT_TIME_HOUR_VP);
        RTS_SndData(0, PRINT_TIME_MIN_VP);
        print_job_timer.reset();
        sd_printing_autopause = false;
        for(int j = 0;j < 20;j ++)
        {
          // clean screen.
          RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
          // clean filename
          RTS_SndData(0, SELECT_FILE_TEXT_VP + j);
        }
        CardRecbuf.recordcount = -1;  //不选中
        RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);//rock_20220708
        change_page_font = 1;
        gcodePicDispalyOnOff(DEFAULT_PRINT_MODEL_VP, true);
      }
      else if(recdat.data[0] == 6)
      {
        #if ENABLED(BLTOUCH)
          waitway = 3;
          RTS_SndData(lang, BEDLEVELING_WAIT_TITLE_VP);
          RTS_SndData(ExchangePageBase + 26, ExchangepageAddr);
          change_page_font = 26;

          #if ENABLED(PROBING_HEATERS_OFF)
            if(thermalManager.temp_hotend[0].target > 0)
            {
              temphot = thermalManager.temp_hotend[0].target;
              thermalManager.setTargetHotend(0, 0);
              rtscheck.RTS_SndData(0, HEAD_SET_TEMP_VP);
            }
          #endif

          #if ENABLED(PROBING_FANS_OFF)
            afterprobe_fan0_speed = thermalManager.fan_speed[0];
            if(thermalManager.fan_speed[0] > 0)
            {
              thermalManager.set_fan_speed(0, 0);
              rtscheck.RTS_SndData(0, PRINTER_FANOPEN_TITLE_VP);
            }
          #endif
  
          queue.enqueue_now_P(PSTR("G28\nG29"));
          RTS_SndData(0, MOTOR_FREE_ICON_VP);
        #endif
      }
      else if(recdat.data[0] == 7)
      {
        if(errorway == 1)
        {

        }
        else if(errorway == 2)
        {
          // auto home fail
        }
        else if(errorway == 3)
        {
          // bed leveling fail
        }
        else if(errorway == 4)
        {

        }
      }
      else if(recdat.data[0] == 8)
      {
        if (true == g_heaterLoadTempAdd) break;
        RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
        if (false == CardRecbuf.selecFlag) {
          gcodePicDispalyOnOff(DEFAULT_PRINT_MODEL_VP, true);
        }
        change_page_font = 1;
      }
      break;

    case AdjustEnterKey:
      if(recdat.data[0] == 1)
      {
        thermalManager.fan_speed[0] ? RTS_SndData(0, PRINTER_FANOPEN_TITLE_VP) : RTS_SndData(1, PRINTER_FANOPEN_TITLE_VP);

        RTS_SndData(ExchangePageBase + 14, ExchangepageAddr);
        change_page_font = 14;
      }
      else if(recdat.data[0] == 2)
      {
        if(PoweroffContinue == true)
        {
          if(card.isPrinting() || (wait_for_heatup == true))
          {
            RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
            change_page_font = 10;
          }
          else
          {
            RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
            change_page_font = 12;
          }
        }
        else
        {
          RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;
        }
        settings.save();
      }
      else if(recdat.data[0] == 3)
      {
        if(thermalManager.fan_speed[0])
        {
          RTS_SndData(1, PRINTER_FANOPEN_TITLE_VP);
          thermalManager.set_fan_speed(0, 0);
        }
        else
        {
          RTS_SndData(0, PRINTER_FANOPEN_TITLE_VP);
          thermalManager.set_fan_speed(0, 255);
        }
      }
      else if(recdat.data[0] == 4)
      {
        if(LEDStatus)
        {
          RTS_SndData(0, PRINTER_LEDOPEN_TITLE_VP);
          // digitalWrite(LED_CONTROL_PIN, HIGH);
          LEDStatus = false;
        }
        else
        {
          RTS_SndData(1, PRINTER_LEDOPEN_TITLE_VP);
          // digitalWrite(LED_CONTROL_PIN, LOW);
          LEDStatus = true;
        }
      }
      else if(recdat.data[0] == 5)
      {
        RTS_SndData(ExchangePageBase + 15, ExchangepageAddr);
        change_page_font = 15;
      }
      else if(recdat.data[0] == 6)
      {
      }
      else if(recdat.data[0] == 7)
      {
        RTS_SndData(ExchangePageBase + 14, ExchangepageAddr);
        change_page_font = 14;
        settings.save();
      }
      else if(recdat.data[0] == 8)
      {
        if(runout.enabled)
        {
          RTS_SndData(1, FILAMENT_CONTROL_ICON_VP);
          runout.enabled = false;
        }
        else
        {
          RTS_SndData(0, FILAMENT_CONTROL_ICON_VP);
          runout.enabled = true;
        }
        settings.save();
      }
      else if(recdat.data[0] == 9)
      {
        if(recovery.enabled)
        {
          RTS_SndData(1, POWERCONTINUE_CONTROL_ICON_VP);
          recovery.enabled = false;
          if(CardReader::flag.mounted) //rock_20220701 修复断电续打开关都是开的bug
          {
            #if ENABLED(SDSUPPORT) && ENABLED(POWER_LOSS_RECOVERY)
              // card.removeJobRecoveryFile();
              if (card.jobRecoverFileExists()) {
                // recovery.init(); // 不清除断电信息
                card.removeFile(recovery.filename);
              }
            #endif
          }
        }
        else
        {
          RTS_SndData(0, POWERCONTINUE_CONTROL_ICON_VP);
          recovery.enabled = true;
          recovery.save(true);
        }
      }
      else if(recdat.data[0] == 10)
      {
        RTS_SndData(ExchangePageBase + 32, ExchangepageAddr);
        change_page_font = 32;
        settings.save();
      }
      break;

    case PrintSpeedEnterKey:
      feedrate_percentage = recdat.data[0];
      RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
      SERIAL_ECHOLNPAIR("M220 S", feedrate_percentage);
      break;

    case StopPrintKey:
      if(recdat.data[0] == 1)  //停止打印弹窗进入
      {
        
        if(home_flag)break;
        RTS_SndData(ExchangePageBase + 13, ExchangepageAddr);
        change_page_font = 13;
        
      }
      else if(recdat.data[0] == 2) //停止打印弹窗确认
      {
        if(PoweroffContinue == false) // rock_20220708 云打印
        {
          SERIAL_ECHOLNPAIR("ok");
          while((planner.movesplanned() < 2) && (destination != current_position))
          {
            idle();
          }
          queue.clear();
          delay(20);

          thermalManager.setTargetHotend(0, 0);
          RTS_SndData(0, HEAD_SET_TEMP_VP);
          thermalManager.setTargetBed(0);
          RTS_SndData(0, BED_SET_TEMP_VP);
          SERIAL_ECHOLN("M79 S4");
          queue.enqueue_now_P(PSTR("M79 S4"));   // 4:cloud print stop
          g_cloudPLRStatusValue = CLOUD_PLR_NULL_STATE;
          RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
          gcodePicDispalyOnOff(DEFAULT_PRINT_MODEL_VP, true);
          change_page_font = 1;
        }
        if(G29_flag||home_flag)
        {
          break;
        }
        else if(PoweroffContinue == true)
        {
          RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
          change_page_font = 40;
          waitway = 7;
          RTS_SndData(0, PRINT_TIME_HOUR_VP);
          RTS_SndData(0, PRINT_TIME_MIN_VP);
          thermalManager.setTargetHotend(0, 0);
          RTS_SndData(0, HEAD_SET_TEMP_VP);
          thermalManager.setTargetBed(0);
          RTS_SndData(0, BED_SET_TEMP_VP);
          temphot = 0;
          thermalManager.zero_fan_speeds();
          Update_Time_Value = 0;
          RTS_SDcard_Stop();
          SERIAL_ECHOLN("M79 S4");
          SERIAL_ECHOLN("ok");
          queue.clear();
          queue.enqueue_now_P(PSTR("G28 XY"));
          queue.enqueue_now_P(PSTR("M84"));
          g_cloudPLRStatusValue = CLOUD_PLR_NULL_STATE;
        } 
        
      }
      else if(recdat.data[0] == 3)   //停止打印弹窗返回
      {
        if(card.isPrinting() || (wait_for_heatup == true)|| home_flag) //暂停
        {
          RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;
        }
        else //继续打印
        {
          RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
          change_page_font = 12;
        }
      }
      else if(recdat.data[0] == 4)
      {
        if(!planner.has_blocks_queued()) //rock_20220401
        {
          if(PoweroffContinue == true)
          {
            
            SERIAL_ECHOLN("ok");

            
            runout.filament_ran_out = false;
            RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
            change_page_font = 40;
            waitway = 7;
            #if ENABLED(FILAMENT_RUNOUT_SENSOR)
              if(runout.enabled == true)
              {
                pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;
                ui.pause_show_message(PAUSE_MESSAGE_RESUME);
                queue.inject_P(PSTR("M108"));
              }
            #endif
            RTS_SndData(0, PRINT_TIME_HOUR_VP);
            RTS_SndData(0, PRINT_TIME_MIN_VP);
            Update_Time_Value = 0;
            temphot = 0;
            card.flag.abort_sd_printing = true;
            queue.clear();
            quickstop_stepper();

            print_job_timer.abort();
            // delay(10);
            while(planner.has_blocks_queued())
            {
              idle();
            }
            thermalManager.setTargetHotend(0, 0);
            thermalManager.setTargetBed(0);
            thermalManager.zero_fan_speeds();
            while(thermalManager.temp_hotend[0].target > 0)
            {
              thermalManager.setTargetHotend(0, 0);
              idle();
            }
            RTS_SDcard_Stop();
        
            SERIAL_ECHOLN("M79 S4");   // 4:cloud print stop
            // RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
            // change_page_font = 1;
           
          }
          else if(PoweroffContinue == false)
          {
            SERIAL_ECHOLN("ok");
            PoweroffContinue = true;
           runout.filament_ran_out = false;
            RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
            change_page_font = 40;
            waitway = 7;
            #if ENABLED(FILAMENT_RUNOUT_SENSOR)
              if(runout.enabled == true)
              {
                pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;
                ui.pause_show_message(PAUSE_MESSAGE_RESUME);
                queue.inject_P(PSTR("M108"));
              }
            #endif
            RTS_SndData(0, PRINT_TIME_HOUR_VP);
            RTS_SndData(0, PRINT_TIME_MIN_VP);
            Update_Time_Value = 0;
            temphot = 0;
            card.flag.abort_sd_printing = true;
            queue.clear();
            quickstop_stepper();

            print_job_timer.abort();
            // delay(10);
            while(planner.has_blocks_queued())
            {
              idle();
            }
            thermalManager.setTargetHotend(0, 0);
            thermalManager.setTargetBed(0);
            thermalManager.zero_fan_speeds();
            while(thermalManager.temp_hotend[0].target > 0)
            {
              thermalManager.setTargetHotend(0, 0);
              idle();
            }
            RTS_SDcard_Stop();
        
            PoweroffContinue = false;
            SERIAL_ECHOLN("M79 S4");   // 4:cloud print stop
            RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
            gcodePicDispalyOnOff(DEFAULT_PRINT_MODEL_VP, true);
            change_page_font = 1;
          }
        }
      }
      else if(recdat.data[0] == 5) //拔卡打印弹窗停止打印
      {
        if(PoweroffContinue == true)
        {
          RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
          change_page_font = 40;
          waitway = 7;
          RTS_SndData(0, PRINT_TIME_HOUR_VP);
          RTS_SndData(0, PRINT_TIME_MIN_VP);
          thermalManager.setTargetHotend(0, 0);
          RTS_SndData(0, HEAD_SET_TEMP_VP);
          thermalManager.setTargetBed(0);
          RTS_SndData(0, BED_SET_TEMP_VP);
          temphot = 0;
          thermalManager.zero_fan_speeds();
          Update_Time_Value = 0;
          RTS_SDcard_Stop();
        }

     



      }
      break;

    case PausePrintKey:
      if(recdat.data[0] == 1)
      {
        // 云打印功能
        if(PoweroffContinue == false) {
          RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
          change_page_font = 11;
        }
        //rock_20220714 解决打印开始前反复点击暂停会重启的问题
        if((G29_flag == true) || (home_flag == true) || wait_for_heatup)
        {
          break;
        }
        RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
        change_page_font = 11;
      }
      else if(recdat.data[0] == 2)
      {
        if(PoweroffContinue == false)
        {
          RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
          SERIAL_ECHOLN("M79 S2");   // 3:cloud print resume
          queue.enqueue_now_P(PSTR("M79 S2"));   // 2:cloud print pause
          Update_Time_Value = 0;
          print_job_timer.pause();
          RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
          change_page_font = 12;
        }
        if((G29_flag == true) || (home_flag == true) || wait_for_heatup)
        {
          break;
        }
        else if((PoweroffContinue) && card.isPrinting() && (!wait_for_heatup))
        {
          while (ABS(thermalManager.degHotend(0) - thermalManager.degTargetHotend(0)) > TEMP_WINDOW)
          {
            idle();
          }
          RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
          waitway = 1;
          pause_e = current_position[E_AXIS] - 3;
          if(!temphot)
          {
            temphot = thermalManager.temp_hotend[0].target;
          }
          card.pauseSDPrint();
          print_job_timer.pause();

          pause_action_flag = true;
          Update_Time_Value = 0;
          planner.synchronize();
          sdcard_pause_check = false;
        }
      }
      else if(recdat.data[0] == 3)
      {
        if(card.isPrinting() || (wait_for_heatup == true))
        {
          RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;
        }
        else
        {
          RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
          change_page_font = 12;
        }
      }
      break;

    case ResumePrintKey:
      if(recdat.data[0] == 1)
      {
        #if ENABLED(FILAMENT_RUNOUT_SENSOR)
          if((1 == READ(FIL_RUNOUT_PIN)) && (runout.enabled == true))
          {
            RTS_SndData(ExchangePageBase + 7, ExchangepageAddr);
            change_page_font = 7;
            break;
          }
        #endif

        if(PoweroffContinue == true)
        {
          RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;

          #if ENABLED(HAS_RESUME_CONTINUE)
            if(wait_for_user)
            {
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
          SERIAL_ECHOLN("M79 S3");   // 3:cloud print resume
        } else {
          SERIAL_ECHOLN("M79 S3");   // 3:cloud print resume
          // RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
          queue.enqueue_now_P(PSTR("M79 S3"));   // 3:cloud print resume
        }
      }
      else if(recdat.data[0] == 2)
      {
        SERIAL_ECHOLNPAIR("ok");
        if(G29_flag||home_flag)
        {
          break;
        }
        else if(sdcard_pause_check == true)
        {
          if(thermalManager.temp_hotend[0].target >= EXTRUDE_MINTEMP)
          {
            thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
          }
          else if(temphot >= EXTRUDE_MINTEMP)
          {
            thermalManager.setTargetHotend(temphot, 0);
          }
          else
          {
            thermalManager.setTargetHotend(200, 0);
          }
          #if ENABLED(FILAMENT_RUNOUT_SENSOR)
            if((1 == READ(FIL_RUNOUT_PIN)) && (runout.enabled == true))
            {
              
              RTS_SndData(ExchangePageBase + 7, ExchangepageAddr);
              change_page_font = 7;
              break;
            }
            else
            {
              if(!planner.has_blocks_queued())
              {
                RTS_SndData(ExchangePageBase + 8, ExchangepageAddr);
                change_page_font = 8;
              }
              else
              {
                RTS_SndData(ExchangePageBase + 7, ExchangepageAddr);
                change_page_font = 7;
                break;
              }
            }
          #endif
        }
        else if(sdcard_pause_check == false)
        {
          if(CardReader::flag.mounted)
          {
            if(1 == READ(FIL_RUNOUT_PIN) && (runout.enabled == true))
            {
              RTS_SndData(ExchangePageBase + 7, ExchangepageAddr);
              change_page_font = 7;
              break;
            }
            else
            {
              PoweroffContinue = true;
              char cmd[30];
              char *c;
              sprintf_P(cmd, PSTR("M23 %s"), CardRecbuf.Cardfilename[CardRecbuf.recordcount]);
              for(c = &cmd[4]; *c; c++)
                *c = tolower(*c);

              memset(cmdbuf, 0, sizeof(cmdbuf));
              strcpy(cmdbuf, cmd);
              queue.enqueue_one_now(cmd);
              queue.enqueue_now_P(PSTR("M24"));
              // clean screen.
              for (int j = 0; j < 20; j ++)
              {
                RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
              }

              RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], PRINT_FILE_TEXT_VP);

              delay(2);

              #if ENABLED(BABYSTEPPING)
                RTS_SndData(0, AUTO_BED_LEVEL_ZOFFSET_VP);
              #endif
              feedrate_percentage = 100;
              RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
              RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
              RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
              Update_Time_Value = 0;
              // 1:cloud print satrt
              SERIAL_ECHOLN("M79 S1");

            }
          }
          else
          {
            break;
          }
        }
      }
      else if(recdat.data[0] == 3)
      {
        #if ENABLED(FILAMENT_RUNOUT_SENSOR)
          if((1 == READ(FIL_RUNOUT_PIN)) && (runout.enabled == true))
          {
            RTS_SndData(ExchangePageBase + 7, ExchangepageAddr);
            change_page_font = 7;
            break;
          }
        #endif
        if(PoweroffContinue == true)
        {
          runout.filament_ran_out = false;
          RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;
          SERIAL_ECHOLN("M79 S1");
            SERIAL_ECHOLN("M79 S3");
          if(temphot > 0)
          {
            RTS_SndData(temphot, HEAD_SET_TEMP_VP);
            thermalManager.setTargetHotend(temphot, 0);
            if (temphot > (thermalManager.temp_hotend[0].celsius - 5))
            {
              thermalManager.wait_for_hotend(0);
            }
            while (ABS(thermalManager.degHotend(0) - thermalManager.degTargetHotend(0)) > TEMP_WINDOW)
            {
              idle();
              if(card.flag.abort_sd_printing)break;//rock _20220913  
            }
          }
              // Move XY to starting position, then Z
          prepare_internal_move_to_destination(NOZZLE_PARK_XY_FEEDRATE);

          // Move Z_AXIS to saved position
          prepare_internal_move_to_destination(NOZZLE_PARK_Z_FEEDRATE);
          pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;
          ui.pause_show_message(PAUSE_MESSAGE_RESUME);
          queue.inject_P(PSTR("M108"));
          
          card.startOrResumeFilePrinting();
          print_job_timer.start();
          Update_Time_Value = 0;
          sdcard_pause_check = true;
          
        }
        else if(PoweroffContinue == false)
        {
          if(CardRecbuf.recordcount > 0)
          {
            char cmd[30];
            char *c;
            sprintf_P(cmd, PSTR("M23 %s"), CardRecbuf.Cardfilename[CardRecbuf.recordcount]);
            for (c = &cmd[4]; *c; c++)
              *c = tolower(*c);

            PoweroffContinue = true;
            memset(cmdbuf, 0, sizeof(cmdbuf));
            strcpy(cmdbuf, cmd);
            queue.enqueue_one_now(cmd);
            delay(20);
            queue.enqueue_now_P(PSTR("M24"));
            // clean screen.
            for (int j = 0; j < 20; j ++)
            {
              RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
            }

            RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], PRINT_FILE_TEXT_VP);

            delay(2);

            #if ENABLED(BABYSTEPPING)
              RTS_SndData(0, AUTO_BED_LEVEL_ZOFFSET_VP);
            #endif
            feedrate_percentage = 100;
            RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
            RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
            RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
            Update_Time_Value = 0;
            // 1:cloud print satrt
            SERIAL_ECHOLN("M79 S1");
            SERIAL_ECHOLN("M79 S3");
          }
          else
          {
            runout.filament_ran_out = false;
            // Move XY to starting position, then Z
            prepare_internal_move_to_destination(NOZZLE_PARK_XY_FEEDRATE);

            // Move Z_AXIS to saved position
            prepare_internal_move_to_destination(NOZZLE_PARK_Z_FEEDRATE);
            pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;
            ui.pause_show_message(PAUSE_MESSAGE_RESUME);
            // queue.inject_P(PSTR("M108"));
            RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
            change_page_font = 10;

            // card.startOrResumeFilePrinting();
            print_job_timer.start();
            Update_Time_Value = 0;
            sdcard_pause_check = true;
            SERIAL_ECHOLN("ok");
            if(temphot > 0)
            {
              RTS_SndData(temphot, HEAD_SET_TEMP_VP);
              thermalManager.setTargetHotend(temphot, 0);
              if (temphot > (thermalManager.temp_hotend[0].celsius - 5))
              {
                thermalManager.wait_for_hotend(0);
              }
              while (ABS(thermalManager.degHotend(0) - thermalManager.degTargetHotend(0)) > TEMP_WINDOW)
              {
                idle();
              }
            }
            SERIAL_ECHOLN("M79 S3");
          }
        }
        else
        {
          runout.filament_ran_out = false;
          // Move XY to starting position, then Z
          prepare_internal_move_to_destination(NOZZLE_PARK_XY_FEEDRATE);

          // Move Z_AXIS to saved position
          prepare_internal_move_to_destination(NOZZLE_PARK_Z_FEEDRATE);

          pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;
          ui.pause_show_message(PAUSE_MESSAGE_RESUME);

          RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;

          print_job_timer.start();
          Update_Time_Value = 0;
          sdcard_pause_check = true;
        }
      }
      else if(recdat.data[0] == 4)
      {
        //  if(CardReader::flag.mounted)
        if(IS_SD_INSERTED())   //有卡
        {
          SD_Card_status = true;
          card.startOrResumeFilePrinting();
          print_job_timer.start();
          Update_Time_Value = 0;
          sdcard_pause_check = true;
          sd_printing_autopause = false;
          HMI_lcd_flag.remove_card_flag=false;
          temp_remove_card_flag=false;
          
          RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;
          gcode.process_subcommands_now_P(PSTR("M24"));
        }
        else
        {     
            CardUpdate = true;
            RTS_SDCardUpate();
            // card.mount();
            // SERIAL_ECHO_MSG("ROCK_MOVE_CARD1111\r\n");
            RTS_SndData(ExchangePageBase + 47, ExchangepageAddr);
            change_page_font = 47;          
        }
      }
      break;

    case ZoffsetEnterKey:
      last_zoffset = zprobe_zoffset;
      if(recdat.data[0] >= 32768)
      {
        zprobe_zoffset = ((float)recdat.data[0] - 65536) / 100;
        zprobe_zoffset -= 0.001;
      }
      else
      {
        zprobe_zoffset = ((float)recdat.data[0]) / 100;
        zprobe_zoffset += 0.001;
      }
      if(WITHIN((zprobe_zoffset), Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX))
      {
        babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
        SERIAL_ECHO_MSG("babystep.add_mm():", zprobe_zoffset - last_zoffset);
      }
      HAL_watchdog_refresh();
      probe.offset.z = zprobe_zoffset;
      // settings.save();
      break;

    case TempControlKey:
      if(recdat.data[0] == 2)
      {
        RTS_SndData(ExchangePageBase + 20, ExchangepageAddr);
        change_page_font = 20;
      }
      else if(recdat.data[0] == 3)
      {
        RTS_SndData(ui.material_preset[0].hotend_temp, PREHEAT_PLA_SET_NOZZLE_TEMP_VP);
        RTS_SndData(ui.material_preset[0].bed_temp, PREHEAT_PLA_SET_BED_TEMP_VP);
        delay(2);
        RTS_SndData(ExchangePageBase + 22, ExchangepageAddr);
        change_page_font = 22;
      }
      else if(recdat.data[0] == 4)
      {
        RTS_SndData(ui.material_preset[1].hotend_temp, PREHEAT_ABS_SET_NOZZLE_TEMP_VP);
        RTS_SndData(ui.material_preset[1].bed_temp, PREHEAT_ABS_SET_BED_TEMP_VP);
        delay(2);
        RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
        change_page_font = 23;
      }
      else if(recdat.data[0] == 5)
      {
        thermalManager.temp_hotend[0].target = ui.material_preset[0].hotend_temp;
        thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
        RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
        thermalManager.temp_bed.target = ui.material_preset[0].bed_temp;
        thermalManager.setTargetBed(thermalManager.temp_bed.target);
              #if ENABLED(BED_TEMP_COMP)     
          if (thermalManager.temp_bed.target > 65 && thermalManager.temp_bed.target <= 85)
              RTS_SndData(thermalManager.temp_bed.target-5, BED_SET_TEMP_VP);
          else if (thermalManager.temp_bed.target > 85 && thermalManager.temp_bed.target <= 127)
              RTS_SndData(thermalManager.temp_bed.target-7, BED_SET_TEMP_VP);
      #else
          RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      #endif
        // RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      }
      else if(recdat.data[0] == 6)
      {
        thermalManager.temp_hotend[0].target = ui.material_preset[1].hotend_temp;
        thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
        RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
        thermalManager.temp_bed.target = ui.material_preset[1].bed_temp;
        thermalManager.setTargetBed(thermalManager.temp_bed.target);
      #if ENABLED(BED_TEMP_COMP)     
          if (thermalManager.temp_bed.target > 65 && thermalManager.temp_bed.target <= 85)
              RTS_SndData(thermalManager.temp_bed.target-5, BED_SET_TEMP_VP);
          else if (thermalManager.temp_bed.target > 85 && thermalManager.temp_bed.target <= 127)
              RTS_SndData(thermalManager.temp_bed.target-7, BED_SET_TEMP_VP);
      #else
          RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      #endif
      }
      else if(recdat.data[0] == 7)
      {
        RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
        change_page_font = 21;
      }
      else if(recdat.data[0] == 8)
      {
        if (true == g_heaterLoadTempAdd) break;
        RTS_SndData(ExchangePageBase + 20, ExchangepageAddr);
        change_page_font = 20;
      }
      break;

    case CoolDownKey:
      if(recdat.data[0] == 1)
      {
        thermalManager.setTargetHotend(0, 0);
        RTS_SndData(0, HEAD_SET_TEMP_VP);
        thermalManager.setTargetBed(0);
        RTS_SndData(0, BED_SET_TEMP_VP);
        thermalManager.fan_speed[0] = 255;
        RTS_SndData(0, PRINTER_FANOPEN_TITLE_VP);
      }
      else if(recdat.data[0] == 2)
      {
        RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
        change_page_font = 21;
      } else if (recdat.data[0] == 3) { // 声音设置
        if (g_soundSetOffOn == 2) {
          g_soundSetOffOn = 1;
          RTS_SndData(DC_SOUND_SET_ON, DC_SOUND_SET_DDR);
          RTS_SndData(0, SOUND_SETTING_OFF_ON_VP);
        } else {
          g_soundSetOffOn = 2;
          RTS_SndData(DC_SOUND_SET_OFF, DC_SOUND_SET_DDR);
          RTS_SndData(1, SOUND_SETTING_OFF_ON_VP);
        }
        settings.save();
      }else if (recdat.data[0] == 4) { // 自动PID
        RTS_SndData(ExchangePageBase + 48, ExchangepageAddr);
        g_uiAutoPIDFlag = true;
        thermalManager.setTargetBed(0); // 关闭热床加热
        thermalManager.setTargetHotend(0, 0); // 关闭喷嘴加热
        last_target_temperature[0] = thermalManager.temp_hotend[0].target;
        last_target_temperature_bed = thermalManager.temp_bed.target;
        RTS_SndData(g_autoPIDHeaterTempTarget, HEAD_SET_TEMP_VP);
        RTS_SndData(g_autoPIDHotBedTempTarget, BED_SET_TEMP_VP);
      }
      break;
    case HeaterTempEnterKey:  //rock_20220624
      if (false == g_uiAutoPIDFlag) {  
        temphot = recdat.data[0];
        thermalManager.setTargetHotend(temphot, 0);
        RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
      } else { // 自动PID
          if ((g_uiAutoPIDRuningFlag == true) || (recdat.data[0] < AUTO_PID_NOZZLE_TARGET_TEMP_MIN)) { //如果正在自动PID就忽略
              RTS_SndData(g_autoPIDHeaterTempTarget, HEAD_SET_TEMP_VP);
              break; 
          }
          g_autoPIDHeaterTempTarget = recdat.data[0];
          RTS_SndData(g_autoPIDHeaterTempTarget, HEAD_SET_TEMP_VP);
      }
      break;

    case HotBedTempEnterKey:
      if (false == g_uiAutoPIDFlag) { 
        tempbed = recdat.data[0];
        temp_bed_display=recdat.data[0];
        #if ENABLED(BED_TEMP_COMP)
            if (tempbed > 60 && tempbed <= 80)
                tempbed += 5;
            else if (tempbed > 80 && tempbed <= 120)
                tempbed += 7;
        #endif
        thermalManager.setTargetBed(tempbed);
         RTS_SndData(temp_bed_display, BED_SET_TEMP_VP);
      } else { // 自动PID
        if ((g_uiAutoPIDRuningFlag == true) || (recdat.data[0] < AUTO_PID_HOTBED_TARGET_TEMP_MIN)) { //如果正在自动PID就忽略
            RTS_SndData(g_autoPIDHotBedTempTarget, BED_SET_TEMP_VP);
            break; 
        }
        tempbed = 0;
        g_autoPIDHotBedTempTarget = recdat.data[0];
        RTS_SndData(g_autoPIDHotBedTempTarget, BED_SET_TEMP_VP);
      }
     
      break;

    // case HeaterTempEnterKey:
    //   temphot = recdat.data[0];
    //   thermalManager.temp_hotend[0].target = temphot;
    //   thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
    //   RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
    //   break;

    // case HotBedTempEnterKey:
    //   thermalManager.temp_bed.target = recdat.data[0];
    //   thermalManager.setTargetBed(thermalManager.temp_bed.target);
    //   RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
    //   break;

    case PrepareEnterKey:
      if(recdat.data[0] == 1)
      {
        RTS_SndData(ExchangePageBase + 28, ExchangepageAddr);
        change_page_font = 28;
      }
      else if(recdat.data[0] == 2)
      {
        RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
        change_page_font = 33;
      }
      else if(recdat.data[0] == 3)
      {
        if (true == g_heaterLoadTempAdd) break;
        RTS_SndData(10 * current_position[X_AXIS], AXIS_X_COORD_VP);
        RTS_SndData(10 * current_position[Y_AXIS], AXIS_Y_COORD_VP);
        RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);
        delay(2);
        RTS_SndData(ExchangePageBase + 16, ExchangepageAddr);
        change_page_font = 16;
      }
      else if(recdat.data[0] == 4)
      {
        // OUT_WRITE(SHUTIDOWN_PIN, LOW);
        delay(2000);
      }
      else if(recdat.data[0] == 5)
      {
        RTS_SndData(MACHINE_TYPE, MACHINE_TYPE_ABOUT_TEXT_VP);
        RTS_SndData(FIRMWARE_VERSION, FIREWARE_VERSION_ABOUT_TEXT_VP);
        // RTS_SndData(SCREEN_VERSION, PRINTER_DISPLAY_VERSION_TEXT_VP);
        RTS_SndData(HARDWARE_VERSION, HARDWARE_VERSION_ABOUT_TEXT_VP);
        RTS_SndData(PRINT_SIZE, PRINTER_PRINTSIZE_TEXT_VP);
        delay(5);
        if(1 == lang)
        {
          RTS_SndData(CORP_WEBSITE_C, WEBSITE_ABOUT_TEXT_VP);
        }
        else
        {
          RTS_SndData(CORP_WEBSITE_E, WEBSITE_ABOUT_TEXT_VP);
        }
        RTS_SndData(ExchangePageBase + 24, ExchangepageAddr);
        change_page_font = 24;
      }
      else if(recdat.data[0] == 6)
      {
        queue.enqueue_now_P(PSTR("M84"));
        RTS_SndData(1, MOTOR_FREE_ICON_VP);
        g_uiXYAxisEnable = false;
        g_uiZAxisEnable = false;
      }
      else if(recdat.data[0] == 7)
      {
        RTS_SndData(ExchangePageBase + 43, ExchangepageAddr);
        change_page_font = 43;
      }
      else if(recdat.data[0] == 8)
      {
        settings.save();
        RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
        change_page_font = 21;
      }
      else if(recdat.data[0] == 9)
      {
        RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
        gcodePicDispalyOnOff(DEFAULT_PRINT_MODEL_VP, true);
        change_page_font = 1;
      }
      else if(recdat.data[0] == 0xA)
      {
        RTS_SndData(ExchangePageBase + 42, ExchangepageAddr);
        change_page_font = 42;
      }
      else if(recdat.data[0] == 0xB)
      {
         #if ENABLED(HAS_MENU_RESET_WIFI)
          WIFI_STATE = PRESSED;
          OUT_WRITE(RESET_WIFI_PIN, LOW);
        #endif
        flag_counter_wifireset = true;
        RTS_SndData(ExchangePageBase + 45, ExchangepageAddr);
        change_page_font = 45;
      }
      else if(recdat.data[0] == 0xC)
      {
        RTS_SndData(ExchangePageBase + 44, ExchangepageAddr);
        change_page_font = 44;
      }
      else if(recdat.data[0] == 0xD)
      {
        settings.reset();
        settings.save();
        language_change_font = lang = 2;
        languagedisplayUpdate();
        for(int i = 0;i < 9;i ++)
        {
          RTS_SndData(0, LANGUAGE_CHINESE_TITLE_VP + i);
        }
        RTS_SndData(1, LANGUAGE_CHINESE_TITLE_VP + (language_change_font - 1));
        RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
        change_page_font = 33;

        RTS_SndData(ui.material_preset[0].hotend_temp, PREHEAT_PLA_SET_NOZZLE_TEMP_VP);
        RTS_SndData(ui.material_preset[0].bed_temp, PREHEAT_PLA_SET_BED_TEMP_VP);
        RTS_SndData(ui.material_preset[1].hotend_temp, PREHEAT_ABS_SET_NOZZLE_TEMP_VP);
        RTS_SndData(ui.material_preset[1].bed_temp, PREHEAT_ABS_SET_BED_TEMP_VP);
        delay(20);

        RTS_SndData(default_max_feedrate[X_AXIS], MAX_VELOCITY_XAXIS_DATA_VP);
        RTS_SndData(default_max_feedrate[Y_AXIS], MAX_VELOCITY_YAXIS_DATA_VP);
        RTS_SndData(default_max_feedrate[Z_AXIS], MAX_VELOCITY_ZAXIS_DATA_VP);
        RTS_SndData(default_max_feedrate[E_AXIS], MAX_VELOCITY_EAXIS_DATA_VP);
        delay(20);

        RTS_SndData(default_max_acceleration[X_AXIS], MAX_ACCEL_XAXIS_DATA_VP);
        RTS_SndData(default_max_acceleration[Y_AXIS], MAX_ACCEL_YAXIS_DATA_VP);
        RTS_SndData(default_max_acceleration[Z_AXIS], MAX_ACCEL_ZAXIS_DATA_VP);
        RTS_SndData(default_max_acceleration[E_AXIS], MAX_ACCEL_EAXIS_DATA_VP);
        delay(20);

        RTS_SndData(default_max_jerk[X_AXIS] * 100, MAX_JERK_XAXIS_DATA_VP);
        RTS_SndData(default_max_jerk[Y_AXIS] * 100, MAX_JERK_YAXIS_DATA_VP);
        RTS_SndData(default_max_jerk[Z_AXIS] * 100, MAX_JERK_ZAXIS_DATA_VP);
        RTS_SndData(default_max_jerk[E_AXIS] * 100, MAX_JERK_EAXIS_DATA_VP);
        delay(20);

        RTS_SndData(default_axis_steps_per_unit[X_AXIS] * 10, MAX_STEPSMM_XAXIS_DATA_VP);
        RTS_SndData(default_axis_steps_per_unit[Y_AXIS] * 10, MAX_STEPSMM_YAXIS_DATA_VP);
        RTS_SndData(default_axis_steps_per_unit[Z_AXIS] * 10, MAX_STEPSMM_ZAXIS_DATA_VP);
        RTS_SndData(default_axis_steps_per_unit[E_AXIS] * 10, MAX_STEPSMM_EAXIS_DATA_VP);
        delay(20);

        RTS_SndData(default_nozzle_ptemp * 100, NOZZLE_TEMP_P_DATA_VP);
        RTS_SndData(default_nozzle_itemp * 100, NOZZLE_TEMP_I_DATA_VP);
        RTS_SndData(default_nozzle_dtemp * 100, NOZZLE_TEMP_D_DATA_VP);
        delay(20);
        RTS_SndData(default_hotbed_ptemp * 100, HOTBED_TEMP_P_DATA_VP);
        RTS_SndData(default_hotbed_itemp * 100, HOTBED_TEMP_I_DATA_VP);
        RTS_SndData(default_hotbed_dtemp * 10, HOTBED_TEMP_D_DATA_VP);
        delay(1000);
      }
      else if(recdat.data[0] == 0xE)
      {
        if(!planner.has_blocks_queued())
        {
          RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
          change_page_font = 33;
        }
      }
      else if(recdat.data[0] == 0xF) // 退出调平
      {
        if (home_flag == true) { // 如果正在回零就返回
          break;
        }
        leveing_flag=false;
        gcode.process_subcommands_now_P(PSTR("M420 S1")); // 退出调平界面
        if(!planner.has_blocks_queued())
        {
          gcode.process_subcommands_now_P(PSTR("G0 Z5"));
          RTS_SndData(10 * current_position[X_AXIS], AXIS_X_COORD_VP);
          RTS_SndData(10 * current_position[Y_AXIS], AXIS_Y_COORD_VP);
          RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);
          RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
          change_page_font = 21;
        }
        settings.save();
        st_bedNozzleHeightCal.isBedLeveling=false;   
      }

      break;

    case BedLevelKey:
    
      if (home_flag == true) { // 如果正在G28就返回
          break;
      }
      if(recdat.data[0] == 1)
      {
        waitway = 0;
        gcode.process_subcommands_now_P(PSTR("M420 S0"));
        planner.synchronize();
        queue.enqueue_now_P(PSTR("G28"));
        queue.enqueue_now_P(PSTR("M420 S0"));
        queue.enqueue_one_P(PSTR("G1 X110 Y110 F3000"));
        queue.enqueue_now_P(PSTR("G1 F200 Z0.0"));           
      }
      else if(recdat.data[0] == 2)
      {
        last_zoffset = zprobe_zoffset;
        if(WITHIN((zprobe_zoffset + 0.05), -5.02, 5.02))
        {
          #if ENABLED(HAS_LEVELING)
            zprobe_zoffset = (zprobe_zoffset + 0.05);
            zprobe_zoffset = zprobe_zoffset - 0.0001;
          #endif
          babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
          probe.offset.z = zprobe_zoffset;
          RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
        }
      }
      else if(recdat.data[0] == 3)
      {
        last_zoffset = zprobe_zoffset;
        if (WITHIN((zprobe_zoffset - 0.05), -5.02, 5.02))
        {
          #if ENABLED(HAS_LEVELING)
            zprobe_zoffset = (zprobe_zoffset - 0.05);
            zprobe_zoffset = zprobe_zoffset + 0.0001;
          #endif
          babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
          probe.offset.z = zprobe_zoffset;
        }
        RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
      }
      else if(recdat.data[0] == 4)
      {
        gcode.process_subcommands_now_P(PSTR("M420 S0"));// 进入调平页面先关掉自动补偿
        if(!planner.has_blocks_queued())
        {
          RTS_SndData(ExchangePageBase + 25, ExchangepageAddr);
          change_page_font = 25;
          waitway = 0;
          queue.enqueue_one_P(PSTR("G1 F200 Z3.0"));
          queue.enqueue_one_P(PSTR("G1 X110 Y110 F3000"));
          queue.enqueue_one_P(PSTR("G1 F200 Z0.0"));
          g_uiZOffsetHomeOkFlag = true;
        }
      }
      else if(recdat.data[0] == 5)
      {
        if(home_flag)break;
        gcode.process_subcommands_now_P(PSTR("M420 S0"));// 进入调平页面先关掉自动补偿

          if(!planner.has_blocks_queued())
          {
            waitway = 0;
            if (g_uiZOffsetHomeOkFlag == false) {
              gcode.process_subcommands_now_P(PSTR("G28"));
              gcode.process_subcommands_now_P(PSTR("M420 S0"));
            }
            queue.enqueue_now_P(PSTR("G1 F600 Z3"));
            queue.enqueue_now_P(PSTR("G1 X110 Y110 F3000"));
            queue.enqueue_now_P(PSTR("G1 F600 Z0"));
            g_uiZOffsetHomeOkFlag = true;
          }

          // queue.enqueue_now_P(PSTR("G28"));
          // queue.enqueue_now_P(PSTR("G1 X110 Y110 F3000"));
          // queue.enqueue_now_P(PSTR("G1 F200 Z0.0"));

      }
      else if (recdat.data[0] == 6)
      {
        // Assitant Level , Front Left 2
        gcode.process_subcommands_now_P(PSTR("M420 S0"));//Rock_20220817 进入调平页面先关掉自动补偿
      
          if(!planner.has_blocks_queued())
          {
            waitway = 0;
            queue.enqueue_now_P(PSTR("G1 F600 Z3"));
            queue.enqueue_now_P(PSTR("G1 X25 Y25 F3000"));
            queue.enqueue_now_P(PSTR("G1 F600 Z0"));
            g_uiZOffsetHomeOkFlag = false;
          }

   
        
      }
      else if (recdat.data[0] == 7)
      {
        // Assitant Level , Front Right 3
        gcode.process_subcommands_now_P(PSTR("M420 S0"));//Rock_20220817 进入调平页面先关掉自动补偿
          if(!planner.has_blocks_queued())
          {
            waitway = 0;
            queue.enqueue_now_P(PSTR("G1 F600 Z3"));
            queue.enqueue_now_P(PSTR("G1 X195 Y25 F3000"));
            queue.enqueue_now_P(PSTR("G1 F600 Z0"));
            g_uiZOffsetHomeOkFlag = false;
          }

      }
      else if (recdat.data[0] == 8)
      {
        if(home_flag)break;
        gcode.process_subcommands_now_P(PSTR("M420 S0"));//Rock_20220817 进入调平页面先关掉自动补偿
        // Assitant Level , Back Right 4

          if(!planner.has_blocks_queued())
          {
            waitway = 0;
             queue.enqueue_now_P(PSTR("G1 F600 Z3"));
            queue.enqueue_now_P(PSTR("G1 X195 Y195 F3000"));
            queue.enqueue_now_P(PSTR("G1 F200 Z0"));
            g_uiZOffsetHomeOkFlag = false;
          }

      }
      else if (recdat.data[0] == 9)
      {
        // Assitant Level , Back Left 5
        gcode.process_subcommands_now_P(PSTR("M420 S0"));//Rock_20220817 进入调平页面先关掉自动补偿

          if(!planner.has_blocks_queued())
          {
            waitway = 0;
            queue.enqueue_now_P(PSTR("G1 F600 Z3"));
            queue.enqueue_now_P(PSTR("G1 X30 Y195 F3000"));
            queue.enqueue_now_P(PSTR("G1 F600 Z0"));
            g_uiZOffsetHomeOkFlag = false;
          }

      }
      else if(recdat.data[0] == 0x0A)
      { 
        gcode.process_subcommands_now_P(PSTR("M420 S0"));//Rock_20220817 进入调平页面先关掉自动补偿       
        RTS_SndData(0, AUTO_BED_LEVEL_TITLE_VP);
        RTS_SndData(0, AUTO_LEVELING_PERCENT_DATA_VP);
        if(!planner.has_blocks_queued())
        {
          RTS_SndData(ExchangePageBase + 26, ExchangepageAddr);
          change_page_font = 26;
        }
      }
      RTS_SndData(0, MOTOR_FREE_ICON_VP);       
      break;

    case AutoHomeKey:
      if(recdat.data[0] == 1)
      {
        AxisUnitMode = 1;
        axis_unit = 10.0;
        RTS_SndData(ExchangePageBase + 16, ExchangepageAddr);
        change_page_font = 16;
        RTS_SndData(3, MOVEAXIS_UNIT_ICON_VP);
      }
      else if(recdat.data[0] == 2)
      {
        AxisUnitMode = 2;
        axis_unit = 1.0;
        RTS_SndData(ExchangePageBase + 17, ExchangepageAddr);
        change_page_font = 17;
        RTS_SndData(2, MOVEAXIS_UNIT_ICON_VP);
      }
      else if(recdat.data[0] == 3)
      {
        AxisUnitMode = 3;
        axis_unit = 0.1;
        RTS_SndData(ExchangePageBase + 18, ExchangepageAddr);
        change_page_font = 18;
        RTS_SndData(1, MOVEAXIS_UNIT_ICON_VP);
      }
      else if(recdat.data[0] == 4)
      {
        queue.enqueue_now_P(PSTR("G28 X Y"));
        g_uiXYAxisEnable = true;
        Update_Time_Value = 0;
        RTS_SndData(0, MOTOR_FREE_ICON_VP);
      }
      else if(recdat.data[0] == 5)
      {
        if((axis_is_trusted(X_AXIS)) && (axis_is_trusted(Y_AXIS)) && (current_position[X_AXIS] >= 50) && (current_position[Y_AXIS] >= 50))
        {
          queue.enqueue_now_P(PSTR("G28 Z"));
          g_uiZAxisEnable = true;
        }
        else
        {
          queue.enqueue_now_P(PSTR("G28"));
          g_uiXYAxisEnable = true;
          g_uiZAxisEnable = true;
        }
        RTS_SndData(0, MOTOR_FREE_ICON_VP);
        Update_Time_Value = 0;
      }
      
      break;

    case XaxismoveKey:
      if(!planner.has_blocks_queued())
      {
        float x_min, x_max;
        
        x_min = 0;
        x_max = X_MAX_POS;//rock_20220615 更改X轴位置不在中心
        // x_max = X_MAX_POS-8;
        if (false == g_uiXYAxisEnable) { // 如果电机XYZ轴没有使能就先归零
          g_uiXYAxisEnable = true;
          gcode.process_subcommands_now_P(PSTR("G28 XY"));
          if (g_uiZAxisEnable) {
            RTS_SndData(0, MOTOR_FREE_ICON_VP);
          }
        }
        current_position[X_AXIS] = ((float)recdat.data[0]) / 10;
        if(current_position[X_AXIS] < x_min)
        {
          current_position[X_AXIS] = x_min;
        }
        else if(current_position[X_AXIS] > x_max)
        {
          current_position[X_AXIS] = x_max;
        }
        RTS_line_to_current(X_AXIS);
        RTS_SndData(10 * current_position[X_AXIS], AXIS_X_COORD_VP);
        delay(1);
        RTS_SndData(0, MOTOR_FREE_ICON_VP);
        waitway = 0;
      }
      else
      {
        RTS_SndData(10 * current_position[X_AXIS], AXIS_X_COORD_VP);
      }
      break;

    case YaxismoveKey:
      if(!planner.has_blocks_queued())
      {
        float y_min, y_max;
        
        y_min = 0;
        y_max = Y_MAX_POS;
        if (false == g_uiXYAxisEnable) { // 如果电机XYZ轴没有使能就先归零
          g_uiXYAxisEnable = true;
          gcode.process_subcommands_now_P(PSTR("G28XY"));
          if (g_uiZAxisEnable) {
            RTS_SndData(0, MOTOR_FREE_ICON_VP);
          }
        }
        current_position[Y_AXIS] = ((float)recdat.data[0]) / 10;
        if(current_position[Y_AXIS] < y_min)
        {
          current_position[Y_AXIS] = y_min;
        }
        else if(current_position[Y_AXIS] > y_max)
        {
          current_position[Y_AXIS] = y_max;
        }
        RTS_line_to_current(Y_AXIS);
        RTS_SndData(10 * current_position[Y_AXIS], AXIS_Y_COORD_VP);
        delay(1);
        RTS_SndData(0, MOTOR_FREE_ICON_VP);
        waitway = 0;
      }
      else
      {
        RTS_SndData(10 * current_position[Y_AXIS], AXIS_Y_COORD_VP);
      }
      break;

    case ZaxismoveKey:
      if(!planner.has_blocks_queued())
      {
        float z_min, z_max;
        
        z_min = Z_MIN_POS;
        z_max = Z_MAX_POS;
        if (false == g_uiZAxisEnable) { // 如果电机XYZ轴没有使能就先归零
          g_uiZAxisEnable = true;
          g_uiXYAxisEnable = true;
          gcode.process_subcommands_now_P(PSTR("G28"));
          if (g_uiXYAxisEnable) {
            RTS_SndData(0, MOTOR_FREE_ICON_VP);
          }
        }
        current_position[Z_AXIS] = ((float)recdat.data[0]) / 10;
        if(current_position[Z_AXIS] < z_min)
        {
          current_position[Z_AXIS] = z_min;
        }
        else if(current_position[Z_AXIS] > z_max)
        {
          current_position[Z_AXIS] = z_max;
        }
        RTS_line_to_current(Z_AXIS);
        RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);
        delay(1);
        RTS_SndData(0, MOTOR_FREE_ICON_VP);
        waitway = 0;
      }
      else
      {
        RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);
      }
      break;

    case HeaterLoadEnterKey:
      if(!planner.has_blocks_queued())
      {
        queue.enqueue_now_P(PSTR("G92 E0"));
        FilamentLOAD = ((float)recdat.data[0]) / 10;
        RTS_SndData(10 * FilamentLOAD, HEAD_FILAMENT_LOAD_DATA_VP);
        #if ENABLED(FILAMENT_RUNOUT_SENSOR)
          if((1 == READ(FIL_RUNOUT_PIN)) && (runout.enabled == true))
          {
            RTS_SndData(ExchangePageBase + 46, ExchangepageAddr);
            change_page_font = 46;
            RTS_SndData(0, HEAD_FILAMENT_LOAD_DATA_VP);
            break;
          }
        #endif
        current_position[E_AXIS] += FilamentLOAD;

        if((thermalManager.temp_hotend[0].target > EXTRUDE_MINTEMP) && (thermalManager.temp_hotend[0].celsius < (thermalManager.temp_hotend[0].celsius - 5)))
        {
          thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
          RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
        }
        else if((thermalManager.temp_hotend[0].target < EXTRUDE_MINTEMP) && (thermalManager.temp_hotend[0].celsius < (ChangeFilamentTemp - 5)))
        {
          thermalManager.setTargetHotend(ChangeFilamentTemp, 0);
          RTS_SndData(ChangeFilamentTemp, HEAD_SET_TEMP_VP);
        }
        g_heaterLoadTempAdd = true;
        while(ABS(thermalManager.degHotend(0) - thermalManager.degTargetHotend(0)) > TEMP_WINDOW)
        {
          idle();
        }
        g_heaterLoadTempAdd = false;
        RTS_line_to_current(E_AXIS);
        RTS_SndData(10 * FilamentLOAD, HEAD_FILAMENT_LOAD_DATA_VP);
        planner.synchronize();
      }
      else
      {
        RTS_SndData(0, HEAD_FILAMENT_LOAD_DATA_VP);
      }
      break;

    case HeaterUnLoadEnterKey:
      if(!planner.has_blocks_queued())
      {
        queue.enqueue_now_P(PSTR("G92 E0"));
        FilamentUnLOAD = ((float)recdat.data[0]) / 10;
        RTS_SndData(10 * FilamentUnLOAD, HEAD_FILAMENT_UNLOAD_DATA_VP);
        #if ENABLED(FILAMENT_RUNOUT_SENSOR)
          if((1 == READ(FIL_RUNOUT_PIN)) && (runout.enabled == true))
          {
            RTS_SndData(ExchangePageBase + 46, ExchangepageAddr);
            change_page_font = 46;
            RTS_SndData(0, HEAD_FILAMENT_UNLOAD_DATA_VP);
            break;
          }
        #endif
        current_position[E_AXIS] -= FilamentUnLOAD;

        if((thermalManager.temp_hotend[0].target > EXTRUDE_MINTEMP) && (thermalManager.temp_hotend[0].celsius < (thermalManager.temp_hotend[0].celsius - 5)))
        {
          thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
          RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
        }
        else if((thermalManager.temp_hotend[0].target < EXTRUDE_MINTEMP) && (thermalManager.temp_hotend[0].celsius < (ChangeFilamentTemp - 5)))
        {
          thermalManager.setTargetHotend(ChangeFilamentTemp, 0);
          RTS_SndData(ChangeFilamentTemp, HEAD_SET_TEMP_VP);
        }
        g_heaterLoadTempAdd = true;
        while(ABS(thermalManager.degHotend(0) - thermalManager.degTargetHotend(0)) > TEMP_WINDOW)
        {
          idle();
        }
        g_heaterLoadTempAdd = false;
        RTS_line_to_current(E_AXIS);
        RTS_SndData(10 * FilamentUnLOAD, HEAD_FILAMENT_UNLOAD_DATA_VP);
        planner.synchronize();
      }
      else
      {
        RTS_SndData(0, HEAD_FILAMENT_UNLOAD_DATA_VP);
      }
      break;

    case HeaterLoadStartKey:
      if(recdat.data[0] == 1)
      {
        if(!planner.has_blocks_queued())
        {
          #if ENABLED(FILAMENT_RUNOUT_SENSOR)
            if((1 == READ(FIL_RUNOUT_PIN)) && (runout.enabled == true))
            {
              RTS_SndData(ExchangePageBase + 46, ExchangepageAddr);
              change_page_font = 46;
              break;
            }
          #endif

          if((thermalManager.temp_hotend[0].target > EXTRUDE_MINTEMP) && (thermalManager.temp_hotend[0].celsius < (thermalManager.temp_hotend[0].celsius - 5)))
          {
            thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
            RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
            break;
          }
          else if((thermalManager.temp_hotend[0].target < EXTRUDE_MINTEMP) && (thermalManager.temp_hotend[0].celsius < (ChangeFilamentTemp - 5)))
          {
            thermalManager.setTargetHotend(ChangeFilamentTemp, 0);
            RTS_SndData(ChangeFilamentTemp, HEAD_SET_TEMP_VP);
            break;
          }
          else
          {
            RTS_line_to_current(E_AXIS);
            planner.synchronize();
          }
          RTS_SndData(ExchangePageBase + 19, ExchangepageAddr);
          change_page_font = 19;
        }
      }
      else if(recdat.data[0] == 2)
      {
        if(!planner.has_blocks_queued())
        {
          RTS_SndData(ExchangePageBase + 19, ExchangepageAddr);
          change_page_font = 19;
        }
      }
      else if(recdat.data[0] == 3)
      {
        queue.enqueue_now_P(PSTR("G92 E0")); //rock_20210927 liuxu  Reset Extruder
        RTS_SndData(ExchangePageBase + 19, ExchangepageAddr);
        change_page_font = 19;
        RTS_SndData(0, HEAD_FILAMENT_LOAD_DATA_VP);
        RTS_SndData(0, HEAD_FILAMENT_UNLOAD_DATA_VP);
      }
      break;

    case SelectLanguageKey:
      if(recdat.data[0] != 0)
      {
        lang = recdat.data[0];
      }
      language_change_font = lang;
      for(int i = 0;i < 9;i ++)
      {
        RTS_SndData(0, LANGUAGE_CHINESE_TITLE_VP + i);
      }
      RTS_SndData(1, LANGUAGE_CHINESE_TITLE_VP + (language_change_font - 1));
      languagedisplayUpdate();
      // settings.save();
      break;

    case PowerContinuePrintKey:
      if(recdat.data[0] == 1)
      {
        if((recovery.info.recovery_flag == true) && (PoweroffContinue == true))
        {
          PoweroffContinue = true;
          power_off_type_yes = 1;
          Update_Time_Value = 0;
          RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;
          //  加载预览图
          gcodePicDispalyOnOff(DEFAULT_PRINT_MODEL_VP, false);
          int32_t ret = gcodePicDataSendToDwin(recovery.info.sd_filename,VP_OVERLAY_PIC_PTINT,PIC_FORMAT_JPG, PIC_RESOLITION_300_300);
          if (ret == PIC_OK) { // 文件中有预览图
            gcodePicDispalyOnOff(DEFAULT_PRINT_MODEL_VP, false);
          } else { // 文件无预览图
            gcodePicDispalyOnOff(DEFAULT_PRINT_MODEL_VP, true);
          }
          // recovery.resume();
          queue.enqueue_now_P(PSTR("M1000"));

          sdcard_pause_check = true;
          zprobe_zoffset = probe.offset.z;
          RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
          RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
          g_cloudPLRStatusValue = CLOUD_PLR_YES_STATE;
          // 3:cloud print resume
          SERIAL_ECHOLN("M79 S3");
        }
        else if(PoweroffContinue == false)
        {
          SERIAL_ECHOLN("M79 S3");
          queue.enqueue_now_P(PSTR("M79 S3"));   // 3:cloud print resume
        }
      }
      else if(recdat.data[0] == 2)
      {
        if(PoweroffContinue == true)
        {
          Update_Time_Value = RTS_UPDATE_VALUE;
          RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
          gcodePicDispalyOnOff(DEFAULT_PRINT_MODEL_VP, true);
          change_page_font = 1;

          RTS_SndData(0, PRINT_TIME_HOUR_VP);
          RTS_SndData(0, PRINT_TIME_MIN_VP);
          Update_Time_Value = 0;
          RTS_SDcard_Stop();
          g_cloudPLRStatusValue = CLOUD_PLR_NULL_STATE;
        }
        else if(PoweroffContinue == false)
        {
          SERIAL_ECHOLN("M79 S4");
          queue.enqueue_now_P(PSTR("M79 S4"));   // 4:cloud print stop
        }
      }
      break;

    case PLAHeadSetEnterKey:
      ui.material_preset[0].hotend_temp = recdat.data[0];
      RTS_SndData(ui.material_preset[0].hotend_temp, PREHEAT_PLA_SET_NOZZLE_TEMP_VP);
      break;

    case PLABedSetEnterKey:
      ui.material_preset[0].bed_temp = recdat.data[0];
      RTS_SndData(ui.material_preset[0].bed_temp, PREHEAT_PLA_SET_BED_TEMP_VP);
      break;

    case ABSHeadSetEnterKey:
      ui.material_preset[1].hotend_temp = recdat.data[0];
      RTS_SndData(ui.material_preset[1].hotend_temp, PREHEAT_ABS_SET_NOZZLE_TEMP_VP);
      break;

    case ABSBedSetEnterKey:
      ui.material_preset[1].bed_temp = recdat.data[0];
      RTS_SndData(ui.material_preset[1].bed_temp, PREHEAT_ABS_SET_BED_TEMP_VP);
      break;

    case StoreMemoryKey:
      if(recdat.data[0] == 1)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 37, ExchangepageAddr);
        change_page_font = 37;
      }
      if(recdat.data[0] == 2)
      {
        // queue.enqueue_now_P(PSTR("M502"));
        language_change_font = lang = 2;
        languagedisplayUpdate();
        for(int i = 0;i < 9;i ++)
        {
          RTS_SndData(0, LANGUAGE_CHINESE_TITLE_VP + i);
        }
        RTS_SndData(1, LANGUAGE_CHINESE_TITLE_VP + (language_change_font - 1));
        last_zoffset = zprobe_zoffset = probe.offset.z = 0;
        RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
        rtscheck.RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
        change_page_font = 33;
        // settings.save();
        ui.material_preset[0].hotend_temp=PREHEAT_1_TEMP_HOTEND;
        ui.material_preset[0].bed_temp=PREHEAT_1_TEMP_BED;
        ui.material_preset[1].hotend_temp=PREHEAT_2_TEMP_HOTEND;
        ui.material_preset[1].bed_temp=PREHEAT_2_TEMP_BED;
        RTS_SndData(ui.material_preset[0].hotend_temp, PREHEAT_PLA_SET_NOZZLE_TEMP_VP);
        RTS_SndData(ui.material_preset[0].bed_temp, PREHEAT_PLA_SET_BED_TEMP_VP);
        RTS_SndData(ui.material_preset[1].hotend_temp, PREHEAT_ABS_SET_NOZZLE_TEMP_VP);
        RTS_SndData(ui.material_preset[1].bed_temp, PREHEAT_ABS_SET_BED_TEMP_VP);
        delay(20);

        RTS_SndData(default_max_feedrate[X_AXIS], MAX_VELOCITY_XAXIS_DATA_VP);
        RTS_SndData(default_max_feedrate[Y_AXIS], MAX_VELOCITY_YAXIS_DATA_VP);
        RTS_SndData(default_max_feedrate[Z_AXIS], MAX_VELOCITY_ZAXIS_DATA_VP);
        RTS_SndData(default_max_feedrate[E_AXIS], MAX_VELOCITY_EAXIS_DATA_VP);
        delay(20);

        RTS_SndData(default_max_acceleration[X_AXIS], MAX_ACCEL_XAXIS_DATA_VP);
        RTS_SndData(default_max_acceleration[Y_AXIS], MAX_ACCEL_YAXIS_DATA_VP);
        RTS_SndData(default_max_acceleration[Z_AXIS], MAX_ACCEL_ZAXIS_DATA_VP);
        RTS_SndData(default_max_acceleration[E_AXIS], MAX_ACCEL_EAXIS_DATA_VP);
        delay(20);

        RTS_SndData(default_max_jerk[X_AXIS] * 100, MAX_JERK_XAXIS_DATA_VP);
        RTS_SndData(default_max_jerk[Y_AXIS] * 100, MAX_JERK_YAXIS_DATA_VP);
        RTS_SndData(default_max_jerk[Z_AXIS] * 100, MAX_JERK_ZAXIS_DATA_VP);
        RTS_SndData(default_max_jerk[E_AXIS] * 100, MAX_JERK_EAXIS_DATA_VP);
        delay(20);

        RTS_SndData(default_axis_steps_per_unit[X_AXIS] * 10, MAX_STEPSMM_XAXIS_DATA_VP);
        RTS_SndData(default_axis_steps_per_unit[Y_AXIS] * 10, MAX_STEPSMM_YAXIS_DATA_VP);
        RTS_SndData(default_axis_steps_per_unit[Z_AXIS] * 10, MAX_STEPSMM_ZAXIS_DATA_VP);
        RTS_SndData(default_axis_steps_per_unit[E_AXIS] * 10, MAX_STEPSMM_EAXIS_DATA_VP);
        delay(20);

        RTS_SndData(default_nozzle_ptemp * 100, NOZZLE_TEMP_P_DATA_VP);
        RTS_SndData(default_nozzle_itemp * 100, NOZZLE_TEMP_I_DATA_VP);
        RTS_SndData(default_nozzle_dtemp * 100, NOZZLE_TEMP_D_DATA_VP);
        delay(20);
        RTS_SndData(default_hotbed_ptemp * 100, HOTBED_TEMP_P_DATA_VP);
        RTS_SndData(default_hotbed_itemp * 100, HOTBED_TEMP_I_DATA_VP);
        RTS_SndData(default_hotbed_dtemp * 10, HOTBED_TEMP_D_DATA_VP);
        delay(20);
        settings.reset();  //rock_20220816
        delay(50);
        settings.save();
        delay(200);
      }
      else if(recdat.data[0] == 3)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
        change_page_font = 33;
      }
      else if(recdat.data[0] == 4)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 34, ExchangepageAddr);
        change_page_font = 34;
      }
      else if(recdat.data[0] == 5)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 39, ExchangepageAddr);
        change_page_font = 39;
      }
      else if(recdat.data[0] == 6)
      {
        if(wifi_enable_flag)
        {
          wifi_enable_flag = 0;
          queue.inject_P(PSTR("M115"));
          RTS_SndData(1, ADV_SETTING_WIFI_ICON_VP);
          RTS_SndData(0, WIFI_CONNECTED_DISPLAY_ICON_VP);
          settings.save();
        }
        else
        {
          wifi_enable_flag = 1;
          queue.inject_P(PSTR("M115"));
          RTS_SndData(0, ADV_SETTING_WIFI_ICON_VP);
          RTS_SndData(1, WIFI_CONNECTED_DISPLAY_ICON_VP);
          settings.save();
        }
      }
      else if(recdat.data[0] == 7)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 38, ExchangepageAddr);
        change_page_font = 38;
      }
      else if(recdat.data[0] == 8)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 36, ExchangepageAddr);
        change_page_font = 36;
      }
      else if(recdat.data[0] == 9)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 37, ExchangepageAddr);
        change_page_font = 37;
      }
      else if(recdat.data[0] == 0x0A)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 35, ExchangepageAddr);
        change_page_font = 35;
      }
      else if(recdat.data[0] == 0x0B)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
        change_page_font = 33;
        settings.save();
        delay(1000);
      }
      else if(recdat.data[0] == 0x0C)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
        change_page_font = 33;
        settings.save();
        delay(1000);
      } else if(recdat.data[0] == 0x0D)
      {
        RTS_SndData(0, HEAD_SET_TEMP_VP);
        RTS_SndData(0, BED_SET_TEMP_VP);
        rtscheck.RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
        change_page_font = 33;
        g_uiAutoPIDFlag = false;
        // settings.save();
        // delay(1000);
      } else if(recdat.data[0] == 0x0E)
      {
        if( g_uiAutoPIDRuningFlag == true) break;
        gcode.process_subcommands_now_P(PSTR("M107"));
        rtscheck.RTS_SndData(ExchangePageBase + 48, ExchangepageAddr);
        change_page_font = 48;
        RTS_SndData(lang, AUTO_PID_HOTBED_TIS_VP);
        RTS_SndData(0, AUTO_PID_RUN_TIS_VP);
        RTS_SndData(lang, AUTO_PID_NOZZLE_TIS_VP);
      } else if(recdat.data[0] == 0x0F)
      {
        gcode.process_subcommands_now_P(PSTR("M106"));
        rtscheck.RTS_SndData(0, WRITE_CURVE_DDR_CMD);
        rtscheck.RTS_SndData(ExchangePageBase + 50, ExchangepageAddr);
        change_page_font = 50;
      } else if(recdat.data[0] == 0x10)
      {
        
        rtscheck.RTS_SndData(0, WRITE_CURVE_DDR_CMD);
        rtscheck.RTS_SndData(ExchangePageBase + 49, ExchangepageAddr);
        change_page_font = 49;
      }
      break;

    case FanSpeedEnterKey:
      thermalManager.fan_speed[0] = recdat.data[0];
      RTS_SndData(thermalManager.fan_speed[0], FAN_SPEED_CONTROL_DATA_VP);
      break;

    case VelocityXaxisEnterKey:
      float velocity_xaxis;
      velocity_xaxis = planner.settings.max_feedrate_mm_s[0];
      velocity_xaxis = recdat.data[0];
      RTS_SndData(velocity_xaxis, MAX_VELOCITY_XAXIS_DATA_VP);
      planner.set_max_feedrate(X_AXIS, velocity_xaxis);
      break;

    case VelocityYaxisEnterKey:
      float velocity_yaxis;
      velocity_yaxis = planner.settings.max_feedrate_mm_s[1];
      velocity_yaxis = recdat.data[0];
      RTS_SndData(velocity_yaxis, MAX_VELOCITY_YAXIS_DATA_VP);
      planner.set_max_feedrate(Y_AXIS, velocity_yaxis);
      break;

    case VelocityZaxisEnterKey:
      float velocity_zaxis;
      velocity_zaxis = planner.settings.max_feedrate_mm_s[2];
      velocity_zaxis = recdat.data[0];
      RTS_SndData(velocity_zaxis, MAX_VELOCITY_ZAXIS_DATA_VP);
      planner.set_max_feedrate(Z_AXIS, velocity_zaxis);
      break;

    case VelocityEaxisEnterKey:
      float velocity_eaxis;
      velocity_eaxis = planner.settings.max_feedrate_mm_s[3];
      velocity_eaxis = recdat.data[0];
      RTS_SndData(velocity_eaxis, MAX_VELOCITY_EAXIS_DATA_VP);
      planner.set_max_feedrate(E_AXIS, velocity_eaxis);
      break;

    case AccelXaxisEnterKey:
      float accel_xaxis;
      accel_xaxis = planner.settings.max_acceleration_mm_per_s2[0];
      accel_xaxis = recdat.data[0];
      RTS_SndData(accel_xaxis, MAX_ACCEL_XAXIS_DATA_VP);
      planner.set_max_acceleration(X_AXIS, accel_xaxis);
      break;

    case AccelYaxisEnterKey:
      float accel_yaxis;
      accel_yaxis = planner.settings.max_acceleration_mm_per_s2[1];
      accel_yaxis = recdat.data[0];
      RTS_SndData(accel_yaxis, MAX_ACCEL_YAXIS_DATA_VP);
      planner.set_max_acceleration(Y_AXIS, accel_yaxis);
      break;

    case AccelZaxisEnterKey:
      float accel_zaxis;
      accel_zaxis = planner.settings.max_acceleration_mm_per_s2[2];
      accel_zaxis = recdat.data[0];
      RTS_SndData(accel_zaxis, MAX_ACCEL_ZAXIS_DATA_VP);
      planner.set_max_acceleration(Z_AXIS, accel_zaxis);
      break;

    case AccelEaxisEnterKey:
      float accel_eaxis;
      accel_eaxis = planner.settings.max_acceleration_mm_per_s2[3];
      accel_eaxis = recdat.data[0];
      RTS_SndData(accel_eaxis, MAX_ACCEL_EAXIS_DATA_VP);
      planner.set_max_acceleration(E_AXIS, accel_eaxis);
      break;

    case JerkXaxisEnterKey:
      float jerk_xaxis;
      jerk_xaxis = planner.max_jerk.x;
      jerk_xaxis = (float)recdat.data[0] / 100;
      RTS_SndData(jerk_xaxis * 100, MAX_JERK_XAXIS_DATA_VP);
      planner.set_max_jerk(X_AXIS, jerk_xaxis);
      break;

    case JerkYaxisEnterKey:
      float jerk_yaxis;
      jerk_yaxis = planner.max_jerk.y;
      jerk_yaxis = (float)recdat.data[0] / 100;
      RTS_SndData(jerk_yaxis * 100, MAX_JERK_YAXIS_DATA_VP);
      planner.set_max_jerk(Y_AXIS, jerk_yaxis);
      break;

    case JerkZaxisEnterKey:
      float jerk_zaxis;
      jerk_zaxis = planner.max_jerk.z;
      jerk_zaxis = (float)recdat.data[0] / 100;
      RTS_SndData(jerk_zaxis * 100, MAX_JERK_ZAXIS_DATA_VP);
      planner.set_max_jerk(Z_AXIS, jerk_zaxis);
      break;

    case JerkEaxisEnterKey:
      float jerk_eaxis;
      jerk_eaxis = planner.max_jerk.e;
      jerk_eaxis = (float)recdat.data[0] / 100;
      RTS_SndData(jerk_eaxis * 100, MAX_JERK_EAXIS_DATA_VP);
      planner.set_max_jerk(E_AXIS, jerk_eaxis);
      break;

    case StepsmmXaxisEnterKey:
      float stepsmm_xaxis;
      stepsmm_xaxis = planner.settings.axis_steps_per_mm[0];
      stepsmm_xaxis = (float)recdat.data[0] / 10;
      RTS_SndData(stepsmm_xaxis * 10, MAX_STEPSMM_XAXIS_DATA_VP);
      planner.settings.axis_steps_per_mm[X_AXIS] = stepsmm_xaxis;
      break;

    case StepsmmYaxisEnterKey:
      float stepsmm_yaxis;
      stepsmm_yaxis = planner.settings.axis_steps_per_mm[1];
      stepsmm_yaxis = (float)recdat.data[0] / 10;
      RTS_SndData(stepsmm_yaxis * 10, MAX_STEPSMM_YAXIS_DATA_VP);
      planner.settings.axis_steps_per_mm[Y_AXIS] = stepsmm_yaxis;
      break;

    case StepsmmZaxisEnterKey:
      float stepsmm_zaxis;
      stepsmm_zaxis = planner.settings.axis_steps_per_mm[2];
      stepsmm_zaxis = (float)recdat.data[0] / 10;
      RTS_SndData(stepsmm_zaxis * 10, MAX_STEPSMM_ZAXIS_DATA_VP);
      planner.settings.axis_steps_per_mm[Z_AXIS] = stepsmm_zaxis;
      break;

    case StepsmmEaxisEnterKey:
      float stepsmm_eaxis;
      stepsmm_eaxis = planner.settings.axis_steps_per_mm[3];
      stepsmm_eaxis = (float)recdat.data[0] / 10;
      RTS_SndData(stepsmm_eaxis * 10, MAX_STEPSMM_EAXIS_DATA_VP);
      planner.settings.axis_steps_per_mm[E_AXIS] = stepsmm_eaxis;
      break;

    case NozzlePTempEnterKey:
      float nozzle_ptemp;
      nozzle_ptemp = (float)recdat.data[0] / 100;
      RTS_SndData(nozzle_ptemp * 100, NOZZLE_TEMP_P_DATA_VP);
      PID_PARAM(Kp, 0) = nozzle_ptemp;
      break;

    case NozzleITempEnterKey:
      float nozzle_itemp;
      nozzle_itemp = (float)recdat.data[0] / 100;
      RTS_SndData(nozzle_itemp * 100, NOZZLE_TEMP_I_DATA_VP);
      PID_PARAM(Ki, 0) = scalePID_i(nozzle_itemp);
      break;

    case NozzleDTempEnterKey:
      float nozzle_dtemp;
      nozzle_dtemp = (float)recdat.data[0] / 100;
      RTS_SndData(nozzle_dtemp * 100, NOZZLE_TEMP_D_DATA_VP);
      PID_PARAM(Kd, 0) = scalePID_d(nozzle_dtemp);
      break;

    case HotbedPTempEnterKey:
      float hotbed_ptemp;
      hotbed_ptemp = (float)recdat.data[0] / 100;
      RTS_SndData(hotbed_ptemp * 100, HOTBED_TEMP_P_DATA_VP);
      thermalManager.temp_bed.pid.Kp = hotbed_ptemp;
      break;

    case HotbedITempEnterKey:
      float hotbed_itemp;
      hotbed_itemp = (float)recdat.data[0] / 100;
      RTS_SndData(hotbed_itemp * 100, HOTBED_TEMP_I_DATA_VP);
      thermalManager.temp_bed.pid.Ki = scalePID_i(hotbed_itemp);
      break;

    case HotbedDTempEnterKey:
      float hotbed_dtemp;
      hotbed_dtemp = (float)recdat.data[0] / 10;
      RTS_SndData(hotbed_dtemp * 10, HOTBED_TEMP_D_DATA_VP);
      thermalManager.temp_bed.pid.Kd = scalePID_d(hotbed_dtemp);
      break;

    case SelectFileKey:
      if (RTS_SD_Detected())
      {
        if (recdat.data[0] > CardRecbuf.Filesum)
        {
          break;
        }
        CardRecbuf.selecFlag = true;
        CardRecbuf.recordcount = recdat.data[0] - 1;

        for(int j = 0; j < 20; j ++)
        {
          RTS_SndData(0, SELECT_FILE_TEXT_VP + j);
        }
        delay(2);
        for(int j = 1;j <= CardRecbuf.Filesum;j ++)
        {
          RTS_SndData((unsigned long)0x073F, FilenameNature + j * 16);
          RTS_SndData(0, FILE1_SELECT_ICON_VP - 1 + j);
        }
        RTS_SndData((unsigned long)0xFFFF, FilenameNature + recdat.data[0] * 16);
        RTS_SndData(1, FILE1_SELECT_ICON_VP + (recdat.data[0] - 1));
      }

      char ret;
      //  加载预览图
      RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
      change_page_font = 1;
      gcodePicDispalyOnOff(DEFAULT_PRINT_MODEL_VP, false);
      ret = gcodePicDataSendToDwin(CardRecbuf.Cardfilename[CardRecbuf.recordcount],VP_OVERLAY_PIC_PTINT,PIC_FORMAT_JPG, PIC_RESOLITION_300_300);
      if (ret == PIC_OK) { // 文件中有预览图
        gcodePicDispalyOnOff(DEFAULT_PRINT_MODEL_VP, false);
      } else { // 文件无预览图
        gcodePicDispalyOnOff(DEFAULT_PRINT_MODEL_VP, true);
      }
      
      delay(20);
      RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], SELECT_FILE_TEXT_VP);
      
      RefreshBrightnessAtPrint(0);
      break;

    case StartFileKey:
      if((recdat.data[0] == 1) && RTS_SD_Detected())
      {
        if((CardRecbuf.recordcount < 0) || (false == CardRecbuf.selecFlag))
        {
          break;
        }

        if(CardReader::flag.mounted)
        {
          char cmd[30];
          char *c;
          sprintf_P(cmd, PSTR("M23 %s"), CardRecbuf.Cardfilename[CardRecbuf.recordcount]);
          for (c = &cmd[4]; *c; c++)
            *c = tolower(*c);

          memset(cmdbuf, 0, sizeof(cmdbuf));
          strcpy(cmdbuf, cmd);
          #if ENABLED(FILAMENT_RUNOUT_SENSOR)
            if((1 == READ(FIL_RUNOUT_PIN)) && (runout.enabled == true))
            {
              RTS_SndData(ExchangePageBase + 7, ExchangepageAddr);
              change_page_font = 7;
              sdcard_pause_check = false;
              break;
            }
          #endif
          PoweroffContinue = true;
          queue.enqueue_one_now(cmd);
          delay(20);
          queue.enqueue_now_P(PSTR("M24"));
          // clean screen.
          for (int j = 0; j < 20; j ++)
          {
            RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
          }

          RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], PRINT_FILE_TEXT_VP);

          delay(2);
          // SERIAL_ECHO_MSG("\r\nrock_20220411_check_filament\r\n");
          wait_for_heatup = wait_for_user = false;
          #if ENABLED(BABYSTEPPING)
            RTS_SndData(0, AUTO_BED_LEVEL_ZOFFSET_VP);
          #endif
          feedrate_percentage = 100;
          RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
          RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
          RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          queue.enqueue_now_P(PSTR("G28 X Y"));// 解决选中文件打印加热烫平台的问题
          Update_Time_Value = 0;
          // 1:cloud print satrt
          SERIAL_ECHOLN("M79 S1");
        }
        else
        {
          break;
        }
      }
      else if(recdat.data[0] == 2)
      {
        if(!planner.has_blocks_queued())
        {
          RTS_SndData(file_total_page, PRINT_COUNT_PAGE_DATA_VP);
          if((file_total_page > file_current_page) && (file_current_page < (MaxFileNumber / 4)))
          {
            file_current_page ++;
          }
          else
          {
            break;
          }
          RTS_SndData(file_current_page, PRINT_CURRENT_PAGE_DATA_VP);
          RTS_SndData(ExchangePageBase + 2, ExchangepageAddr);
          change_page_font = 2;

          if(CardReader::flag.mounted)
          {
            RTS_line_to_filelist();
          }
        }
      }
      else if(recdat.data[0] == 3)
      {
        if(!planner.has_blocks_queued())
        {
          RTS_SndData(file_total_page, PRINT_COUNT_PAGE_DATA_VP);
          if(file_current_page > 1)
          {
            file_current_page --;
          }
          else
          {
            break;
          }
          RTS_SndData(file_current_page, PRINT_CURRENT_PAGE_DATA_VP);
          RTS_SndData(ExchangePageBase + 2, ExchangepageAddr);
          change_page_font = 2;

          if(CardReader::flag.mounted)
          {
            RTS_line_to_filelist();
          }
        }
      }
      else if(recdat.data[0] == 4)
      {
        if(!planner.has_blocks_queued())
        {
          RTS_SndData(file_total_page, PRINT_COUNT_PAGE_DATA_VP);
          file_current_page = 1;
          RTS_SndData(file_current_page, PRINT_CURRENT_PAGE_DATA_VP);

          RTS_line_to_filelist();

          RTS_SndData(ExchangePageBase + 2, ExchangepageAddr);
          change_page_font = 2;
        }
      }
      else if(recdat.data[0] == 5)
      {
        if(!planner.has_blocks_queued())
        {
          RTS_SndData(file_total_page, PRINT_COUNT_PAGE_DATA_VP);
          file_current_page = file_total_page;
          RTS_SndData(file_current_page, PRINT_CURRENT_PAGE_DATA_VP);

          RTS_line_to_filelist();

          RTS_SndData(ExchangePageBase + 2, ExchangepageAddr);
          change_page_font = 2;
        }
      } else if (recdat.data[0] == 6) { // 喷嘴PID
        if( g_uiAutoPIDRuningFlag == true) break;
        char cmd[30];
        g_uiAutoPIDRuningFlag = true;
        g_uiAutoPIDRuningDiff = 1;
        g_uiCurveDataCnt = 0;
        RTS_SndData(lang + 10, AUTO_PID_START_VP);
        RTS_SndData(0, AUTO_PID_NOZZLE_TIS_VP);
        RTS_SndData(lang, AUTO_PID_RUN_TIS_VP);
        memset(cmd, 0, sizeof(cmd));
        sprintf_P(cmd, PSTR("M303 E0 C8 S%d"), g_autoPIDHeaterTempTarget);
        gcode.process_subcommands_now(cmd);
        PID_PARAM(Kp, 0) = g_autoPID.Kp;
        PID_PARAM(Ki, 0) = scalePID_i(g_autoPID.Ki);
        PID_PARAM(Kd, 0) = scalePID_d(g_autoPID.Kd);
        RTS_SndData(lang + 10, AUTO_PID_RUN_TIS_VP);
        RTS_SndData(PID_PARAM(Kp, 0) * 100, NOZZLE_TEMP_P_DATA_VP);
        RTS_SndData(unscalePID_i(PID_PARAM(Ki, 0)) * 100, NOZZLE_TEMP_I_DATA_VP);
        RTS_SndData(unscalePID_d(PID_PARAM(Kd, 0)) * 100, NOZZLE_TEMP_D_DATA_VP);
        HAL_watchdog_refresh();
        settings.save();
        delay(1000);
        g_uiAutoPIDRuningDiff = 0;
        RTS_SndData(lang, AUTO_PID_START_VP);
        g_uiAutoPIDRuningFlag = false;
      } else if (recdat.data[0] == 7) { // 热床PID
        if( g_uiAutoPIDRuningFlag == true) break;
        
        g_uiAutoPIDRuningFlag = true;
        g_uiAutoPIDRuningDiff = 2;
        g_uiCurveDataCnt = 0;
        char cmd[30];
        RTS_SndData(lang + 10, AUTO_PID_START_VP);
        RTS_SndData(0, AUTO_PID_HOTBED_TIS_VP);
        RTS_SndData(lang, AUTO_PID_RUN_TIS_VP);
        memset(cmd, 0, sizeof(cmd));
        sprintf_P(cmd, PSTR("M303 E-1 C8 S%d"), g_autoPIDHotBedTempTarget);
        gcode.process_subcommands_now(cmd);
        thermalManager.temp_bed.pid.Kp = g_autoPID.Kp;
        thermalManager.temp_bed.pid.Ki = scalePID_i(g_autoPID.Ki);
        thermalManager.temp_bed.pid.Kd = scalePID_d(g_autoPID.Kd);
        RTS_SndData(lang + 10, AUTO_PID_RUN_TIS_VP);
        RTS_SndData(thermalManager.temp_bed.pid.Kp * 100, HOTBED_TEMP_P_DATA_VP);
        RTS_SndData(unscalePID_i(thermalManager.temp_bed.pid.Ki) * 100, HOTBED_TEMP_I_DATA_VP);
        RTS_SndData(unscalePID_d(thermalManager.temp_bed.pid.Kd) * 10, HOTBED_TEMP_D_DATA_VP);
        HAL_watchdog_refresh();
        settings.save();
        delay(1000);
        g_uiAutoPIDRuningDiff = 0;
        RTS_SndData(lang, AUTO_PID_START_VP);
        g_uiAutoPIDRuningFlag = false;
      }
      break;

    case ChangePageKey:
      RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], PRINT_FILE_TEXT_VP);

      // represents to update file list
      if (CardUpdate && lcd_sd_status && IS_SD_INSERTED())
      {
        RTS_line_to_filelist();
        for(uint16_t i = 0;i < 5;i ++)
        {
          delay(1);
          RTS_SndData((unsigned long)0xFFFF, FilenameNature + (i + 1) * 16);
          RTS_SndData(0, FILE1_SELECT_ICON_VP + i);
        }
      }

      RTS_SndData(MACHINE_TYPE, MACHINE_TYPE_ABOUT_TEXT_VP);
      RTS_SndData(FIRMWARE_VERSION, FIREWARE_VERSION_ABOUT_TEXT_VP);
      // RTS_SndData(SCREEN_VERSION, PRINTER_DISPLAY_VERSION_TEXT_VP);
      RTS_SndData(HARDWARE_VERSION, HARDWARE_VERSION_ABOUT_TEXT_VP);
      RTS_SndData(PRINT_SIZE, PRINTER_PRINTSIZE_TEXT_VP);

      if(1 == lang)
      {
        RTS_SndData(CORP_WEBSITE_C, WEBSITE_ABOUT_TEXT_VP);
      }
      else
      {
        RTS_SndData(CORP_WEBSITE_E, WEBSITE_ABOUT_TEXT_VP);
      }

      if(thermalManager.fan_speed[0] == 0)
      {
        RTS_SndData(0, PRINTER_FANOPEN_TITLE_VP);
      }
      else
      {
        RTS_SndData(1, PRINTER_FANOPEN_TITLE_VP);
      }

      if(LEDStatus)
      {
        RTS_SndData(1, PRINTER_LEDOPEN_TITLE_VP);
      }
      else
      {
        RTS_SndData(0, PRINTER_LEDOPEN_TITLE_VP);
      }
      // Percentrecord = card.percentDone() + 1;
      // if (Percentrecord <= 100)
      // {
      //   rtscheck.RTS_SndData((unsigned char)Percentrecord, PRINT_PROCESS_ICON_VP);
      // }
      rtscheck.RTS_SndData((unsigned char)card.percentDone(), PRINT_PROCESS_VP);
      RefreshBrightnessAtPrint((unsigned char)card.percentDone());

      RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);

      RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
      RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
          #if ENABLED(BED_TEMP_COMP)     
          if (thermalManager.temp_bed.target > 65 && thermalManager.temp_bed.target <= 85)
              RTS_SndData(thermalManager.temp_bed.target-5, BED_SET_TEMP_VP);
          else if (thermalManager.temp_bed.target > 85 && thermalManager.temp_bed.target <= 127)
              RTS_SndData(thermalManager.temp_bed.target-7, BED_SET_TEMP_VP);
      #else
          RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      #endif
      // RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      languagedisplayUpdate();

      RTS_SndData(change_page_font + ExchangePageBase, ExchangepageAddr);
      break;

    case ErrorKey:
      {
        if(recdat.data[0] == 1)
        {
          if(printingIsActive())
          {
            RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
            change_page_font = 10;
          }
          else if(printingIsPaused())
          {
            RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
            change_page_font = 12;
          }
          else
          {
            RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
            change_page_font = 1;
            gcodePicDispalyOnOff(DEFAULT_PRINT_MODEL_VP, true);
          }

          if(errorway == 4)
          {
            // reboot
            NVIC_SystemReset();
          }
        }
      }
      break;

    default:
      break;
  }
  memset(&recdat, 0, sizeof(recdat));
  recdat.head[0] = FHONE;
  recdat.head[1] = FHTWO;
}

void EachMomentUpdate(void)
{
  millis_t ms = millis();

  if(flag_counter_wifireset)
  {
    millis_t ms_3 = millis();
    if(ms_3 > next_wifireset_update_ms)
    {
      next_wifireset_update_ms = ms_3 + 1000;
      wifiresetcount_ms++;
      rtscheck.RTS_SndData((TIME_WIFI_RESET_BACKPAGE - wifiresetcount_ms), WIFI_RESET_REMAIN_TIME_DATA_VP);
    }

    if(wifiresetcount_ms > TIME_WIFI_RESET_BACKPAGE)
    {
      flag_counter_wifireset = false;
      rtscheck.RTS_SndData(0, WIFI_CONNECTED_DISPLAY_ICON_VP);
      rtscheck.RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
      change_page_font = 33;
    }
  }
  else
  {
    wifiresetcount_ms = 0;
  }

  if(ms > next_rts_update_ms)
  {
    // print the file before the power is off.
    if((power_off_type_yes == 0) && lcd_sd_status && (recovery.info.recovery_flag == true))
    {
      rtscheck.RTS_SndData(ExchangePageBase, ExchangepageAddr);
      if(startprogress < 80)
      {
        rtscheck.RTS_SndData(startprogress, START_PROCESS_ICON_VP);
      }
      delay(30);
      if((startprogress += 1) > 80)
      {
        rtscheck.RTS_SndData(StartSoundSet, SoundAddr);
        power_off_type_yes = 1;

        fileCnt = card.get_num_Files();
        card.getWorkDirName();
        if(card.filename[0] != '/')
        {
          card.cdup();
        }

        for(uint16_t i = 0;(i < fileCnt) && (i < MaxFileNumber);i ++)
        {
          card.selectFileByIndex(fileCnt - 1 - i);
          char *pointFilename = card.longFilename;
          int filenamelen = strlen(card.longFilename);
          int j = 1;
          while((strncmp(&pointFilename[j], ".gcode", 6) && strncmp(&pointFilename[j], ".GCODE", 6)) && ((j ++) < filenamelen));

          for (int j = 0; j < 20; j ++)
          {
            rtscheck.RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
          }

          if (j >= TEXTBYTELEN)
          {
            strncpy(&card.longFilename[TEXTBYTELEN - 3], "..", 2);
            card.longFilename[TEXTBYTELEN - 1] = '\0';
            j = TEXTBYTELEN - 1;
          }

          strncpy(CardRecbuf.Cardshowfilename[i], card.longFilename, j);

          strcpy(CardRecbuf.Cardfilename[i], card.filename);
          if(!strcmp(CardRecbuf.Cardfilename[i], &recovery.info.sd_filename[1]))
          {
            rtscheck.RTS_SndData(CardRecbuf.Cardshowfilename[i], PRINT_FILE_TEXT_VP);
            break;
          }
        }
        rtscheck.RTS_SndData(ExchangePageBase + 27, ExchangepageAddr);
        change_page_font = 27;
        PoweroffContinue = true;
        g_cloudPLRStatusValue = CLOUD_PLR_CHOOSE_STATE; // 云打印断电续打的选择界面
        SERIAL_ECHOLN("M79 S6");   // 6:cloud print power continue
      }
      return;
    }
    else if((power_off_type_yes == 0) && (recovery.info.recovery_flag == false))
    {
      rtscheck.RTS_SndData(ExchangePageBase, ExchangepageAddr);
      if(startprogress < 80)
      {
        rtscheck.RTS_SndData(startprogress, START_PROCESS_ICON_VP);
      }
      delay(30);
      if((startprogress += 1) > 80)
      {
        rtscheck.RTS_SndData(StartSoundSet, SoundAddr);
        power_off_type_yes = 1;
        Update_Time_Value = RTS_UPDATE_VALUE;
        rtscheck.RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
        change_page_font = 1;
        gcodePicDispalyOnOff(DEFAULT_PRINT_MODEL_VP, true);
      }
      return;
    }
    else
    {
      static unsigned char last_cardpercentValue = 100;
      #if ENABLED(SDSUPPORT)
        if(PoweroffContinue == true)
        {
          duration_t elapsed = print_job_timer.duration();
          rtscheck.RTS_SndData(elapsed.value / 3600, PRINT_TIME_HOUR_VP);
          rtscheck.RTS_SndData((elapsed.value % 3600) / 60, PRINT_TIME_MIN_VP);
        }
      #endif

      if(card.isPrinting() && (last_cardpercentValue != card.percentDone()))
      {
        if((unsigned char) card.percentDone() > 0)
        {
          Percentrecord = card.percentDone();
          if(Percentrecord <= 100)
          {
            rtscheck.RTS_SndData((unsigned char)Percentrecord, PRINT_PROCESS_ICON_VP);
          }
        }
        else
        {
          rtscheck.RTS_SndData(0, PRINT_PROCESS_ICON_VP);
        }
        rtscheck.RTS_SndData((unsigned char)card.percentDone(), PRINT_PROCESS_VP);
        RefreshBrightnessAtPrint((unsigned char)card.percentDone());
        last_cardpercentValue = card.percentDone();
        rtscheck.RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);
      }

      if(pause_action_flag && (false == sdcard_pause_check) && printingIsPaused() && !planner.has_blocks_queued())
      {
        pause_action_flag = false;
        queue.enqueue_now_P(PSTR("G0 F3000 X220 Y220"));  //rock_20220704
        thermalManager.setTargetHotend(0, 0);
        rtscheck.RTS_SndData(0, HEAD_SET_TEMP_VP);
      }

      rtscheck.RTS_SndData(thermalManager.temp_hotend[0].celsius, HEAD_CURRENT_TEMP_VP);
      #if ENABLED(BED_TEMP_COMP)
      float bedTempDisp = thermalManager.temp_bed.celsius; 
      if (thermalManager.degTargetBed() > 65 && thermalManager.degTargetBed() <= 127)
      {
          if (bedTempDisp > 65 && bedTempDisp <= 86)
              bedTempDisp -= (5 * bedTempDisp) / thermalManager.degTargetBed();
          else if (bedTempDisp > 86 && bedTempDisp <= 127)
              bedTempDisp -= (7 * bedTempDisp) / thermalManager.degTargetBed();
      }
      rtscheck.RTS_SndData(bedTempDisp, BED_CURRENT_TEMP_VP);
      #else 
      rtscheck.RTS_SndData(thermalManager.temp_bed.celsius, BED_CURRENT_TEMP_VP);
      #endif
     

      #if ENABLED(SDSUPPORT)
        if((sd_printing_autopause == true) && (PoweroffContinue == true))
        {
          if(true == sdcard_pause_check)
          {
            rtscheck.RTS_SndData(0, CHANGE_SDCARD_ICON_VP);
            sdcard_pause_check = false;
          }
        }

        // if((false == sdcard_pause_check) && (!card.isPrinting()) && !planner.has_blocks_queued())
        if(HMI_lcd_flag.remove_card_flag)
        {
          // if(CardReader::flag.mounted)
          if(temp_remove_card_flag&&!planner.has_blocks_queued())
          {           
            queue.inject_P(PSTR("G1 F1200 X220 Y220"));
            temp_remove_card_flag=false;
            // #endif
            // queue.enqueue_now_P(PSTR("G28 X220 Y220"));                
          }
       
       
          
          // else 
          // {
            if(IS_SD_INSERTED()) //有卡
            {
              // SERIAL_ECHO_MSG("ROCK_MOVE_CARD333\r\n");
              rtscheck.RTS_SndData(1, CHANGE_SDCARD_ICON_VP);
            }
            else
            {
              rtscheck.RTS_SndData(0, CHANGE_SDCARD_ICON_VP);
            }
          // }
        
        }
      #endif

      if((last_target_temperature[0] != thermalManager.temp_hotend[0].target) || (last_target_temperature_bed != thermalManager.temp_bed.target))
      {
        thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
        thermalManager.setTargetBed(thermalManager.temp_bed.target);
        rtscheck.RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
              #if ENABLED(BED_TEMP_COMP)     
          if (thermalManager.temp_bed.target > 65 && thermalManager.temp_bed.target <= 85)
              rtscheck.RTS_SndData(thermalManager.temp_bed.target-5, BED_SET_TEMP_VP);
          else if (thermalManager.temp_bed.target > 85 && thermalManager.temp_bed.target <= 127)
              rtscheck.RTS_SndData(thermalManager.temp_bed.target-7, BED_SET_TEMP_VP);
      #else
          rtscheck.RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      #endif
        // rtscheck.RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
        last_target_temperature[0] = thermalManager.temp_hotend[0].target;
        last_target_temperature_bed = thermalManager.temp_bed.target;
      }

      if((thermalManager.temp_hotend[0].celsius >= thermalManager.temp_hotend[0].target) && (heatway == 1))
      {
        rtscheck.RTS_SndData(ExchangePageBase + 19, ExchangepageAddr);
        change_page_font = 19;
        heatway = 0;
        rtscheck.RTS_SndData(10 * FilamentLOAD, HEAD_FILAMENT_LOAD_DATA_VP);
        rtscheck.RTS_SndData(10 * FilamentUnLOAD, HEAD_FILAMENT_UNLOAD_DATA_VP);
      }

      #if ENABLED(FILAMENT_RUNOUT_SENSOR)
        if(1 == READ(FIL_RUNOUT_PIN))
        {
          rtscheck.RTS_SndData(0, FILAMENT_LOAD_ICON_VP);
        }
        else
        {
          rtscheck.RTS_SndData(1, FILAMENT_LOAD_ICON_VP);
        }
      #endif
    }
    next_rts_update_ms = ms + RTS_UPDATE_INTERVAL + Update_Time_Value;
  }
}

void RTSSHOW::languagedisplayUpdate(void)
{
  RTS_SndData(lang, MAIN_PAGE_BLUE_TITLE_VP);
  RTS_SndData(lang, SELECT_FILE_BLUE_TITLE_VP);
  RTS_SndData(lang, PREPARE_PAGE_BLUE_TITLE_VP);
  RTS_SndData(lang, SETTING_PAGE_BLUE_TITLE_VP);
  RTS_SndData(lang, MAIN_PAGE_BLACK_TITLE_VP);
  RTS_SndData(lang, SELECT_FILE_BLACK_TITLE_VP);
  RTS_SndData(lang, PREPARE_PAGE_BLACK_TITLE_VP);
  RTS_SndData(lang, SETTING_PAGE_BLACK_TITLE_VP);

  RTS_SndData(lang, PRINT_ADJUST_MENT_TITLE_VP);
  RTS_SndData(lang, PRINT_SPEED_TITLE_VP);
  RTS_SndData(lang, HEAD_SET_TITLE_VP);
  RTS_SndData(lang, BED_SET_TITLE_VP);
  RTS_SndData(lang, LEVEL_ZOFFSET_TITLE_VP);
  RTS_SndData(lang, FAN_CONTROL_TITLE_VP);
  RTS_SndData(lang, LED_CONTROL_TITLE_VP);

  RTS_SndData(lang, MOVE_AXIS_ENTER_GREY_TITLE_VP);
  RTS_SndData(lang, CHANGE_FILAMENT_GREY_TITLE_VP);
  RTS_SndData(lang, PREHAET_PAGE_GREY_TITLE_VP);
  RTS_SndData(lang, MOVE_AXIS_ENTER_BLACK_TITLE_VP);
  RTS_SndData(lang, CHANGE_FILAMENT_BLACK_TITLE_VP);
  RTS_SndData(lang, PREHAET_PAGE_BLACK_TITLE_VP);

  RTS_SndData(lang, PREHEAT_PLA_BUTTON_TITLE_VP);
  RTS_SndData(lang, PREHEAT_ABS_BUTTON_TITLE_VP);
  RTS_SndData(lang, COOL_DOWN_BUTTON_TITLE_VP);

  RTS_SndData(lang, FILAMENT_LOAD_BUTTON_TITLE_VP);
  RTS_SndData(lang, FILAMENT_UNLOAD_BUTTON_TITLE_VP);

  RTS_SndData(lang, LANGUAGE_SELECT_ENTER_VP);
  RTS_SndData(lang, FACTORY_DEFAULT_ENTER_TITLE_VP);
  RTS_SndData(lang, LEVELING_PAGE_TITLE_VP);

  RTS_SndData(lang, PRINTER_DEVICE_GREY_TITLE_VP);
  RTS_SndData(lang, PRINTER_ADVINFO_GREY_TITLE_VP);
  RTS_SndData(lang, PRINTER_INFO_ENTER_GREY_TITLE_VP);
  RTS_SndData(lang, PRINTER_DEVICE_BLACK_TITLE_VP);
  RTS_SndData(lang, PRINTER_ADVINFO_BLACK_TITLE_VP);
  RTS_SndData(lang, PRINTER_INFO_ENTER_BLACK_TITLE_VP);

  RTS_SndData(lang, PREHEAT_PLA_SET_TITLE_VP);
  RTS_SndData(lang, PREHEAT_ABS_SET_TITLE_VP);

  RTS_SndData(lang, STORE_MEMORY_CONFIRM_TITLE_VP);
  RTS_SndData(lang, STORE_MEMORY_CANCEL_TITLE_VP);

  RTS_SndData(lang, FILAMENT_UNLOAD_IGNORE_TITLE_VP);
  RTS_SndData(lang, FILAMENT_USEUP_TITLE_VP);
  RTS_SndData(lang, BUTTON_CHECK_CONFIRM_TITLE_VP);
  RTS_SndData(lang, BUTTON_CHECK_CANCEL_TITLE_VP);
  RTS_SndData(lang, FILAMENT_LOAD_TITLE_VP);
  RTS_SndData(lang, FILAMENT_LOAD_RESUME_TITLE_VP);
  RTS_SndData(lang, PAUSE_PRINT_POP_TITLE_VP);
  RTS_SndData(lang, STOP_PRINT_POP_TITLE_VP);
  RTS_SndData(lang, POWERCONTINUE_POP_TITLE_VP);
  RTS_SndData(lang, AUTO_HOME_WAITING_POP_TITLE_VP);

  RTS_SndData(0, BEDLEVELING_WAIT_TITLE_VP);
  RTS_SndData(lang, RESTORE_FACTORY_TITLE_VP);
  RTS_SndData(lang, RESET_WIFI_SETTING_TITLE_VP);
  RTS_SndData(lang, KILL_THERMAL_RUNAWAY_TITLE_VP);
  RTS_SndData(lang, KILL_HEATING_FAIL_TITLE_VP);
  RTS_SndData(lang, KILL_THERMISTOR_ERROR_TITLE_VP);
  RTS_SndData(lang, WIND_AUTO_SHUTDOWN_TITLE_VP);
  RTS_SndData(lang, RESET_WIFI_SETTING_BUTTON_VP);
  RTS_SndData(lang, PRINTER_AUTO_SHUTDOWN_TITLE_VP);
  RTS_SndData(lang, WIND_AUTO_SHUTDOWN_PAGE_VP);
  RTS_SndData(lang, AUTO_LEVELING_START_TITLE_VP);
  RTS_SndData(lang, AUX_LEVELING_GREY_TITLE_VP);
  RTS_SndData(lang, AUTO_LEVELING_GREY_TITLE_VP);
  RTS_SndData(lang, AUX_LEVELING_BLACK_TITLE_VP);
  RTS_SndData(lang, AUTO_LEVELING_BLACK_TITLE_VP);
  RTS_SndData(lang, LANGUAGE_SELECT_PAGE_TITLE_VP);
  RTS_SndData(lang, ADV_SETTING_MOTION_TITLE_VP);
  RTS_SndData(lang, ADV_SETTING_PID_TITLE_VP);
  RTS_SndData(lang, ADV_SETTING_WIFI_TITLE_VP);

  RTS_SndData(lang, MOTION_SETTING_TITLE_VP);
  RTS_SndData(lang, MOTION_SETTING_STEPSMM_TITLE_VP);
  RTS_SndData(lang, MOTION_SETTING_ACCEL_TITLE_VP);
  RTS_SndData(lang, MOTION_SETTING_JERK_TITLE_VP);
  RTS_SndData(lang, MOTION_SETTING_VELOCITY_TITLE_VP);

  RTS_SndData(lang, MAX_VELOCITY_SETTING_TITLE_VP);
  RTS_SndData(lang, MAX_VELOCITY_XAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_VELOCITY_YAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_VELOCITY_ZAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_VELOCITY_EAXIS_TITLE_VP);

  RTS_SndData(lang, MAX_ACCEL_SETTING_TITLE_VP);
  RTS_SndData(lang, MAX_ACCEL_XAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_ACCEL_YAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_ACCEL_ZAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_ACCEL_EAXIS_TITLE_VP);

  RTS_SndData(lang, MAX_JERK_SETTING_TITLE_VP);
  RTS_SndData(lang, MAX_JERK_XAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_JERK_YAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_JERK_ZAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_JERK_EAXIS_TITLE_VP);

  RTS_SndData(lang, MAX_STEPSMM_SETTING_TITLE_VP);
  RTS_SndData(lang, MAX_STEPSMM_XAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_STEPSMM_YAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_STEPSMM_ZAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_STEPSMM_EAXIS_TITLE_VP);

  RTS_SndData(lang, TEMP_PID_SETTING_TITLE_VP);
  RTS_SndData(lang, NOZZLE_TEMP_P_TITLE_VP);
  RTS_SndData(lang, NOZZLE_TEMP_I_TITLE_VP);
  RTS_SndData(lang, NOZZLE_TEMP_D_TITLE_VP);
  RTS_SndData(lang, HOTBED_TEMP_P_TITLE_VP);
  RTS_SndData(lang, HOTBED_TEMP_I_TITLE_VP);
  RTS_SndData(lang, HOTBED_TEMP_D_TITLE_VP);

  RTS_SndData(lang, FILAMENT_CONTROL_TITLE_VP);
  RTS_SndData(lang, POWERCONTINUE_CONTROL_TITLE_VP);

  RTS_SndData(lang, MACHINE_TYPE_ABOUT_CHAR_VP);
  RTS_SndData(lang, FIREWARE_VERSION_ABOUT_CHAR_VP);
  RTS_SndData(lang, PRINTER_DISPLAY_VERSION_TITLE_VP);
  RTS_SndData(lang, HARDWARE_VERSION_ABOUT_TITLE_VP);
  RTS_SndData(lang, WIFI_DN_CODE_CHAR_VP);
  RTS_SndData(lang, WEBSITE_ABOUT_CHAR_VP);
  RTS_SndData(lang, PRINTER_PRINTSIZE_TITLE_VP);
  RTS_SndData(lang, PLA_SETTINGS_TITLE_VP);
  RTS_SndData(lang, ABS_SETTINGS_TITLE_VP);
  RTS_SndData(lang, LEVELING_WAY_TITLE_VP);
  RTS_SndData(lang, SOUND_SETTING_VP);
  //自动PID
  RTS_SndData(lang, AUTO_PID_INLET_VP);
  RTS_SndData(lang, AUTO_PID_HOTBED_INLET_VP);
  RTS_SndData(lang, AUTO_PID_HOTBED_TIS_VP);
  RTS_SndData(lang, AUTO_PID_NOZZLE_INLET_VP);
  RTS_SndData(lang, AUTO_PID_NOZZLE_TIS_VP);
  RTS_SndData(lang, AUTO_PID_TIS_VP);
  RTS_SndData(lang, AUTO_PID_START_VP);
}

bool rts_test = false;
// looping at the loop function
void RTSUpdate(void)
{
  // Check the status of card
  if(!home_flag&&!G29_flag)//回零和调平中不检测卡
  rtscheck.RTS_SDCardUpate();

  EachMomentUpdate();
  // wait to receive massage and response
  if(rtscheck.RTS_RecData() > 0)
  {
    rtscheck.RTS_HandleData();
  }
  if(!wait_for_heatup&&!home_flag&&!G29_flag)Remove_card_window_check(); // rock_20220709
  HAL_watchdog_refresh();

  AutoUIBedNozzleHeightCali();//caixiaoliang 20210806调用喷嘴平台高度测量函数  
}

void RTS_PauseMoveAxisPage(void)
{
  if(waitway == 1)
  {
    rtscheck.RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
    change_page_font = 12;
    waitway = 0;
    SERIAL_ECHOLN("M79 S2");   // 2:cloud print pause
  }
  else if(waitway == 5)
  {
    rtscheck.RTS_SndData(ExchangePageBase + 7, ExchangepageAddr);
    change_page_font = 7;
    waitway = 0;
  }
  else if(waitway == 7)
  {
    // Click stop print
    rtscheck.RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
    change_page_font = 1;
    gcodePicDispalyOnOff(DEFAULT_PRINT_MODEL_VP, true);
    waitway = 0;
  }
}

void RTS_AutoBedLevelPage(void)
{
  if(waitway == 3)
  {
    rtscheck.RTS_SndData(0, BEDLEVELING_WAIT_TITLE_VP);
    rtscheck.RTS_SndData(ExchangePageBase + 26, ExchangepageAddr);
    change_page_font = 26;
    waitway = 0;
  }
}

void RTS_MoveAxisHoming(void)
{
  if(waitway == 4)
  {
    rtscheck.RTS_SndData(ExchangePageBase + 16 + (AxisUnitMode - 1), ExchangepageAddr);
    change_page_font = 16;
    waitway = 0;
  }
  else if(waitway == 6)
  {
    rtscheck.RTS_SndData(ExchangePageBase + 25, ExchangepageAddr);
    change_page_font = 25;
    waitway = 0;
  }
  else if(waitway == 7)
  {
    // Click Print finish
    rtscheck.RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
    change_page_font = 1;
    gcodePicDispalyOnOff(DEFAULT_PRINT_MODEL_VP, true);
    waitway = 0;
  }

  rtscheck.RTS_SndData(10 * current_position[X_AXIS], AXIS_X_COORD_VP);
  rtscheck.RTS_SndData(10 * current_position[Y_AXIS], AXIS_Y_COORD_VP);
  rtscheck.RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);
}

void RTS_CommandPause(void)
{
  if(printingIsActive())
  {
    rtscheck.RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
    change_page_font = 12;
    // card.pauseSDPrint();
    // print_job_timer.pause();
    // pause_action_flag = true;
  }
}

void ErrorHanding(void)
{
  // No more operations
  if(errorway == 1)
  {
    errorway = errornum = 0;
  }
  else if(errorway == 2)
  {
    // Z axis home failed
    home_errornum ++;
    if(home_errornum <= 3)
    {
      errorway = 0;
      waitway  = 4;
      queue.enqueue_now_P(PSTR("G28"));
      rtscheck.RTS_SndData(0, MOTOR_FREE_ICON_VP);
      Update_Time_Value = 0;
    }
    else
    {
      // After three failed returns to home, it will report the failure interface
      home_errornum = 0;
      errorway = 0;
      rtscheck.RTS_SndData(ExchangePageBase + 41, ExchangepageAddr);
      change_page_font = 41;
      // Z axis home failed
      rtscheck.RTS_SndData(Error_202, ABNORMAL_PAGE_TEXT_VP);

      if(printingIsActive())
      {
        rtscheck.RTS_SndData(0, PRINT_TIME_HOUR_VP);
        rtscheck.RTS_SndData(0, PRINT_TIME_MIN_VP);
        Update_Time_Value = 0;

        rtscheck.RTS_SDcard_Stop();
      }
    }
  }
  else if(errorway == 3)
  {
    // No more operations
    reset_bed_level();
    errorway = 0;
    errornum = 0;
  }
  else if(errorway == 4)
  {

  }
}

/**
 * [调平步骤--开始找限位开关到CRTouch的距离]
 * @Author Creality
 * @Time   2021-09-22
 */
bool RTSSHOW::LevelStepHeighMeasure(void)
{
 #ifdef BLTOUCH
    float coordinateOffset = VAULE_INVALIDE_8BIT;
    //if(gLcdSermoonV2UI.GetStaGoingHome() != GO_HOME_DONE) return false;
        
    /* set go-home status - 'GO_HOME_IDLE' */
    // SetStaGoingHome(GO_HOME_IDLE);  //此处用 home_flag 代替
    /* 启用了bltouch+下限位 */
    #if ENABLED(BLTOUCH_AND_Z_LIMIT)
        // constexpr xy_float_t test_offset_xy = {LEVEL_HEIGH_MEASURE_X, LEVEL_HEIGH_MEASURE_Y };
        // destination.set(test_offset_xy, current_position.z);
        // // destination.set(safe_homing_xy, current_position.z);
        // TERN_(1, destination -= probe.offset_xy);
        // if (position_is_reachable(destination)) {
        //     do_blocking_move_to_xy(destination);  // 把bltouch移动至中间
        // }
        // else {
        //     SERIAL_ECHO_MSG(STR_ZPROBE_OUT_SER);
        // }
        // 界面却换到“操作中”
        // lcdDispaly.GotoScreen(DGUSLCD_SCREEN_MSG_OPERATE);
        rtscheck.RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
        // 开始测量两坐标差值
        // coordinateOffset = homeaxis_bl(Z_AXIS, Z_MEASURE_FEEDRATE_FAST, Z_MEASURE_FEEDRATE_SLOW);
        if (coordinateOffset != VAULE_INVALIDE_8BIT) {
            rtscheck.SetzCoordinateOffset(coordinateOffset);
        } else {
            SERIAL_ECHOLN(" coordinateOffset err!!! ");
            // rtscheck.SetzCoordinateOffset(0.0);
        }
        settings.save();
    #endif

    /* 测量完成两坐标的差值后，喷头移动到检测点进行对高调节 */
    // MoveXYBlock((float)(LEVEL_HEIGH_MEASURE_X), (float)(LEVEL_HEIGH_MEASURE_Y));
    /* get to the base height, block */
    // MoveZBlock(0.0);

    // 显示Z轴补偿
    rtscheck.RTS_SndData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
    // dwinShow.RTS_SndData(probe.offset.z*1000, VP_DATA_Z_OFFSET);
    // uiShow.UI_ShowText(probe.offset.z, 3, lcdDispaly.current_screen, VP_DATA_Z_OFFSET);
 #endif

    return true;
}

void AutoUIBedNozzleHeightCali(void)
{
    if(st_bedNozzleHeightCal.bedNozzleHeightCalFinishFlag == false)
    {
      switch(st_bedNozzleHeightCal.bedNozzleHeightState)
      {
        case 0:
              st_bedNozzleHeightCal.isBedLeveling=true;
              st_bedNozzleHeightCal.goHomeSta=GO_HOME_IDLE;
              queue.enqueue_now_P(PSTR("M420 S0"));
              #ifdef BLTOUCH

                  queue.enqueue_now_P(PSTR("G28")); 
                  //queue.enqueue_now_P(PSTR("G0 Z0")); 
              #else
                  queue.inject_P(PSTR("G29 S1"));
              #endif  
              st_bedNozzleHeightCal.bedNozzleHeightState++;          
              break;
        case 1:
              /*wait for going home*/
              if(st_bedNozzleHeightCal.goHomeSta == GO_HOME_DONE)
              {
                /* set go-home status - 'GO_HOME_IDLE*/
                  st_bedNozzleHeightCal.goHomeSta=GO_HOME_IDLE;
                  SERIAL_ECHO("\r\n----------------------current_position.z!!!!!!= ");
                  SERIAL_PRINT((current_position.z*100), 10);
                  SERIAL_ECHO("\r\n");

                  //此时的坐标应该是 Z_MAX_POS
                  SERIAL_PRINT((current_position.z*100), 10);
                  SERIAL_ECHO("\r\n");
                  destination.set(safe_homing_xy, current_position.z);

                  TERN_(1, destination -= probe.offset_xy);

                  if (position_is_reachable(destination)) {

                      // This causes the carriage on Dual X to unpark
                      TERN_(DUAL_X_CARRIAGE, active_extruder_parked = false);

                      TERN_(SENSORLESS_HOMING, safe_delay(500)); // Short delay needed to settle

                      do_blocking_move_to_xy(destination);
                  }
                  else 
                  {
                      SERIAL_ECHO_MSG(STR_ZPROBE_OUT_SER);
                  } 
                  homeaxis_bl(Z_AXIS);
                  /*get to the center(x,y),block*/
                  LcdAutoUIMoveXYBlock((float)(X_BED_SIZE/2), (float)(Y_BED_SIZE/2));
                  /*get to the base height block*/
                  LcdAutoUIMoveZBlock(0.0);
                  bedNozzleHeightCalZ = st_bedNozzleHeightCal.zCoordinateOffset;
                  settings.save();//调用保存数据，保存测量到的高度数据
                  st_bedNozzleHeightCal.bedNozzleHeightState++; 
              }
              break;
        case 2:
              st_bedNozzleHeightCal.bedNozzleHeightState=0;
              st_bedNozzleHeightCal.bedNozzleHeightCalFinishFlag=true;
              break;  
        default:
              break;
      }  
    }
  
      // if((st_bedNozzleHeightCal.bedNozzleHeightState==2)&&(st_bedNozzleHeightCal.bedNozzleHeightCalFinishFlag == true))
      // {//测量动作完成且接收到save 按钮或者返回按钮
      //   st_bedNozzleHeightCal.bedNozzleHeightState=0;
      //   // SERIAL_ECHO("\r\n..bedNozzleHeightCalFinish");
      // }    
}  

/**
 * [LcdAutoUIMoveXY :get to the given postion(x, y), block program running]
 * @Author Creality
 * @Time   2021-06-08
 * @param  _posX      [X coordinate]
 * @param  _posY      [Y coordinate]
 */
void LcdAutoUIMoveXYBlock(float _posX, float _posY)
{
    float lPosX = _posX > X_BED_SIZE ? X_BED_SIZE : _posX;
    float lPosY = _posY > Y_BED_SIZE ? Y_BED_SIZE : _posY;

    do_blocking_move_to_xy(lPosX, lPosY);
}

/**
 * [LcdAutoUIMoveZBlock :get to the given postion(z), block program running]
 * @Author Creality
 * @Time   2021-06-08
 * @param  _posZ      [Z coordinate]
 */
void LcdAutoUIMoveZBlock(float _posZ)
{
    float lPosZ = _posZ > Z_MAX_POS ? Z_MAX_POS : _posZ;

    do_blocking_move_to_z(lPosZ);
}

#endif

#include <stack/ble/ble.h>
#include "tl_common.h"
#include "drivers.h"
#include "app_config.h"
#include "vendor/common/blt_led.h"
#include "application/keyboard/keyboard.h"
#include "vendor/common/tl_audio.h"
#include "vendor/common/blt_soft_timer.h"
#include "vendor/common/blt_common.h"
#include "app_uart.h"

#define		MY_RF_POWER_INDEX	RF_POWER_P10p29dBm // 10.29 dbm
//#define	MY_RF_POWER_INDEX	RF_POWER_P0p04dBm

#define STATUS_FLAG_BATTERY_MASK           0b11000000
#define STATUS_FLAG_COUNTER_MASK           0b00111111
#define STATUS_FLAG_MEDIUM_BATTERY         0b01000000
#define STATUS_FLAG_LOW_BATTERY            0b10000000
#define STATUS_FLAG_CRITICALLY_LOW_BATTERY 0b11000000
#define STATUS_FLAG_FULL_BATTERY           0b00000000
#define STATUS_FLAG_BATTERY_UPDATES_SUPPORT 0b00100000

// Battery voltage thresholds (in mV)
#define BATTERY_EMPTY_THRESHOLD     2000
#define BATTERY_LOW_THRESHOLD       2500
#define BATTERY_MEDIUM_THRESHOLD    2800

#define GPIO_VBAT_DETECT    GPIO_PB7
#define ADC_INPUT_PCHN      B7P

// Battery voltage reading
#define ADC_SAMPLE_NUM		8
_attribute_data_retention_ u16 batt_vol_mv = 0;
_attribute_data_retention_ volatile unsigned int adc_dat_buf[ADC_SAMPLE_NUM];  //size must 16 byte aligned
u8 adc_hw_initialized = 0;

static u8 public_keys[3][28] = {
  { /* Key 1*/ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00 },
  { /* Key 2*/ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00 },
  { /* Key 3*/ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00 }
};

__attribute__((section(".retention_data"))) static u8 current_key_index = 0;
__attribute__((section(".retention_data"))) static u8 stored_battery_status = 0;
__attribute__((section(".retention_data"))) static u16 battery_update_counter = 0; 
__attribute__((section(".retention_data"))) static u16 ble_update_counter = 0;

void derive_mac_from_key(const u8 *key, u8 *mac_addr) {
  mac_addr[5] = key[0] | 0xc0;
  mac_addr[4] = key[1];
  mac_addr[3] = key[2];
  mac_addr[2] = key[3];
  mac_addr[1] = key[4];
  mac_addr[0] = key[5];
}

_attribute_ram_code_ void read_battery_voltage(void)
{
  if(!adc_hw_initialized) {
      // Initialize ADC for battery reading
    gpio_set_output_en(GPIO_VBAT_DETECT, 1);
    gpio_write(GPIO_VBAT_DETECT, 1);

    /******set adc sample clk as 4MHz******/
    adc_set_sample_clk(5); //adc sample clk= 24M/(1+5)=4M

    /******set adc L R channel Gain Stage bias current trimming******/
    adc_set_left_right_gain_bias(GAIN_STAGE_BIAS_PER100, GAIN_STAGE_BIAS_PER100);

    //set misc channel en,  and adc state machine state cnt 2( "set" stage and "capture" state for misc channel)
    adc_set_chn_enable_and_max_state_cnt(ADC_MISC_CHN, 2);  	//set total length for sampling state machine and channel

    //set "capture state" length for misc channel: 240
    //set "set state" length for misc channel: 10
    //adc state machine  period  = 24M/250 = 96K, T = 10.4 uS
    adc_set_state_length(240, 0, 10);  	//set R_max_mc,R_max_c,R_max_s

    //set misc channel use differential_mode,
    //set misc channel resolution 14 bit,  misc channel differential mode
    //notice that: in differential_mode MSB is sign bit, rest are data,  here BIT(13) is sign bit
    analog_write(anareg_adc_res_m, RES14 | FLD_ADC_EN_DIFF_CHN_M);
    adc_set_ain_chn_misc(ADC_INPUT_PCHN, GND);

    //set misc channel vref 1.2V
    adc_set_ref_voltage(ADC_MISC_CHN, ADC_VREF_1P2V);

    //set misc t_sample 6 cycle of adc clock:  6 * 1/4M
    adc_set_tsample_cycle_chn_misc(SAMPLING_CYCLES_6);  	//Number of ADC clock cycles in sampling phase

    //set Analog input pre-scaling 1/8
    adc_set_ain_pre_scaler(ADC_PRESCALER_1F8);

    /******power on sar adc********/
    //note: this setting must be set after all other settings
    adc_power_on_sar_adc(1);
    adc_hw_initialized = 1;
  }

  adc_reset_adc_module();
  u32 t0 = clock_time();

  u16 adc_sample[ADC_SAMPLE_NUM] = {0};
  u32 adc_result;

  for(int i=0; i<ADC_SAMPLE_NUM; i++) {
    adc_dat_buf[i] = 0;
  }
  while(!clock_time_exceed(t0, 25));

  adc_config_misc_channel_buf((u16 *)adc_dat_buf, ADC_SAMPLE_NUM<<2);
  dfifo_enable_dfifo2();

  for(int i=0; i<ADC_SAMPLE_NUM; i++) {
    while(!adc_dat_buf[i]);

    if(adc_dat_buf[i] & BIT(13)) {
        adc_sample[i] = 0;
    } else {
        adc_sample[i] = ((u16)adc_dat_buf[i] & 0x1FFF);
    }
  }

  dfifo_disable_dfifo2();

  u32 adc_average = (adc_sample[2] + adc_sample[3] + adc_sample[4] + adc_sample[5])/4;
  adc_result = adc_average;
  
  // Convert to voltage: (adc_result * Vref * 8) / 0x2000
  // For 1.2V reference: (adc_result * 1.2 * 8) / 0x2000
  batt_vol_mv = (adc_result * 1200 * 8) / 0x2000;
}

u8 get_battery_status(void) {
    return stored_battery_status;
}


int update_battery_status(void) {

  //app_uart_init();
  //at_print("UART reinitialized.\r\n");

  __attribute__((section(".retention_data"))) static u8 first_boot = 1;
  //char debug_buf[64];  // Buffer for formatted debug strings

  //at_print("Entering update_battery_status()\r\n");

  // Always read battery on first boot, otherwise check counter
  if(!first_boot && ++battery_update_counter < 672) {
      //sprintf(debug_buf, "Skip battery update. Counter: %d\r\n", battery_update_counter);
      //at_print(debug_buf);
      return 0;
  }

  //sprintf(debug_buf, "Proceeding with battery update. First boot: %d\r\n", first_boot);
  //at_print(debug_buf);

  first_boot = 0;
  battery_update_counter = 0;

  //at_print("Reading battery voltage...\r\n");
  read_battery_voltage();

  //sprintf(debug_buf, "Battery voltage read: %d mV\r\n", batt_vol_mv);
  //at_print(debug_buf);

  // Set the battery updates support flag
  stored_battery_status = STATUS_FLAG_BATTERY_UPDATES_SUPPORT;
  //at_print("Battery update support flag set.\r\n");

  // Evaluate battery level thresholds
  if(batt_vol_mv <= BATTERY_EMPTY_THRESHOLD) {
      stored_battery_status |= STATUS_FLAG_CRITICALLY_LOW_BATTERY;
      //at_print("Battery level: CRITICALLY LOW\r\n");
  } else if(batt_vol_mv <= BATTERY_LOW_THRESHOLD) {
      stored_battery_status |= STATUS_FLAG_LOW_BATTERY;
      //at_print("Battery level: LOW\r\n");
  } else if(batt_vol_mv <= BATTERY_MEDIUM_THRESHOLD) {
      stored_battery_status |= STATUS_FLAG_MEDIUM_BATTERY;
      //at_print("Battery level: MEDIUM\r\n");
  } else {
      stored_battery_status |= STATUS_FLAG_FULL_BATTERY;
      //at_print("Battery level: FULL\r\n");
  }

  //at_print("Battery status update complete.\r\n");
  return 0;
}


void reinit_ble_with_key(u8 key_index) {
  u8 mac_addr[6];
  derive_mac_from_key(public_keys[key_index], mac_addr);

  bls_ll_setAdvEnable(0); // Stop advertising

  blc_ll_initStandby_module(mac_addr);
  blc_ll_initAdvertising_module(mac_addr);

  bls_ll_setAdvParam(ADV_INTERVAL_2S,
                   ADV_INTERVAL_2S,
                   ADV_TYPE_NONCONNECTABLE_UNDIRECTED,
                   OWN_ADDRESS_PUBLIC,
                   0,
                   NULL,
                   BLT_ENABLE_ADV_ALL,
                   ADV_FP_NONE);

  u8 tbl_advData[] = {
    0x1e, 
    0xff, 
    0x4c, 0x00, 
    0x12, 0x19, 
    0x00,
    0x11, 0x22, 0x33, 0x22, 0x11, 0x22, 0x33, 0x22,
    0x11, 0x22, 0x33, 0x22, 0x11, 0x22, 0x33, 0x22,
    0x11, 0x22, 0x33, 0x22, 0x11, 0x22,
    0x00, 0x00,
  };
  memcpy(&tbl_advData[7], &public_keys[key_index][6], 22);
  tbl_advData[29] = public_keys[key_index][0] >> 6;

  // Use stored battery status instead of reading it again
  tbl_advData[6] = (tbl_advData[6] & ~STATUS_FLAG_BATTERY_MASK) | stored_battery_status;

  bls_ll_setAdvData(tbl_advData, sizeof(tbl_advData));

  bls_ll_setAdvEnable(1); // Restart advertising
}

int key_rotation_callback(void) {
  // Reinitialize UART to ensure it's working
  //app_uart_init();
  update_battery_status();

  if(++ble_update_counter < 15) {
      return 0;
  }

  ble_update_counter = 0;

  //at_print("Key rotation triggered\r\n");
  current_key_index = (current_key_index + 1) % 3;
  reinit_ble_with_key(current_key_index);
  return 0;
}

void user_init_normal(void)
{
	random_generator_init();

	// Initialize UART for debug output
	//app_uart_init();

	u8 mac_addr[6];
	derive_mac_from_key(public_keys[current_key_index], mac_addr);

	////// Controller Initialization  //////////
	blc_ll_initBasicMCU();
	blc_ll_initStandby_module(mac_addr);
	blc_ll_initAdvertising_module(mac_addr);
	blc_ll_initPowerManagement_module();
	bls_pm_setSuspendMask(SUSPEND_ADV | DEEPSLEEP_RETENTION_ADV | SUSPEND_CONN | DEEPSLEEP_RETENTION_CONN);

	u8 tbl_advData[] = {
		0x1e, /* Length (30) */
		0xff, /* Manufacturer Specific Data (type 0xff) */
		0x4c, 0x00, /* Company ID (Apple) */
		0x12, 0x19, /* Offline Finding type and length */
		0x00, /* State */
		0x11, 0x22, 0x33, 0x22, 0x11, 0x22, 0x33, 0x22,
		0x11, 0x22, 0x33, 0x22, 0x11, 0x22, 0x33, 0x22,
		0x11, 0x22, 0x33, 0x22, 0x11, 0x22,
		0x00, /* First two bits */
		0x00, /* Hint (0x00) */
	};

	memcpy(&tbl_advData[7], &public_keys[current_key_index][6], 22);
	tbl_advData[29] = public_keys[current_key_index][0] >> 6;

	// Get initial battery status
	update_battery_status();
	tbl_advData[6] = (tbl_advData[6] & ~STATUS_FLAG_BATTERY_MASK) | stored_battery_status;

	bls_ll_setAdvData((u8 *)tbl_advData, sizeof(tbl_advData));

	u8 status = bls_ll_setAdvParam(ADV_INTERVAL_2S,
									ADV_INTERVAL_2S,
									ADV_TYPE_NONCONNECTABLE_UNDIRECTED,
									OWN_ADDRESS_PUBLIC,
									0,
									NULL,
									BLT_ENABLE_ADV_ALL,
									ADV_FP_NONE);

	if(status != BLE_SUCCESS)
	{
		write_reg8(0x40000, 0x11);
		while(1);
	}

	rf_set_power_level_index (MY_RF_POWER_INDEX);
	bls_ll_setAdvEnable(1);

	blt_soft_timer_init();
  blt_soft_timer_add(key_rotation_callback, (2 * 60 * 1000 * 1000)); // 2 minutes
	//at_print("Normal\r\n");
}

_attribute_ram_code_ void user_init_deepRetn(void)
{
  //app_uart_init();
    
	blc_ll_initBasicMCU();   //mandatory
	rf_set_power_level_index (MY_RF_POWER_INDEX);
	blc_ll_recoverDeepRetention();
	irq_enable();

	//at_print("DeepRetn\r\n");
}

_attribute_ram_code_ void main_loop (void)
{
    static u32 last_print = 0;
    u32 current_time = clock_time();
    
    blt_sdk_main_loop();
    blt_soft_timer_process(MAINLOOP_ENTRY);
    
}


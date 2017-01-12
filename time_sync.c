#include "time_sync.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_error.h"
#include "nrf.h"
#include "nrf_error.h"
#include "nrf_log.h"
#include "nrf_soc.h"
#include "nrf_sdm.h"

#if   defined ( __CC_ARM )
#define TX_CHAIN_DELAY_PRESCALER_0 699
#elif defined ( __ICCARM__ )
#define TX_CHAIN_DELAY_PRESCALER_0 703
#elif defined ( __GNUC__ )
#define TX_CHAIN_DELAY_PRESCALER_0 704
#endif

#define SYNC_TIMER_PRESCALER 0
#define SYNC_RTC_PRESCALER 0

#if SYNC_TIMER_PRESCALER == 0
#define TX_CHAIN_DELAY TX_CHAIN_DELAY_PRESCALER_0
#else
#error Invalid prescaler value
#endif

  
#define TS_LEN_US                            (1000UL)
#define TX_LEN_EXTENSION_US                  (1000UL)
#define TS_SAFETY_MARGIN_US                  (500UL)    /**< The timeslot activity should be finished with this much to spare. */
#define TS_EXTEND_MARGIN_US                  (700UL)   /**< The timeslot activity should request an extension this long before end of timeslot. */


#define MAIN_DEBUG                           0x12345678UL

#define TIMER_MAX_VAL (0xFFFF)
#define RTC_MAX_VAL   (0xFFFFFF)

static volatile bool m_timeslot_session_open;
volatile uint32_t    m_blocked_cancelled_count;
static uint32_t      m_total_timeslot_length = 0;
static uint32_t      m_timeslot_distance = 0;
static uint32_t      m_ppi_chen_mask = 0;
static ts_params_t   m_params;
static volatile bool m_send_sync_pkt = false;
static volatile bool m_timer_update_in_progress = false;

volatile uint32_t m_test_count = 0;
volatile uint32_t m_rcv_count = 0;

static volatile struct
{
    int32_t timer_val;
    int32_t rtc_val;
} m_sync_pkt;

static volatile enum
{
    RADIO_STATE_IDLE, /* Default state */
    RADIO_STATE_RX,   /* Waiting for packets */
    RADIO_STATE_TX    /* Trying to transmit packet */
} m_radio_state = RADIO_STATE_IDLE;

static void sync_timer_offset_compensate(void);
static void timeslot_begin_handler(void);
static void timeslot_end_handler(void);

/**< This will be used when requesting the first timeslot or any time a timeslot is blocked or cancelled. */
static nrf_radio_request_t m_timeslot_req_earliest = {
        NRF_RADIO_REQ_TYPE_EARLIEST,
        .params.earliest = {
            NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED,
            NRF_RADIO_PRIORITY_NORMAL,
            TS_LEN_US,									// 1000 us
            NRF_RADIO_EARLIEST_TIMEOUT_MAX_US
        }};

/**< This will be used at the end of each timeslot to request the next timeslot. */
static nrf_radio_request_t m_timeslot_req_normal = {
        NRF_RADIO_REQ_TYPE_NORMAL,
        .params.normal = {
            NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED,
            NRF_RADIO_PRIORITY_NORMAL,
            0,
            TS_LEN_US
        }};

/**< This will be used at the end of each timeslot to request the next timeslot. */
static nrf_radio_signal_callback_return_param_t m_rsc_return_sched_next_normal = {
        NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END,
        .params.request = {
                (nrf_radio_request_t*) &m_timeslot_req_normal
        }};

/**< This will be used at the end of each timeslot to request the next timeslot. */
static nrf_radio_signal_callback_return_param_t m_rsc_return_sched_next_earliest = {
        NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END,
        .params.request = {
                (nrf_radio_request_t*) &m_timeslot_req_earliest
        }};

/**< This will be used at the end of each timeslot to request an extension of the timeslot. */
static nrf_radio_signal_callback_return_param_t m_rsc_extend = {
        NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND,
        .params.extend = {TX_LEN_EXTENSION_US}
        };

/**< This will be used at the end of each timeslot to request the next timeslot. */
static nrf_radio_signal_callback_return_param_t m_rsc_return_no_action = {
        NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE,
        .params.request = {NULL}
        };


void RADIO_IRQHandler(void)			// 这个是收到了syncpacket后的handler ？
{
    if (NRF_RADIO->EVENTS_END != 0)
    {
        NRF_RADIO->EVENTS_END = 0;
        (void)NRF_RADIO->EVENTS_END;
        
        if (m_radio_state == RADIO_STATE_RX && 
           (NRF_RADIO->CRCSTATUS & RADIO_CRCSTATUS_CRCSTATUS_Msk) == (RADIO_CRCSTATUS_CRCSTATUS_CRCOk << RADIO_CRCSTATUS_CRCSTATUS_Pos))
        {
            sync_timer_offset_compensate();
            ++m_rcv_count; // means compensated 100 times?
        }
    }
}


/**@brief   Function for handling timeslot events.
 */
static nrf_radio_signal_callback_return_param_t * radio_callback (uint8_t signal_type)
{   
    // NOTE: This callback runs at lower-stack priority (the highest priority possible).
    switch (signal_type) {
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
        // TIMER0 is pre-configured for 1Mhz.
        NRF_TIMER0->TASKS_STOP          = 1;
        NRF_TIMER0->TASKS_CLEAR         = 1;
        NRF_TIMER0->MODE                = (TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos);  // set to timer mode
        NRF_TIMER0->EVENTS_COMPARE[0]   = 0;				// You still need to enable compare interrupt
        NRF_TIMER0->EVENTS_COMPARE[1]   = 0;  				// what dose "=0" mean?
        /*
         * 自己的理解：
         * enable EVENTS_COMPARE[x] channel，然后在对应的CC[x] register 上设置数值，一旦达到这个数值就触发event。
         * 或者enable TASKS_CAPTURE channel，当capture到task时，对应的CC[x] register 上就会被写入timer的数值
         */
    
        	if (m_send_sync_pkt) // 第一次按下button1后，括号里就为true了
        	{
        		NRF_TIMER0->INTENSET  = (TIMER_INTENSET_COMPARE0_Set << TIMER_INTENSET_COMPARE0_Pos); // Enable timer0 compare0 interrupt
        	}
        	else
        	{
        		NRF_TIMER0->INTENSET = (TIMER_INTENSET_COMPARE0_Set << TIMER_INTENSET_COMPARE0_Pos) |
        							   (TIMER_INTENSET_COMPARE1_Set << TIMER_INTENSET_COMPARE1_Pos); // Enable timer0 compare0 and compare1 interrupt
        	}

        NRF_TIMER0->CC[0]               = (TS_LEN_US - TS_SAFETY_MARGIN_US);					// capture/compare 0 是 timeslot lenght - safety margin
        NRF_TIMER0->CC[1]               = (TS_LEN_US - TS_EXTEND_MARGIN_US);					// timeslot tutorial 里也有这一句
        NRF_TIMER0->BITMODE             = (TIMER_BITMODE_BITMODE_24Bit << TIMER_BITMODE_BITMODE_Pos);//把 timer0 设置为 24bit，TIMER0 is pre-configured for 1Mhz
        NRF_TIMER0->TASKS_START         = 1;														   // start timer0
    

        NRF_RADIO->POWER                = (RADIO_POWER_POWER_Enabled << RADIO_POWER_POWER_Pos);		// turn on radio

        NVIC_EnableIRQ(TIMER0_IRQn); // turn on NRF_TIMER0 interrupt			// 这样NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0 就可以发生了。
        
        m_total_timeslot_length = 0;
        
        timeslot_begin_handler();			// A7
        
        break;
    
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
        if (NRF_TIMER0->EVENTS_COMPARE[0] &&
           (NRF_TIMER0->INTENSET & (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENCLR_COMPARE0_Pos))) // NRF_TIMER0->EVENTS_COMPARE[0]要不为零
        {
            NRF_TIMER0->TASKS_STOP  = 1;
            NRF_TIMER0->EVENTS_COMPARE[0] = 0;
            (void)NRF_TIMER0->EVENTS_COMPARE[0];
            
            // This is the "timeslot is about to end" timeout

            timeslot_end_handler();
            
            // Schedule next timeslot
            if (m_send_sync_pkt) 									// 按下 button 后就为 true 了。
            {
                m_timeslot_req_normal.params.normal.distance_us = m_total_timeslot_length + m_timeslot_distance;
                return (nrf_radio_signal_callback_return_param_t*) &m_rsc_return_sched_next_normal;
            }
            else
            {
                return (nrf_radio_signal_callback_return_param_t*) &m_rsc_return_sched_next_earliest;
            }
        }


        //NRF_TIMER0->EVENTS_COMPARE[1] 不能为零
        if (NRF_TIMER0->EVENTS_COMPARE[1] &&
           (NRF_TIMER0->INTENSET & (TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENCLR_COMPARE1_Pos))) //不按button1才开启timer0 compare1
        {
            NRF_TIMER0->EVENTS_COMPARE[1] = 0;
            (void)NRF_TIMER0->EVENTS_COMPARE[1];
            
            // This is the "try to extend timeslot" timeout
            
            if (m_total_timeslot_length < (128000000UL - 5000UL - TX_LEN_EXTENSION_US) && !m_send_sync_pkt)
            {
                // Request timeslot extension if total length does not exceed 128 seconds
                return (nrf_radio_signal_callback_return_param_t*) &m_rsc_extend;
            }
            else if (!m_send_sync_pkt)
            {
                // Don't do anything. Timeslot will end and new one requested upon the next timer0 compare. 
                
//                // Return with normal action request
//                m_timeslot_req_normal.params.normal.distance_us = m_total_timeslot_length + m_timeslot_distance;
//                return (nrf_radio_signal_callback_return_param_t*) &m_rsc_return_sched_next_normal;
            }
        }
        
        break;

        
        
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
        RADIO_IRQHandler();
        break;
    
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED:
        // Don't do anything. Our timer will expire before timeslot ends
        return (nrf_radio_signal_callback_return_param_t*) &m_rsc_return_no_action;
    
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED:
        // Extension succeeded: update timer
        NRF_TIMER0->TASKS_STOP          = 1;
        NRF_TIMER0->EVENTS_COMPARE[0]   = 0;
        NRF_TIMER0->EVENTS_COMPARE[1]   = 0;
        NRF_TIMER0->CC[0]               += (TX_LEN_EXTENSION_US - 25);
        NRF_TIMER0->CC[1]               += (TX_LEN_EXTENSION_US - 25);
        NRF_TIMER0->TASKS_START         = 1;
    
        // Keep track of total length
        m_total_timeslot_length += TX_LEN_EXTENSION_US;
        break;
    
    default:
        app_error_handler(MAIN_DEBUG, __LINE__, (const uint8_t*)__FILE__);
        break;
    };

    // Fall-through return: return with no action request
    return (nrf_radio_signal_callback_return_param_t*) &m_rsc_return_no_action;
}


static void update_radio_parameters()
{   
    // TX power
    NRF_RADIO->TXPOWER   = RADIO_TXPOWER_TXPOWER_0dBm   << RADIO_TXPOWER_TXPOWER_Pos;
    
    // RF bitrate
    NRF_RADIO->MODE      = RADIO_MODE_MODE_Nrf_2Mbit       << RADIO_MODE_MODE_Pos;
    
    // Fast startup mode
    NRF_RADIO->MODECNF0 = RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos;
    
    // CRC configuration
    NRF_RADIO->CRCCNF    = RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos; 
    NRF_RADIO->CRCINIT = 0xFFFFFFUL;      // Initial value      
    NRF_RADIO->CRCPOLY = 0x11021UL;     // CRC poly: x^16+x^12^x^5+1
    
    // Packet format 
    NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_S0LEN_Pos) | (0 << RADIO_PCNF0_LFLEN_Pos) | (0 << RADIO_PCNF0_S1LEN_Pos);
    NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled        << RADIO_PCNF1_WHITEEN_Pos) |
                       (RADIO_PCNF1_ENDIAN_Big              << RADIO_PCNF1_ENDIAN_Pos)  |
                       (4                                   << RADIO_PCNF1_BALEN_Pos)   |
                       (sizeof(m_sync_pkt)                  << RADIO_PCNF1_STATLEN_Pos) |
                       (sizeof(m_sync_pkt)                  << RADIO_PCNF1_MAXLEN_Pos);
    NRF_RADIO->PACKETPTR = (uint32_t)&m_sync_pkt;
    
    // Radio address config
    NRF_RADIO->PREFIX0 = m_params.rf_addr[0];
    NRF_RADIO->BASE0   = (m_params.rf_addr[1] << 24 | m_params.rf_addr[2] << 16 | m_params.rf_addr[3] << 8 | m_params.rf_addr[4]);
    
    NRF_RADIO->TXADDRESS   = 0;
    NRF_RADIO->RXADDRESSES = (1 << 0);
    
    NRF_RADIO->FREQUENCY = m_params.rf_chn;
    NRF_RADIO->TXPOWER   = RADIO_TXPOWER_TXPOWER_Pos4dBm << RADIO_TXPOWER_TXPOWER_Pos;
    
    NRF_RADIO->EVENTS_END = 0;
    
    NRF_RADIO->INTENCLR = 0xFFFFFFFF;
    NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;
    
    NVIC_EnableIRQ(RADIO_IRQn);			//A11
}

/**@brief IRQHandler used for execution context management. 
  *        Any available handler can be used as we're not using the associated hardware.
  *        This handler is used to stop and disable UESB
  */
void timeslot_end_handler(void)
{   
    uint32_t ppi_chn;
    
    ppi_chn = m_params.ppi_chns[2];			// eep 和tep 在  begin handler 里设置了。
    
    NRF_RADIO->TASKS_DISABLE = 1;
    NRF_RADIO->INTENCLR      = 0xFFFFFFFF;
    
    NRF_PPI->CHENCLR = (1 << ppi_chn);
    
    m_total_timeslot_length = 0;
    m_radio_state           = RADIO_STATE_IDLE;
}

/**@brief IRQHandler used for execution context management. 
  *        Any available handler can be used as we're not using the associated hardware.
  *        This handler is used to initiate UESB RX/TX
  *
  *        This is the first timing-critical part of the code, which ensures that the
  *        timing master captures the free running timer value at a consistent time delta
  *        from when the packet is actually transmitted
  */
void timeslot_begin_handler(void)
{
    uint32_t ppi_chn;
    uint32_t ppi_chn2;
    
    if (!m_send_sync_pkt) //此时m_send_sync_pkt为false，当按了button后就是true，所以state为TX时也进不了这个if，只有idle时才行。那这个if有何意义？
    {
        if (m_radio_state    != RADIO_STATE_RX ||
            NRF_RADIO->STATE != (RADIO_STATE_STATE_Rx << RADIO_STATE_STATE_Pos))			//A9	此时m_radio_state = RADIO_STATE_IDLE
        {
            ppi_chn = m_params.ppi_chns[2];    									//名字是ppi_chns[2]，但其实是3
            
            update_radio_parameters();					//A10
            
            NRF_RADIO->SHORTS     = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_START_Msk;
            NRF_RADIO->TASKS_RXEN = 1;
            
            NRF_PPI->CH[ppi_chn].EEP = (uint32_t) &NRF_RADIO->EVENTS_ADDRESS;					  	// trigger: Address sent or received  //What exactly is it?
            NRF_PPI->CH[ppi_chn].TEP = (uint32_t) &m_params.high_freq_timer[0]->TASKS_CAPTURE[1]; 	// capture timer value to CC[1] register
            																		//就是说收到广播后获取本地timer2的时间，这个时间就是compensate里的本地时间
            NRF_PPI->CHENSET         = (1 << ppi_chn);											  	//enable channel
            
            m_radio_state = RADIO_STATE_RX;				//A12   现在 m_radio_state = RADIO_STATE_RX
        }
        
        return;
    }
    
    if (m_radio_state == RADIO_STATE_RX)
    {
        NRF_RADIO->EVENTS_DISABLED = 0;
        NRF_RADIO->TASKS_DISABLE = 1;
        while (NRF_RADIO->EVENTS_DISABLED == 0)
        {
            __NOP(); //也就是说这句话真的是存不存在就行？ 在while循环里出不来了啊。 timeslot end handler 会把state设置回IDLE, 这样就出来循环了。
        }
    }

    
// 那也就是说只有当 m_radio_state == RADIO_STATE_TX  时才会执行下面的语句？
    update_radio_parameters();
    
    //here to re-configure ppi for TX mode, "ppi_configure" is configured for RX mode.
    //NRF_TIMER3 is used to trigger "get nrf_timer0 value" and "radio transmission"

    ppi_chn  = m_params.ppi_chns[0];	//ppi_chn=1
    ppi_chn2 = m_params.ppi_chns[1];	//ppi_chn2=2
    // Use PPI to create fixed offset between timer capture and packet transmission

    NRF_PPI->CH[ppi_chn].EEP = (uint32_t) &m_params.high_freq_timer[1]->EVENTS_COMPARE[0];// Compare event #0: Capture timer value for free running timer
    NRF_PPI->CH[ppi_chn].TEP = (uint32_t) &m_params.high_freq_timer[0]->TASKS_CAPTURE[1]; //capture the timer value and write to cc1 register
    NRF_PPI->CHENSET         = (1 << ppi_chn);											//enable channel
    
    NRF_PPI->CH[ppi_chn2].EEP = (uint32_t) &m_params.high_freq_timer[1]->EVENTS_COMPARE[1];// Compare event #1: Trigger radio transmission
    NRF_PPI->CH[ppi_chn2].TEP = (uint32_t) &NRF_RADIO->TASKS_START;
    NRF_PPI->CHENSET          = (1 << ppi_chn2);									    //enable channel
    
    m_params.high_freq_timer[1]->PRESCALER   = 4; // 1 us resolution
    m_params.high_freq_timer[1]->MODE        = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
    m_params.high_freq_timer[1]->SHORTS      = TIMER_SHORTS_COMPARE1_STOP_Msk | TIMER_SHORTS_COMPARE1_CLEAR_Msk;
    m_params.high_freq_timer[1]->TASKS_STOP  = 1;
    m_params.high_freq_timer[1]->TASKS_CLEAR = 1;
    m_params.high_freq_timer[1]->CC[0]       = 45; // Matches 40 us radio rampup time
    m_params.high_freq_timer[1]->CC[1]       = 60; // Margin for timer readout
    
    m_params.high_freq_timer[1]->EVENTS_COMPARE[0] = 0;
    m_params.high_freq_timer[1]->EVENTS_COMPARE[1] = 0;
    
    NRF_RADIO->SHORTS                        = RADIO_SHORTS_END_DISABLE_Msk;
    NRF_RADIO->TASKS_TXEN                    = 1;
    m_params.high_freq_timer[1]->TASKS_START = 1;						//start nrf_timer3
    
    while (m_params.high_freq_timer[1]->EVENTS_COMPARE[0] == 0)   		//如果不等于零，还能等于什么？
    {
        // Wait for timer to trigger
        __NOP();
    }
    
    m_radio_state                                 = RADIO_STATE_TX;
    m_sync_pkt.timer_val                          = m_params.high_freq_timer[0]->CC[1];
    m_sync_pkt.rtc_val                            = m_params.rtc->COUNTER;
    
    ++m_test_count;
}

/**@brief Function for handling the Application's system events.
 *
 * @param[in]   sys_evt   system event.  在main.c里
 */
void ts_on_sys_evt(uint32_t sys_evt)
{
    switch(sys_evt)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
        case NRF_EVT_FLASH_OPERATION_ERROR:
            break;
        case NRF_EVT_RADIO_BLOCKED:
        case NRF_EVT_RADIO_CANCELED:
        {
            // Blocked events are rescheduled with normal priority. They could also
            // be rescheduled with high priority if necessary.
            uint32_t err_code = sd_radio_request((nrf_radio_request_t*) &m_timeslot_req_earliest);
            APP_ERROR_CHECK(err_code);

            m_blocked_cancelled_count++;
            
            break;
        }
        case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
            NRF_LOG_INFO("NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN\r\n");
            app_error_handler(MAIN_DEBUG, __LINE__, (const uint8_t*)__FILE__);
            break;
        case NRF_EVT_RADIO_SESSION_CLOSED:
            {
                m_timeslot_session_open = false;
                
                NRF_LOG_INFO("NRF_EVT_RADIO_SESSION_CLOSED\r\n");
            }
        
            break;
        case NRF_EVT_RADIO_SESSION_IDLE:
        {
            NRF_LOG_INFO("NRF_EVT_RADIO_SESSION_IDLE\r\n");
            
            uint32_t err_code = sd_radio_session_close();
            APP_ERROR_CHECK(err_code);
            break;
        }
        default:
            // No implementation needed.
            NRF_LOG_INFO("Event: 0x%08x\r\n", sys_evt);
            break;
    }
}

static void sync_timer_start(void)		// 		实际是NRF_TIMER2，定义的名字叫high_freq_timer[0]		这个就是中心坐标时间
{
    m_params.high_freq_timer[0]->TASKS_STOP  = 1;
    m_params.high_freq_timer[0]->TASKS_CLEAR = 1;
    m_params.high_freq_timer[0]->PRESCALER   = SYNC_TIMER_PRESCALER;
    m_params.high_freq_timer[0]->BITMODE     = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;		//16 bit timer
    m_params.high_freq_timer[0]->CC[0]       = TIMER_MAX_VAL;
    m_params.high_freq_timer[0]->CC[3]       = TIMER_MAX_VAL / 2; // Only used for debugging purposes such as pin toggling
    m_params.high_freq_timer[0]->SHORTS      = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    m_params.high_freq_timer[0]->TASKS_START = 1;
    NRF_LOG_INFO("timer2 started\r\n");
}

void SWI3_EGU3_IRQHandler(void)						// I don't know what it is used for
{
    if (NRF_EGU3->EVENTS_TRIGGERED[0] != 0)
    {
        NRF_EGU3->EVENTS_TRIGGERED[0] = 0;
        (void) NRF_EGU3->EVENTS_TRIGGERED[0];
        
        NRF_PPI->CHENCLR = m_ppi_chen_mask;		    //'enable clear' means clear
        NRF_LOG_INFO("m_ppi_chen_mask be cleared\r\n");
        
        m_timer_update_in_progress = false;
    }

    if (NRF_EGU3->EVENTS_TRIGGERED[1] != 0)
        {
            NRF_EGU3->EVENTS_TRIGGERED[1] = 0;
            (void) NRF_EGU3->EVENTS_TRIGGERED[1];

            NRF_PPI->CHENCLR = (1 << 6) | (1 << 7)| (1 << 9);
            NRF_LOG_INFO("679 be cleared\r\n");
        }
}

/*
 * The second timing-critical part is how the receiver updates its local free running
 * timer when a sync beacon packet is received. Note the magic value "TX_CHAIN_DELAY" in the following code:
 */
static inline void sync_timer_offset_compensate(void)
{
    uint32_t peer_timer;
    uint32_t local_timer;
    uint32_t timer_offset;
    
    if (m_timer_update_in_progress)
    {
        return;
    }

    peer_timer  = m_sync_pkt.timer_val;
    peer_timer += TX_CHAIN_DELAY;									//TX_CHAIN_DELAY怎么就magical了？干什么用的？
    local_timer = m_params.high_freq_timer[0]->CC[1];   //cc1 was captured in "timeslot begin handler"
    
    if (local_timer > peer_timer)
    {
        timer_offset = TIMER_MAX_VAL - local_timer + peer_timer;    //TIMER_MAX_VAL = 2^16 = 65536  //为什么与timer的最大值有关系？不理解
    }
    else
    {
        timer_offset = peer_timer - local_timer;
    }
    
    if (timer_offset == 0 ||
        timer_offset == TIMER_MAX_VAL)
    {
//        NRF_GPIO->OUT ^= (1 << 25);
    	NRF_LOG_INFO("already in sync\r\n");
        return;
    }
    
    // Write offset to timer compare register
    m_params.high_freq_timer[0]->CC[2] = (TIMER_MAX_VAL - timer_offset);
    
    m_timer_update_in_progress = true;
    
    // Enable PPI channels
    NRF_PPI->CHENSET = m_ppi_chen_mask;
    NRF_LOG_INFO("m_ppi_chen_mask be set\r\n");
}

static void ppi_configure(void)		//A3;B3												// in this function, only configuration, not enable, CHENSET is enable
{
    uint32_t chn0, chn1, chn2, chg;
    
    chn0 = m_params.ppi_chns[0];		//chn0=1 										//ppi_chns[2] is in timeslot_begin_handler and end handler
    chn1 = m_params.ppi_chns[1];		//chn1=2
    chn2 = m_params.ppi_chns[3];		//chn3=4
    chg  = m_params.ppi_chhg;			//chg =0
    
    m_ppi_chen_mask = (1 << chn0) | (1 << chn1) | (1 << chn2);
    
    // so CH[chn0]=CH[1] which is channel 1, don't be confused!
    // PPI channel 0: clear timer when offset value is reached
    NRF_PPI->CHENCLR      = (1 << chn0); // Channel enable clear 这什么意思？clear是clearregister上的bit？相当与初始化？
    NRF_PPI->CH[chn0].EEP = (uint32_t) &m_params.high_freq_timer[0]->EVENTS_COMPARE[2]; //把channel0的eep设为timer0的compare event
    NRF_PPI->CH[chn0].TEP = (uint32_t) &m_params.high_freq_timer[0]->TASKS_CLEAR;       //tep是清零timer
    
    // PPI channel 1: disable PPI channel 0 such that the timer is only reset once. 
    NRF_PPI->CHENCLR      = (1 << chn1);
    NRF_PPI->CH[chn1].EEP = (uint32_t) &m_params.high_freq_timer[0]->EVENTS_COMPARE[2]; //这个是说把channel0的eep设为timer0的compare event
    NRF_PPI->CH[chn1].TEP = (uint32_t) &NRF_PPI->TASKS_CHG[chg].DIS;  					// TEP is 'disable ppi group'
    
    //
    NRF_PPI->CHENCLR      = (1 << chn2);
    NRF_PPI->CH[chn2].EEP = (uint32_t) &m_params.high_freq_timer[0]->EVENTS_COMPARE[2];
    NRF_PPI->CH[chn2].TEP = (uint32_t) &m_params.egu->TASKS_TRIGGER[0];					// Here 'egu' appears
    
    //自己的comment：PPI group
    NRF_PPI->TASKS_CHG[chg].DIS = 1;													// here the group is disabled
    NRF_PPI->CHG[chg]           = (1 << chn0) | (1 << chn2);							// the ppi CHG[0] contain chn0 and chn2
}

uint32_t ts_init(const ts_params_t * p_params)
{
    memcpy(&m_params, p_params, sizeof(ts_params_t));
    
    if (m_params.high_freq_timer[0] == 0 ||
        m_params.rtc                == 0 ||
        m_params.egu                == 0)
    {
        // TODO: Check all params
        return NRF_ERROR_INVALID_PARAM;
    }
    
    if (SYNC_RTC_PRESCALER != m_params.rtc->PRESCALER)
    {
        // TODO: Handle this
        return NRF_ERROR_INVALID_STATE;
    }
    
    return NRF_SUCCESS;
}

uint32_t ts_enable(void)	//A1;B1			// 这个function运行完后就待命了。
{
    uint32_t err_code;
    
    if (m_timeslot_session_open)		// 这地方是true还是false ? 我觉得是 false
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    err_code = sd_clock_hfclk_request();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code |= sd_power_mode_set(NRF_POWER_MODE_CONSTLAT); //Constant latency mode
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = sd_radio_session_open(radio_callback); //Opens a session for radio timeslot requests			//A2;B2
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = sd_radio_request(&m_timeslot_req_earliest); //Requests a radio timeslot
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    ppi_configure();	//A3;B3
    
    NVIC_ClearPendingIRQ(m_params.egu_irq_type);
    NVIC_SetPriority(m_params.egu_irq_type, 7);
    NVIC_EnableIRQ(m_params.egu_irq_type);
    
    m_params.egu->INTENSET = EGU_INTENSET_TRIGGERED0_Msk; 				//Enable interrupt for TRIGGERED[0] event
    m_params.egu->INTENSET = EGU_INTENSET_TRIGGERED1_Msk;
    
    m_blocked_cancelled_count  = 0;
    m_send_sync_pkt            = false;					//A4
    m_radio_state              = RADIO_STATE_IDLE;		//A5
    
    sync_timer_start();			//A6;B4      //是NRF_TIMER2，定义的名字叫high_freq_timer[0]
    // since three ppi channels about NRF_TIMER2 already been configured, now the NRF_TIMER2 is running. the last thing is enable ppi channels.
    
    m_timeslot_session_open    = true;
    
//    NRF_GPIO->DIRSET = (1 << 23);
//    NRF_GPIO->DIRSET = (1 << 25);
    
    return NRF_SUCCESS;
}

uint32_t ts_disable(void)
{
    return NRF_ERROR_NOT_SUPPORTED;
}

uint32_t ts_tx_start(uint32_t sync_freq_hz)			// controlled by button
{
    uint32_t distance;
    
    distance = (1000000 / sync_freq_hz);
    if (distance >= NRF_RADIO_DISTANCE_MAX_US)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    m_timeslot_distance = distance;
    
    m_send_sync_pkt = true;
    
    return NRF_SUCCESS;
}
    
uint32_t ts_tx_stop()								// controlled by button
{
    m_send_sync_pkt = false;
    
    return NRF_SUCCESS;
}

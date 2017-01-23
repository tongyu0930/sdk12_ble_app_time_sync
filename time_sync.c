/**
 * @}
 */

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


#define TX_CHAIN_DELAY_PRESCALER_0 		0 //原值＝704		这个数值需要调试
#define SYNC_TIMER_PRESCALER 			0
#define SYNC_RTC_PRESCALER 				0

#if SYNC_TIMER_PRESCALER 				== 0
#define TX_CHAIN_DELAY 					TX_CHAIN_DELAY_PRESCALER_0
#else
#error Invalid prescaler value
#endif
  
#define TS_LEN_US                            (1000UL)
#define TX_LEN_EXTENSION_US                  (1000UL)
#define TS_SAFETY_MARGIN_US                  (500UL)   /**< The timeslot activity should be finished with this much to spare. */
#define TS_EXTEND_MARGIN_US                  (700UL)   /**< The timeslot activity should request an extension this long before end of timeslot. */

#define MAIN_DEBUG                           0x12345678UL

#define TIMER_MAX_VAL (0xFFFF)	 //16bit
#define RTC_MAX_VAL   (0xFFFFFF) //24bit


static volatile bool 	m_timeslot_session_open;
static uint32_t      	m_total_timeslot_length = 0;
static uint32_t     	m_timeslot_distance = 0;
static uint32_t     	m_ppi_chen_mask = 0;
static ts_params_t  	ts_params;
static volatile bool 	m_send_sync_pkt = false;
static volatile bool 	m_timer_update_in_progress = false;

volatile uint32_t    	m_blocked_cancelled_count = 0;	// 这三个count有何意义？
volatile uint32_t 					 m_test_count = 0;
volatile uint32_t 					  m_rcv_count = 0;

volatile bool alreadyinsync = false;
volatile bool I_WANNA_STOP = false;

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

/**< This will be used at the end of each timeslot to end timeslot. */
static nrf_radio_signal_callback_return_param_t m_rsc_return_end = {
		NRF_RADIO_SIGNAL_CALLBACK_ACTION_END,
        .params.request = {NULL}
        };


void RADIO_IRQHandler(void) // 收到了sync packet后经过 radio callback，然后来到这里
{
    if (NRF_RADIO->EVENTS_END != 0)
    {
        NRF_RADIO->EVENTS_END = 0;
        (void)NRF_RADIO->EVENTS_END;
        
        // 如果是RX mode， Packet received
        if (m_radio_state==RADIO_STATE_RX&&(NRF_RADIO->CRCSTATUS & RADIO_CRCSTATUS_CRCSTATUS_Msk)==(RADIO_CRCSTATUS_CRCSTATUS_CRCOk<<RADIO_CRCSTATUS_CRCSTATUS_Pos))
        {
            sync_timer_offset_compensate();
            ++m_rcv_count;
        }
    }
}


/**@brief   Function for handling timeslot events.
     "radio_callback"
     will be called when the radio timeslot starts. From this point the NRF_RADIO and NRF_TIMER0 peripherals can be freely accessed by the application.
     is called whenever the NRF_TIMER0 interrupt occurs.
     is called whenever the NRF_RADIO interrupt occurs.
     will be called at ARM interrupt priority level 0. This implies that none of the sd_* API calls can be used from p_radio_signal_callback().

     注意这个functiong的返回类型，看看其declaration，里面的callback_action一共有4中可能
 */
static nrf_radio_signal_callback_return_param_t * radio_callback (uint8_t signal_type) // This callback runs at lower-stack priority(the highest priority possible).
{
    switch (signal_type) {

    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
        // TIMER0 is pre-configured for 1Mhz. which means 1us increace one.
        NRF_TIMER0->TASKS_STOP          = 1;
        NRF_TIMER0->TASKS_CLEAR         = 1;
        NRF_TIMER0->MODE                = (TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos);
        NRF_TIMER0->EVENTS_COMPARE[0]   = 0;													// You still need to enable compare interrupt
        NRF_TIMER0->EVENTS_COMPARE[1]   = 0;  													// what dose "=0" mean?
        NRF_TIMER0->CC[0]               = (TS_LEN_US - TS_SAFETY_MARGIN_US);					// capture/compare 0 是 timeslot lenght - safety margin
        NRF_TIMER0->CC[1]               = (TS_LEN_US - TS_EXTEND_MARGIN_US);					// timeslot tutorial 里也有这一句
        NRF_TIMER0->BITMODE             = (TIMER_BITMODE_BITMODE_24Bit << TIMER_BITMODE_BITMODE_Pos);//把 timer0 设置为 24bit，TIMER0 is pre-configured for 1Mhz
        NRF_TIMER0->TASKS_START         = 1;														   // start timer0

        if (m_send_sync_pkt) // true or false is only controlled by button1
        {	// for TX mode
        	NRF_TIMER0->INTENSET  = (TIMER_INTENSET_COMPARE0_Set << TIMER_INTENSET_COMPARE0_Pos); // Enable timer0 compare0 interrupt
        }
        else
        {	// for RX mode
        	NRF_TIMER0->INTENSET = (TIMER_INTENSET_COMPARE0_Set << TIMER_INTENSET_COMPARE0_Pos) |
        						   (TIMER_INTENSET_COMPARE1_Set << TIMER_INTENSET_COMPARE1_Pos); // Enable timer0 compare0 and compare1 interrupt
        }

        NVIC_EnableIRQ(TIMER0_IRQn); // turn on NRF_TIMER0 interrupt   // 这样NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0 就可以发生了。

        NRF_RADIO->POWER                = (RADIO_POWER_POWER_Enabled << RADIO_POWER_POWER_Pos);		// turn on radio
        
        m_total_timeslot_length = 0;
        
        timeslot_begin_handler();
        break;
    
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:

    	if(I_WANNA_STOP){
    		return (nrf_radio_signal_callback_return_param_t*) &m_rsc_return_end;
    	}else{
			// for both RX and TX mode
			if (NRF_TIMER0->EVENTS_COMPARE[0] && (NRF_TIMER0->INTENSET & (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENCLR_COMPARE0_Pos)))
			{
				NRF_TIMER0->TASKS_STOP  = 1;
				NRF_TIMER0->EVENTS_COMPARE[0] = 0;	// 这是说停止timer0后在清零compare？
				(void)NRF_TIMER0->EVENTS_COMPARE[0];

				// This is the "timeslot is about to end" timeout
				timeslot_end_handler(); // 这里就是所谓的 do graceful shut down?

				// Schedule next timeslot
				if (m_send_sync_pkt) // true or false is only controlled by button1
				{	// for TX mode
					m_timeslot_req_normal.params.normal.distance_us = m_total_timeslot_length + m_timeslot_distance;
					return (nrf_radio_signal_callback_return_param_t*) &m_rsc_return_sched_next_normal;
				}
				else
				{	// for RX mode
					return (nrf_radio_signal_callback_return_param_t*) &m_rsc_return_sched_next_earliest; // 1000 us
				}
			}

			// only for 程序处于 RX mode
			if (NRF_TIMER0->EVENTS_COMPARE[1] && (NRF_TIMER0->INTENSET & (TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENCLR_COMPARE1_Pos)))
			{
				NRF_TIMER0->EVENTS_COMPARE[1] = 0;
				(void)NRF_TIMER0->EVENTS_COMPARE[1];

				// This is the "try to extend timeslot" timeout
				if (m_total_timeslot_length < (128000000UL - 5000UL - TX_LEN_EXTENSION_US) && !m_send_sync_pkt)	// 5000UL 是什么？
				{
					// Request timeslot extension if total length does not exceed 128 seconds
					return (nrf_radio_signal_callback_return_param_t*) &m_rsc_extend; // 1000UL
				}
				else if (!m_send_sync_pkt) // true or false is only controlled by button1
				{
					// Don't do anything. Timeslot will end and new one requested upon the next timer0 compare.

					// Return with normal action request
					//m_timeslot_req_normal.params.normal.distance_us = m_total_timeslot_length + m_timeslot_distance;
					//return (nrf_radio_signal_callback_return_param_t*) &m_rsc_return_sched_next_normal;
				}
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

        m_total_timeslot_length += TX_LEN_EXTENSION_US; // Keep track of total length
        //NRF_LOG_INFO("extend succeeded\r\n");
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
    NRF_RADIO->MODE      = RADIO_MODE_MODE_Nrf_2Mbit    << RADIO_MODE_MODE_Pos;
    
    // Fast startup mode
    NRF_RADIO->MODECNF0  = RADIO_MODECNF0_RU_Fast 		<< RADIO_MODECNF0_RU_Pos;
    
    // CRC configuration
    NRF_RADIO->CRCCNF    = RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos; 
    NRF_RADIO->CRCINIT   = 0xFFFFFFUL;    // Initial value
    NRF_RADIO->CRCPOLY   = 0x11021UL;     // CRC poly: x^16+x^12^x^5+1
    
    // Packet format 
    NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_S0LEN_Pos) | (0 << RADIO_PCNF0_LFLEN_Pos) | (0 << RADIO_PCNF0_S1LEN_Pos);
    NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled        << RADIO_PCNF1_WHITEEN_Pos) |
                       (RADIO_PCNF1_ENDIAN_Big              << RADIO_PCNF1_ENDIAN_Pos)  |
                       (4                                   << RADIO_PCNF1_BALEN_Pos)   |
                       (sizeof(m_sync_pkt)                  << RADIO_PCNF1_STATLEN_Pos) |
                       (sizeof(m_sync_pkt)                  << RADIO_PCNF1_MAXLEN_Pos);
    NRF_RADIO->PACKETPTR = (uint32_t)&m_sync_pkt;
    
    // Radio address config
    NRF_RADIO->PREFIX0 		= ts_params.rf_addr[0];
    NRF_RADIO->BASE0  		= (ts_params.rf_addr[1] << 24 | ts_params.rf_addr[2] << 16 | ts_params.rf_addr[3] << 8 | ts_params.rf_addr[4]);
    
    NRF_RADIO->TXADDRESS    = 0;
    NRF_RADIO->RXADDRESSES  = (1 << 0);
    
    NRF_RADIO->FREQUENCY 	= ts_params.rf_chn;
    NRF_RADIO->TXPOWER   	= RADIO_TXPOWER_TXPOWER_Pos4dBm << RADIO_TXPOWER_TXPOWER_Pos;
    
    NRF_RADIO->EVENTS_END 	= 0;
    
    NRF_RADIO->INTENCLR 	= 0xFFFFFFFF;
    NRF_RADIO->INTENSET 	= RADIO_INTENSET_END_Msk;  //radio活动结束时interrupt
    
    NVIC_EnableIRQ(RADIO_IRQn);
}

/**@brief IRQHandler used for execution context management. 
  *        Any available handler can be used as we're not using the associated hardware.
  *        This handler is used to stop and disable UESB
  */
void timeslot_end_handler(void)
{   
    uint32_t ppi_chn;
    
    ppi_chn = ts_params.ppi_chns[2];			// eep 和tep 在 begin handler 里设置了。
    
    NRF_RADIO->TASKS_DISABLE = 1;				// 关闭radio
    NRF_RADIO->INTENCLR      = 0xFFFFFFFF;
    
    NRF_PPI->CHENCLR = (1 << ppi_chn); 			// disable 在begin handler里设置的ppi
    
    m_total_timeslot_length  = 0;
    m_radio_state            = RADIO_STATE_IDLE;
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
    if (!m_send_sync_pkt) // 只要是 程序出于RX mode（没按button）就会 只 进入这个if
    {
    	// 没有在发送packet 并且RADIO也不是在RX状态
        if (m_radio_state != RADIO_STATE_RX || NRF_RADIO->STATE != (RADIO_STATE_STATE_Rx << RADIO_STATE_STATE_Pos)) //m_radio_state = RADIO_STATE_IDLE 才可进入
        {
            update_radio_parameters(); // 这里面设置了radio interrupt 但没开启radio, 因为前面已经开启了radio，所以现在出于开启状态，但不是在RX mode
            
            NRF_RADIO->SHORTS     	 = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_START_Msk;
            NRF_RADIO->TASKS_RXEN    = 1;																		//Enable RADIO in RX mode
            
            NRF_PPI->CH[3].EEP = (uint32_t) &NRF_RADIO->EVENTS_ADDRESS;					  	// trigger: Address sent or received  //What exactly is it?
            NRF_PPI->CH[3].TEP = (uint32_t) &NRF_TIMER2->TASKS_CAPTURE[1]; 	// capture timer value to timer2's CC[1] register
            NRF_PPI->CHENSET         = (1 << 3); // enable channel3							//就是说收到广播后获取本地timer2的时间，这个时间就是compensate里的本地时间
            
            m_radio_state = RADIO_STATE_RX;
        }
        
        return;
    }
    
    if (m_radio_state == RADIO_STATE_RX)						// 这个if是为了干什么？
    {
        NRF_RADIO->EVENTS_DISABLED = 0;
        NRF_RADIO->TASKS_DISABLE = 1;  	// Disable RADIO
        while (NRF_RADIO->EVENTS_DISABLED == 0)
        {
            __NOP();											// 到了这句话系统还能不能继续走下去？
        }
    }

    // 那也就是说只有当 m_radio_state != RX || IDLE  时才会执行下面的语句
    update_radio_parameters();
    
    //here to re-configure ppi for TX mode, "ppi_configure" is configured for RX mode.
    //NRF_TIMER3 is used to trigger "get nrf_timer0 value" and "radio transmission"
    // Use PPI to create fixed offset between timer capture and packet transmission
    
    NRF_PPI->CH[1].EEP = (uint32_t) &NRF_TIMER3->EVENTS_COMPARE[0];	// Compare event #0: Capture timer value for free running timer
    NRF_PPI->CH[1].TEP = (uint32_t) &NRF_TIMER2->TASKS_CAPTURE[1]; 	// capture the timer value and write to cc1 register
    NRF_PPI->CHENSET         = (1 << 1);											  	// enable channel1

    NRF_PPI->CH[2].EEP = (uint32_t) &NRF_TIMER3->EVENTS_COMPARE[1]; // Compare event #1: Trigger radio transmission
    NRF_PPI->CH[2].TEP = (uint32_t) &NRF_RADIO->TASKS_START;
    NRF_PPI->CHENSET          = (1 << 2);									        //enable channel2
    
    NRF_TIMER3->PRESCALER   = 4; // 1 us resolution
    NRF_TIMER3->MODE        = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
    NRF_TIMER3->SHORTS      = TIMER_SHORTS_COMPARE1_STOP_Msk | TIMER_SHORTS_COMPARE1_CLEAR_Msk;
    NRF_TIMER3->TASKS_STOP  = 1;
    NRF_TIMER3->TASKS_CLEAR = 1;
    NRF_TIMER3->CC[0]       = 45; // Matches 40 us radio rampup time
    NRF_TIMER3->CC[1]       = 60; // Margin for timer readout
    
    NRF_TIMER3->EVENTS_COMPARE[0] = 0;
    NRF_TIMER3->EVENTS_COMPARE[1] = 0;
    
    NRF_RADIO->SHORTS                        = RADIO_SHORTS_END_DISABLE_Msk;
    NRF_RADIO->TASKS_TXEN                    = 1;
    NRF_TIMER3->TASKS_START = 1;						//start nrf_timer3
    
    while (NRF_TIMER3->EVENTS_COMPARE[0] == 0)   		//如果不等于零，还能等于什么？
    {
        // Wait for timer to trigger
        __NOP();
    }
    
    m_radio_state                = RADIO_STATE_TX;
    m_sync_pkt.timer_val         = NRF_TIMER2->CC[1];
    m_sync_pkt.rtc_val           = ts_params.rtc->COUNTER;
    
    ++m_test_count;
}

/**@brief Function for handling the Application's system events.
 *
 * @param[in]   sys_evt   system event.  在main.c里call的		这个应该是系统有反应了就自动call
 */
void ts_on_sys_evt(uint32_t sys_evt)
{
    switch(sys_evt)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS: //注意这里没有break，如果走的是这个case，出去后继续再往下走，而不是退出函数。
        case NRF_EVT_FLASH_OPERATION_ERROR:
            break;
        case NRF_EVT_RADIO_BLOCKED:
        	/*
			 * The requested timeslot could not be scheduled due to a collision with other activities.
			 * The application should request a new timeslot either at the earliest possible, or at the next normal position.
			 * 去这里看看
			 * SoftDevices > S132 SoftDevice > S132 SoftDevice Specification > Multiprotocol support > Radio Timeslot API usage scenarios
			 */
        case NRF_EVT_RADIO_CANCELED:
        	{// Blocked events are rescheduled with normal priority. They could also
            // be rescheduled with high priority if necessary.
            uint32_t err_code = sd_radio_request((nrf_radio_request_t*) &m_timeslot_req_earliest);
            APP_ERROR_CHECK(err_code);

            m_blocked_cancelled_count++;
            break;}

        case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
            NRF_LOG_INFO("NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN\r\n");
            app_error_handler(MAIN_DEBUG, __LINE__, (const uint8_t*)__FILE__);
            break;

        case NRF_EVT_RADIO_SESSION_IDLE:
        	{/*
			 * The session has no remaining scheduled timeslots. If this event is triggered, the application ends the session.
			 * Note that it is also possible to request new timeslots here(more on that later).
			 */
			NRF_LOG_INFO("NRF_EVT_RADIO_SESSION_IDLE\r\n");

			uint32_t err_code = sd_radio_session_close();  //cannot consider the session closed until the @ref NRF_EVT_RADIO_SESSION_CLOSED event is received.
			APP_ERROR_CHECK(err_code);
			break;}

        case NRF_EVT_RADIO_SESSION_CLOSED: //The session is closed and all acquired resources are released
			m_timeslot_session_open = false;

			NRF_LOG_INFO("NRF_EVT_RADIO_SESSION_CLOSED\r\n");
            break;

        default:
            // No implementation needed.
            NRF_LOG_INFO("Event: 0x%08x\r\n", sys_evt);
            break;
    }
}


void SWI3_EGU3_IRQHandler(void)				// it is used to shut down thoses PPIs that is used for clear timer2's offset
{
    if (NRF_EGU3->EVENTS_TRIGGERED[0] != 0)
    {
        NRF_EGU3->EVENTS_TRIGGERED[0] = 0;
        (void) NRF_EGU3->EVENTS_TRIGGERED[0];
        
        NRF_PPI->CHENCLR = m_ppi_chen_mask;		// 关闭对时的那几个channel
        NRF_LOG_INFO("m_ppi_chen_mask be cleared\r\n");
        
        m_timer_update_in_progress = false;
    }

    /*
    if (NRF_EGU3->EVENTS_TRIGGERED[1] != 0)
        {
            NRF_EGU3->EVENTS_TRIGGERED[1] = 0;
            (void) NRF_EGU3->EVENTS_TRIGGERED[1];

            // TODO
        }
     */
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
    peer_timer += TX_CHAIN_DELAY; // magic value
    local_timer = NRF_TIMER2->CC[1];   //cc1 was captured in "timeslot begin handler"
    
    if (local_timer > peer_timer)
    {
        timer_offset = TIMER_MAX_VAL - local_timer + peer_timer;    //TIMER_MAX_VAL = 2^16 = 65536
    }
    else
    {
        timer_offset = peer_timer - local_timer;
    }
    
    //if (timer_offset == 0 ||timer_offset == TIMER_MAX_VAL) // this one is original
    if (timer_offset < 10 || timer_offset > (TIMER_MAX_VAL-10))
    {
        //NRF_GPIO->OUT ^= (1 << 25); // 这话是作者留下的，什么意思？
    	alreadyinsync = true;
    	NRF_LOG_INFO("already in sync\r\n");
        //return;
    }else{
    	//alreadyinsync = false;  //去掉这句，already in sync 后就再也不会变为false了。
    }
    
	if(!alreadyinsync)
	{
    NRF_TIMER2->CC[2] = (TIMER_MAX_VAL - timer_offset); // Write offset to timer compare register
    
    m_timer_update_in_progress = true;
    
    NRF_PPI->CHENSET = m_ppi_chen_mask;	// Enable PPI channels 开启对时的那几个channel
    NRF_LOG_INFO("m_ppi_chen_mask be set\r\n");
	}
}

static void ppi_configure(void)	// in this function, only configuration, not enable, CHENSET is enable
{
    uint32_t chn0, chn1, chn2, chg;
    
    chn0 = ts_params.ppi_chns[0];		//chn0=1 										//ppi_chns[2] is in timeslot_begin_handler and end handler
    chn1 = ts_params.ppi_chns[1];		//chn1=2
    chn2 = ts_params.ppi_chns[3];		//chn3=4
    chg  = ts_params.ppi_chhg;			//chg =0  // group
    
    m_ppi_chen_mask = (1 << chn0) | (1 << chn1) | (1 << chn2);
    
    // so CH[chn0]=CH[1] which is channel 1, don't be confused!
    // PPI channel 0: clear timer when offset value is reached
    NRF_PPI->CHENCLR      = (1 << chn0);
    NRF_PPI->CH[chn0].EEP = (uint32_t) &NRF_TIMER2->EVENTS_COMPARE[2]; //把channel0的eep设为timer0的compare event  与cc[2] compare
    NRF_PPI->CH[chn0].TEP = (uint32_t) &NRF_TIMER2->TASKS_CLEAR;       //tep是清零timer
    
    // PPI channel 1: disable PPI channel 0 such that the timer is only reset once. 
    NRF_PPI->CHENCLR      = (1 << chn1);
    NRF_PPI->CH[chn1].EEP = (uint32_t) &NRF_TIMER2->EVENTS_COMPARE[2]; //这个是说把channel0的eep设为timer0的compare event
    NRF_PPI->CH[chn1].TEP = (uint32_t) &NRF_PPI->TASKS_CHG[chg].DIS;  					// TEP is 'disable ppi group'
    
    NRF_PPI->CHENCLR      = (1 << chn2);
    NRF_PPI->CH[chn2].EEP = (uint32_t) &NRF_TIMER2->EVENTS_COMPARE[2];
    NRF_PPI->CH[chn2].TEP = (uint32_t) &ts_params.egu->TASKS_TRIGGER[0];					// Here 'egu' appears
    
    //自己的comment：PPI group
    NRF_PPI->TASKS_CHG[chg].DIS = 1;													// here the group is disabled
    NRF_PPI->CHG[chg]           = (1 << chn0) | (1 << chn2);							// the ppi CHG[0] contain chn0 and chn2

	ts_params.egu->INTENSET 		= EGU_INTENSET_TRIGGERED0_Msk; 				//Enable interrupt for TRIGGERED[0] event
	ts_params.egu->INTENSET 		= EGU_INTENSET_TRIGGERED1_Msk;				//这个是我自己加的，当初为了测试
    NVIC_ClearPendingIRQ(ts_params.egu_irq_type);
    NVIC_SetPriority(ts_params.egu_irq_type, 7);
    NVIC_EnableIRQ(ts_params.egu_irq_type);


    //for test start

    	NRF_PPI->CHENCLR      = (1 << 0);
    	NRF_PPI->CH[0].EEP = (uint32_t) &NRF_TIMER2->EVENTS_COMPARE[3];
    	NRF_PPI->CH[0].TEP = (uint32_t) &NRF_GPIOTE->TASKS_OUT[0];
    	NRF_PPI->CHENSET   = PPI_CHENSET_CH0_Msk; // enable

    /*

    	//NRF_PPI->CHENCLR      = (1 << 6);
    	//NRF_PPI->CH[6].EEP = (uint32_t) &NRF_TIMER2->EVENTS_COMPARE[3];
    	//NRF_PPI->CH[6].TEP = (uint32_t) &NRF_TIMER4->TASKS_CLEAR;//清零timer

    	// PPI channel 7: disable PPI channel 6 such that the timer is only reset once.
    	//NRF_PPI->CHENCLR      = (1 << 7);
    	//NRF_PPI->CH[7].EEP = (uint32_t) &NRF_TIMER2->EVENTS_COMPARE[3]; //这个是说把channel0的eep设为timer0的compare event
    	//NRF_PPI->CH[7].TEP = (uint32_t) &NRF_PPI->TASKS_CHG[2].DIS;  					// TEP is 'disable ppi group'

    	//egu
    	//NRF_PPI->CHENCLR      = (1 << 9);
    	//NRF_PPI->CH[9].EEP = (uint32_t) &NRF_TIMER2->EVENTS_COMPARE[3];
    	//NRF_PPI->CH[9].TEP = (uint32_t) &NRF_EGU3->TASKS_TRIGGER[1];					// Here 'egu' appears

    	// PPI group
    	//NRF_PPI->TASKS_CHG[2].DIS = 1;													// here the group is disabled
    	//NRF_PPI->CHG[2]           = (1 << 6) | (1 << 9);							// the ppi CHG[0] contain chn0 and chn2

    	//led toggle
    	//NRF_PPI->CHENCLR      = (1 << 8);
    	//NRF_PPI->CH[8].EEP = (uint32_t) &NRF_TIMER4->EVENTS_COMPARE[0];  // for led
    	//NRF_PPI->CH[8].TEP = (uint32_t) &NRF_GPIOTE->TASKS_OUT[0];  //Action on pin is configured in CONFIG[0].POLARITY
    	//NRF_PPI->CHENSET   = PPI_CHENSET_CH8_Msk; // enable channel8

    	// timer4
    	NRF_TIMER4->TASKS_STOP  = 1;
    	NRF_TIMER4->TASKS_CLEAR = 1;
    	NRF_TIMER4->PRESCALER   = 8;
    	NRF_TIMER4->BITMODE     = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;		//16 bit timer
    	//NRF_TIMER4->MODE     	= TIMER_MODE_MODE_Counter << TIMER_MODE_MODE_Pos;
    	NRF_TIMER4->CC[0]       = 0xFFFF;
    	//NRF_TIMER4->CC[1]       = 0;
    	NRF_TIMER4->SHORTS      = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    	NRF_TIMER4->EVENTS_COMPARE[0] = 0; //这句话是干什么？
    	NRF_TIMER4->TASKS_START = 1;
    */

    //for test end
}


uint32_t ts_enable(void) // 这个function返回error_code，所以类型是uint32_t
{

    uint32_t       err_code;
    uint32_t 	   hfclk_is_running;

    I_WANNA_STOP = false;

//timersync parameter
	uint8_t        rf_address[5] = {0xDE, 0xAD, 0xBE, 0xEF, 0x19};						// 地址是干嘛的？
	ts_params.high_freq_timer[0] = NRF_TIMER2;											// 为什么要有parameter这个结构体，是为了让动用了哪些peripheral一目了然。
	ts_params.high_freq_timer[1] = NRF_TIMER3;
	ts_params.rtc                = NRF_RTC1;
	ts_params.egu                = NRF_EGU3;
	ts_params.egu_irq_type       = SWI3_EGU3_IRQn;
	ts_params.ppi_chhg           = 0; // PPI Channel Group
	ts_params.ppi_chns[0]    	 = 1;
	ts_params.ppi_chns[1]    	 = 2;
	ts_params.ppi_chns[2]    	 = 3;
	ts_params.ppi_chns[3]    	 = 4;
	ts_params.rf_chn         	 = 125;													// what is this channel?
	memcpy(ts_params.rf_addr, rf_address, sizeof(rf_address)); 							// copy // 要不然就要对数组的每个unit单独赋值。要5句话
    //memcpy(&ts_params, &ts_params, sizeof(ts_params_t)); // copy p_params to ts_params



	ppi_configure();



// 设置并开启 timer2 这个就是被同步的timer
	NRF_TIMER2->TASKS_STOP  = 1;
	NRF_TIMER2->TASKS_CLEAR = 1;
	NRF_TIMER2->PRESCALER   = 8; // 原值为：SYNC_TIMER_PRESCALER
	NRF_TIMER2->BITMODE     = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;		//16 bit timer
	NRF_TIMER2->CC[0]       = TIMER_MAX_VAL;
	NRF_TIMER2->CC[3]       = TIMER_MAX_VAL; // Only used for debugging purposes such as pin toggling
	NRF_TIMER2->SHORTS      = TIMER_SHORTS_COMPARE0_CLEAR_Msk | TIMER_SHORTS_COMPARE3_CLEAR_Msk;
	NRF_TIMER2->TASKS_START = 1;

	NRF_TIMER2->EVENTS_COMPARE[3] = 0; // 自己加的 我也不知道这到底是干什么的

	NRF_LOG_INFO("timer2 started\r\n");



// timeslot related
    if (m_timeslot_session_open) { return NRF_ERROR_INVALID_STATE; } // 这地方是true还是false ? 我觉得是 false

    sd_clock_hfclk_is_running(&hfclk_is_running);
    NRF_LOG_INFO("hfclk_is_running = %d\r\n", hfclk_is_running);
    if(!hfclk_is_running)
    {
    	err_code = sd_clock_hfclk_request();
    	if (err_code != NRF_SUCCESS) { return err_code; }
    }

    err_code = sd_power_mode_set(NRF_POWER_MODE_CONSTLAT); //Constant latency mode // How about low power mode?
    if (err_code != NRF_SUCCESS) { return err_code; }

    /*
     * "radio_callback"
     * will be called when the radio timeslot starts. From this point the NRF_RADIO and NRF_TIMER0 peripherals can be freely accessed by the application.
     * is called whenever the NRF_TIMER0 interrupt occurs.
     * is called whenever the NRF_RADIO interrupt occurs.
     * will be called at ARM interrupt priority level 0. This implies that none of the sd_* API calls can be used from p_radio_signal_callback().
     */
    err_code = sd_radio_session_open(radio_callback); //Opens a session for radio timeslot requests
NRF_LOG_INFO("error code = %d\r\n", err_code);
    if (err_code != NRF_SUCCESS) { return err_code; }

    // request the first timeslot (which must be of type earliest possible)
    err_code = sd_radio_request(&m_timeslot_req_earliest); // Requests a radio timeslot // 我觉得这就是开始timeslot，这句话完了之后radio callback里的第一个case就会被call

    if (err_code != NRF_SUCCESS) { return err_code; }

    m_timeslot_session_open    	= true;
    


    m_blocked_cancelled_count  	= 0;
    m_send_sync_pkt            	= false;
    m_radio_state              	= RADIO_STATE_IDLE;
    

    NRF_LOG_INFO("Started listening for beacons.\r\n");
    NRF_LOG_INFO("Call 'ts_tx_start' to start sending sync beacons\r\n");

    return NRF_SUCCESS;
}

uint32_t ts_disable(void)
{
	I_WANNA_STOP = true;

	NRF_LOG_INFO("session try to close\r\n");

	//NRF_TIMER3->TASKS_STOP  = 1; // turn off timer3

	//NRF_PPI->CHENCLR      = (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4); // clear 4 channels for time sync, test channel which is channel0 is not cleared yet.

	//NRF_RADIO->TASKS_DISABLE = 1; // stop radio
	//NRF_RADIO->POWER                = (RADIO_POWER_POWER_Disabled << RADIO_POWER_POWER_Pos);		// turn off radio

	//m_timer_update_in_progress = false;

	//alreadyinsync = false;

	/*
	 *  m_timeslot_distance 不用管
	 *  m_send_sync_pkt 不用管
	 *  m_radio_state
	 *  m_timeslot_session_open 不用管
	 *  m_total_timeslot_length 不用管
	 *  m_ppi_chen_mask 不用管
	 *  no need to stop timer0, because softdevice need the timer0 anyway.
	 */

    //return NRF_ERROR_NOT_SUPPORTED;
	return NRF_SUCCESS;
}

uint32_t ts_tx_start(uint32_t sync_freq_hz)	// controlled by button
{
    uint32_t distance;
    
    distance = (1000000 / sync_freq_hz);
    if (distance >= NRF_RADIO_DISTANCE_MAX_US) { return NRF_ERROR_INVALID_PARAM; }
    
    m_timeslot_distance = distance; // m_timeslot_distance 的值一直为 distance
    
    m_send_sync_pkt = true;
    
    return NRF_SUCCESS;
}
    
uint32_t ts_tx_stop()					   // controlled by button
{
    m_send_sync_pkt = false;
    
    return NRF_SUCCESS;
}

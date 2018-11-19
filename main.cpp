#include <sx1280.h>
#include <ns_trace.h>
#include <RangingCorrection.h>
#include "mbed.h"
#include "radio.h"
#include "sx1280-hal.h"
#include "ST7565.h"
#define TRACE_GROUP  "main"


/*!
 * \brief Status of ranging distance
 */
enum RangingStatus
{
    RNG_INIT = 0,
    RNG_PROCESS,
    RNG_VALID,
    RNG_TIMEOUT,
    RNG_PER_ERROR
};

/*!
 * \brief List of states for demo state machine
 */
enum AppStates
{
    APP_IDLE = 0,               // nothing to do (or wait a radio interrupt)
    APP_RANGING_DONE,
    APP_RANGING_TIMEOUT,
    APP_RANGING_CONFIG,
    APP_RNG,
    APP_RX,                     // Rx done
    APP_RX_TIMEOUT,             // Rx timeout
    APP_RX_ERROR,               // Rx error
    APP_TX,                     // Tx done
    APP_TX_TIMEOUT,             // Tx error
    PER_TX_START,               // PER master
    PER_RX_START                // PER slave
};

/*!
 * \brief Define current demo mode
 */
enum EntityMode
{
    MASTER = 0,
    SLAVE
};

#define CHANNELS 40
const uint32_t Channels[] =
    {
        2450000000,
        2402000000,
        2476000000,
        2436000000,
        2430000000,
        2468000000,
        2458000000,
        2416000000,
        2424000000,
        2478000000,
        2456000000,
        2448000000,
        2462000000,
        2472000000,
        2432000000,
        2446000000,
        2422000000,
        2442000000,
        2460000000,
        2474000000,
        2414000000,
        2464000000,
        2454000000,
        2444000000,
        2404000000,
        2434000000,
        2410000000,
        2408000000,
        2440000000,
        2452000000,
        2480000000,
        2426000000,
        2428000000,
        2466000000,
        2418000000,
        2412000000,
        2406000000,
        2470000000,
        2438000000,
        2420000000,
    };


double t0 =       -0.016432807883697;                         // X0
double t1 =       0.323147003165358;                          // X1
double t2 =       0.014922061351196;                          // X1^2
double t3 =       0.000137832006285;                          // X1^3
double t4 =       0.536873856625399;                          // X2
double t5 =       0.040890089178579;                          // X2^2
double t6 =       -0.001074801048732;                         // X2^3
double t7 =       0.000009240142234;                          // X2^4


double p[8] = { 0,
                -4.1e-9,
                1.03e-7,
                1.971e-5,
                -0.00107,
                0.018757,
                0.869171,
                3.072450 };

/*!
 * \brief Defines the local payload buffer size
 */
#define BUFFER_SIZE                     255


/*!
 * \brief Define time used in PingPong demo to synch with cycle
 * RX_TX_INTER_PACKET_DELAY is the free time between each cycle (time reserve)
 */
#define RX_TX_INTER_PACKET_DELAY        150  // ms
#define RX_TX_TRANSITION_WAIT           15   // ms

/*!
 * \brief Size of ticks (used for Tx and Rx timeout)
 */
#define RX_TIMEOUT_TICK_SIZE            RADIO_TICK_SIZE_1000_US

#define RNG_TIMER_MS                    384 // ms
#define RNG_COM_TIMEOUT                 250 // ms LORA_BW_1600:100ms LORA_BW_0800:250ms
#define APP_CONFIG_BANDWIDTH            LORA_BW_0800
/*!
 * \brief Ranging raw factors
 *                                  SF5     SF6     SF7     SF8     SF9     SF10
 */
const uint16_t RNG_CALIB_0400[] = { 10299,  10271,  10244,  10242,  10230,  10246  };
const uint16_t RNG_CALIB_0800[] = { 11486,  11474,  11453,  11426,  11417,  11401  };
const uint16_t RNG_CALIB_1600[] = { 13308,  13493,  13528,  13515,  13430,  13376  };
const double   RNG_FGRAD_0400[] = { -0.148, -0.214, -0.419, -0.853, -1.686, -3.423 };
const double   RNG_FGRAD_0800[] = { -0.041, -0.811, -0.218, -0.429, -0.853, -1.737 };
const double   RNG_FGRAD_1600[] = { 0.103,  -0.041, -0.101, -0.211, -0.424, -0.87  };


/*!
 * \brief Buffer and its size
 */
uint8_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

/*!
 * \brief Define min and max ranging channels count
 */
const uint16_t DEMO_RNG_CHANNELS_COUNT_MAX = 255;
const uint16_t DEMO_RNG_CHANNELS_COUNT_MIN = 10;

static uint8_t CurrentChannel;
static uint16_t MeasuredChannels;
int RngResultIndex;
double RawRngResults[DEMO_RNG_CHANNELS_COUNT_MAX];
double RssiRng[DEMO_RNG_CHANNELS_COUNT_MAX];


/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( void );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( IrqErrorCode_t );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRangingDone( IrqRangingCode_t );

/*!
 * \brief All the callbacks are stored in a structure
 */
RadioCallbacks_t Callbacks =
    {
        &OnTxDone,        // txDone
        &OnRxDone,        // rxDone
        NULL,             // syncWordDone
        NULL,             // headerDone
        &OnTxTimeout,     // txTimeout
        &OnRxTimeout,     // rxTimeout
        &OnRxError,       // rxError
        &OnRangingDone,   // rangingDone
        NULL,             // cadDone
    };

SX1280Hal Radio(LORA_MOSI, LORA_MISO, LORA_SCLK, LORA_NSS, LORA_BUSY, LORA_DIO1, NC, NC, LORA_RST, &Callbacks);
DigitalOut TX_LED(APP_LED1);
DigitalOut RX_LED(APP_LED2);
#ifdef APP_HAS_LCD
ST7565 LCD(SPI_8, 40000000, LCD_MOSI, LCD_MISO, LCD_SCLK, LCD_NSS, LCD_RESET, LCD_DC, "lcd", 128, 64);
#endif

/*!
 * \brief Locals parameters and status for radio API
 * NEED TO BE OPTIMIZED, COPY OF STUCTURE ALREADY EXISTING
 */
PacketParams_t PacketParams;
PacketStatus_t PacketStatus;
ModulationParams_t ModulationParams;

/*!
 * \brief Flag holding the current internal state of the demo application
 */
AppStates AppState = APP_IDLE;
/*!
 * \brief Ticker for master to synch Tx frames. Flags for PER and PingPong demo
 * for Synch TX in cycle.
 */
Ticker SendNextPacket;
static bool SendNext = false;

/*!
 * \brief Ticker for slave to synch Tx frames. Flags for PER and PingPong demo
 * for Synch RX in cycle.
 */
Ticker ReceiveNextPacket;
static bool ReceiveNext = false;


int8_t RssiValue=0;
uint32_t CntPacketTx = 0;
uint32_t CntPacketRxOK = 0;
uint32_t CntPacketRxOKSlave = 0;
RangingStatus RngStatus = RNG_INIT;
uint32_t RngAddress = 0x32101230;
uint8_t RngRequestCount = 60;
uint8_t RngFullScale = 30;
double RngFei = 0.0f;
double RngDistance = 0.0f;
double RngFeiFactor;
uint16_t RngReqDelay;        // Time between ranging request
uint16_t RngCalib;           // Ranging Calibration
uint16_t RxTimeOutCount;     // Rx packet received KO (by timeout)
uint32_t CntPacketRxKOSlave;
uint8_t RngAntenna;          // NOT USED

/**
 * LCD CONFIGS
 */
unsigned short backgroundcolor=White;
unsigned short foregroundcolor=Black;
char orient=1;
uint8_t refreshDisplay = 0;

/*!
 * \brief Callback of ticker PerSendNextPacket
 */
void SendNextPacketEvent( void )
{
  SendNext = true;
  if( RngStatus == RNG_PROCESS )
  {
    CntPacketRxKOSlave++;
  }
}

uint8_t CheckDistance( void )
{
  double displayRange = 0.0;

  uint16_t j = 0;
  uint16_t i;

  tr_info( "#id: %d", (int)CntPacketTx );
  if( RngResultIndex > 0 )
  {
    for( i = 0; i < RngResultIndex; ++i )
    {
      RawRngResults[i] = RawRngResults[i] - ( RngFeiFactor * RngFei / 1000 );
    }

    for (int i = RngResultIndex - 1; i > 0; --i)
    {
      for (int j = 0; j < i; ++j)
      {
        if (RawRngResults[j] > RawRngResults[j+1])
        {
          int temp = RawRngResults[j];
          RawRngResults[j] = RawRngResults[j+1];
          RawRngResults[j+1] = temp;
        }
      }
    }
    double median;
    if ((RngResultIndex % 2) == 0)
    {
      median = (RawRngResults[RngResultIndex/2] + RawRngResults[(RngResultIndex/2) - 1])/2.0;
    }
    else
    {
      median = RawRngResults[RngResultIndex/2];
    }

    if( median < 100 )
    {
      tr_info("median: %f \n\r", median );
      // Apply the short range correction and RSSI short range improvement below 50 m
      displayRange = Sx1280RangingCorrection::ComputeRangingCorrectionPolynome(
          ModulationParams.Params.LoRa.SpreadingFactor,
          ModulationParams.Params.LoRa.Bandwidth,
          median
      );
      tr_info("Corrected range: %f \n\r", displayRange );
      //displayRange = t0 + t1 * rssi + t2 * pow(rssi,2) + t3 * pow(rssi, 3) + t4 * median + t5 * pow(median,2) + t6 * pow(median, 3) + t7 * pow(median, 4) ;
      //printf("displayRange %f \n\r", displayRange );
//            double correctedRange = 0;
//            uint8_t k = 0;
//            uint8_t order = 6;
//            for( k = 1; k <= (order+1); k++ )                    // loop though each polynomial term and sum
//            {
//                correctedRange = correctedRange + p[k] * pow( median, ( order + 1 - k ) );
//                printf("correctedRange[%d] %f \n\r", k, correctedRange );
//            }
//            printf("Final correctedRange %f \n\r", correctedRange );
//            displayRange = correctedRange - 2;
    }
    else
    {
      displayRange = median;
    }

    if( j < DEMO_RNG_CHANNELS_COUNT_MIN )
    {
      RngStatus = RNG_PER_ERROR;
    }
    else
    {
      RngStatus = RNG_VALID;
    }

    if( displayRange < 0 )
    {
      RngDistance = 0.0;
    }
    else
    {
      RngDistance = displayRange;
    }
  }
  tr_info( ", Rssi: %d, Zn: %3d, Zmoy: %5.1f, FEI: %d\r\n", RssiValue, j, displayRange, ( int32_t )RngFei );

  return j;
}

void updateDisplay(void){
#ifdef APP_HAS_LCD
  LCD.cls();
  LCD.locate(0,0);
  LCD.printf("Lora Ranging\r\n");
  LCD.printf("#id %d\r\n",CntPacketTx);
  LCD.printf("RNG: %f\r\n", RngDistance);
  LCD.printf("Rssi: %d\r\n", RssiValue);
#endif
}

/*!
 * \brief Specify serial datarate for UART debug output
 */
void baud(int baudrate) {
  Serial s(USBTX, USBRX);

  s.baud(baudrate);
}

int main() {
  baud(115200);
  TX_LED = 1;
  RX_LED = 1;
  mbed_trace_init();
  mbed_trace_config_set(TRACE_ACTIVE_LEVEL_DEBUG);
  wait_ms(10); // wait for on board DC/DC start-up time

  tr_info("Starting Ranging");
  TX_LED = 0;
  RX_LED = 0;

#ifdef APP_HAS_LCD
  LCD.set_contrast(0x0F);
  LCD.set_orientation(orient);
  LCD.background(backgroundcolor);
  LCD.foreground(foregroundcolor);
  updateDisplay();
#endif

  MeasuredChannels  = 0;
  CurrentChannel    = 0;

  CntPacketTx = 0;
  RngFei = 0;
  Radio.Init( );
  Radio.SetStandby( STDBY_RC );
  Radio.SetRegulatorMode( USE_DCDC );

  Radio.SetBufferBaseAddresses( 0x00, 0x00 );
  Radio.SetTxParams( 13, RADIO_RAMP_20_US );
  ModulationParams.PacketType = PACKET_TYPE_LORA;
  PacketParams.PacketType = PACKET_TYPE_LORA;
  ModulationParams.Params.LoRa.SpreadingFactor = LORA_SF10;
  ModulationParams.Params.LoRa.Bandwidth = APP_CONFIG_BANDWIDTH;
  ModulationParams.Params.LoRa.CodingRate = LORA_CR_4_5;
  PacketParams.Params.LoRa.PreambleLength = 12;
  PacketParams.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH;
  PacketParams.Params.LoRa.PayloadLength = 7;
  PacketParams.Params.LoRa.Crc = LORA_CRC_ON;
  PacketParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
  switch( ModulationParams.Params.LoRa.Bandwidth )
  {
    case LORA_BW_0400:
      RngCalib     = RNG_CALIB_0400[ ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) - 5 ];
      RngFeiFactor = ( double )RNG_FGRAD_0400[ ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) - 5 ];
      RngReqDelay  = RNG_TIMER_MS >> ( 0 + 10 - ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) );
      break;

    case LORA_BW_0800:
      RngCalib     = RNG_CALIB_0800[ ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) - 5 ];
      RngFeiFactor = ( double )RNG_FGRAD_0800[ ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) - 5 ];
      RngReqDelay  = RNG_TIMER_MS >> ( 1 + 10 - ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) );
      break;

    case LORA_BW_1600:
      RngCalib     = RNG_CALIB_1600[ ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) - 5 ];
      RngFeiFactor = ( double )RNG_FGRAD_1600[ ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) - 5 ];
      RngReqDelay  = RNG_TIMER_MS >> ( 2 + 10 - ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) );
      break;
  }

  Radio.SetPollingMode( );
  Radio.SetLNAGainSetting(LNA_HIGH_SENSITIVITY_MODE);


  uint16_t TimeOnAir = RX_TX_INTER_PACKET_DELAY;
  EntityMode Entity = (EntityMode)APP_ENTITY_MODE;

  if(Entity == MASTER){
    Radio.SetDioIrqParams( IRQ_RX_DONE | IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_RANGING_MASTER_RESULT_VALID | IRQ_RANGING_MASTER_TIMEOUT,
                           IRQ_RX_DONE | IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_RANGING_MASTER_RESULT_VALID | IRQ_RANGING_MASTER_TIMEOUT,
                           IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    RngDistance = 0.0f;
    AppState = APP_RANGING_CONFIG;
  }
  else{
    Radio.SetDioIrqParams( IRQ_RADIO_ALL, IRQ_RADIO_ALL, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    AppState = APP_RANGING_CONFIG;
  }

  while(1){
    if(refreshDisplay){
      refreshDisplay = 0;
      updateDisplay();
    }
    Radio.ProcessIrqs( );
    if(Entity == MASTER) {
      switch (AppState) {
        case APP_RANGING_CONFIG:
          RngStatus = RNG_INIT;
          CntPacketTx++;
          ModulationParams.PacketType = PACKET_TYPE_LORA;
          PacketParams.PacketType = PACKET_TYPE_LORA;
          ModulationParams.Params.LoRa.SpreadingFactor = LORA_SF10;
          ModulationParams.Params.LoRa.Bandwidth = APP_CONFIG_BANDWIDTH;
          ModulationParams.Params.LoRa.CodingRate = LORA_CR_4_5;
          PacketParams.Params.LoRa.PreambleLength = 12;
          PacketParams.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH;
          PacketParams.Params.LoRa.PayloadLength = 7;
          PacketParams.Params.LoRa.Crc = LORA_CRC_ON;
          PacketParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;

          Radio.SetPacketType(ModulationParams.PacketType);
          Radio.SetModulationParams(&ModulationParams);
          Radio.SetPacketParams(&PacketParams);
          Radio.SetRfFrequency(2402000000UL);
          CntPacketRxOK = 0;
          CntPacketRxOKSlave = 0;
          Buffer[0] = (uint8_t)(( RngAddress >> 24 ) & 0xFF);
          Buffer[1] = (uint8_t)(( RngAddress >> 16 ) & 0xFF);
          Buffer[2] = (uint8_t)(( RngAddress >>  8 ) & 0xFF);
          Buffer[3] = (uint8_t)(( RngAddress & 0xFF ));
          Buffer[4] = CurrentChannel;    // set the first channel to use
          Buffer[5] = 0;      // set the antenna strategy
          Buffer[6] = RngRequestCount; // set the number of hops
          TX_LED = 1;
          Radio.SendPayload( Buffer, PacketParams.Params.LoRa.PayloadLength, ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, RNG_COM_TIMEOUT } );
          AppState = APP_IDLE;
          break;

        case APP_RNG:
          if( SendNext )
          {
            SendNext = false;
            MeasuredChannels++;
            if( MeasuredChannels <= RngRequestCount )
            {
              Radio.SetRfFrequency( Channels[CurrentChannel] );
              TX_LED = 1;

              CurrentChannel++;
              if( CurrentChannel >= CHANNELS )
              {
                CurrentChannel -= CHANNELS;
              }

              AppState = APP_IDLE;
              Radio.SetTx( ( TickTime_t ){ RADIO_TICK_SIZE_1000_US, 0xFFFF } );
            }
            else
            {
              CntPacketRxOKSlave = CheckDistance( );
              refreshDisplay = 1;
              SendNextPacket.detach( );
              SendNext = false;
              AppState = APP_RANGING_CONFIG;
              RngStatus = RNG_INIT;
            }
          }
          break;

        case APP_RANGING_DONE:
          TX_LED = 0;
          RawRngResults[RngResultIndex] = Radio.GetRangingResult( RANGING_RESULT_RAW );
          RawRngResults[RngResultIndex] += Sx1280RangingCorrection::GetRangingCorrectionPerSfBwGain(
              ModulationParams.Params.LoRa.SpreadingFactor,
              ModulationParams.Params.LoRa.Bandwidth,
              Radio.GetRangingPowerDeltaThresholdIndicator( )
          );
          RngResultIndex++;

          CntPacketRxOK++;
          AppState = APP_RNG;
          break;

        case APP_RANGING_TIMEOUT:
          TX_LED = 0;
          AppState = APP_RNG;
          break;

        case APP_RX:
          RX_LED = 0;
          if( RngStatus == RNG_INIT )
          {
            Radio.GetPayload( Buffer, &BufferSize, BUFFER_SIZE );
            if( BufferSize > 0 )
            {
              RxTimeOutCount = 0;
              RngStatus = RNG_PROCESS;
              RngFei = ( double )( ( ( int32_t )Buffer[4] << 24 ) | \
                                                                            ( ( int32_t )Buffer[5] << 16 ) | \
                                                                            ( ( int32_t )Buffer[6] <<  8 ) | \
                                                                                         Buffer[7] );
              RssiValue = Buffer[8]; // for ranging post-traitment (since V3 only)
              ModulationParams.PacketType = PACKET_TYPE_RANGING;
              PacketParams.PacketType     = PACKET_TYPE_RANGING;

              ModulationParams.Params.LoRa.SpreadingFactor = LORA_SF10;
              ModulationParams.Params.LoRa.Bandwidth = APP_CONFIG_BANDWIDTH;
              ModulationParams.Params.LoRa.CodingRate = LORA_CR_4_5;
              PacketParams.Params.LoRa.PreambleLength = 12;
              PacketParams.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH;
              PacketParams.Params.LoRa.PayloadLength = 10;
              PacketParams.Params.LoRa.Crc = LORA_CRC_ON;
              PacketParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;

              Radio.SetPacketType( ModulationParams.PacketType );
              Radio.SetModulationParams( &ModulationParams );
              Radio.SetPacketParams( &PacketParams );
              Radio.SetRangingRequestAddress( RngAddress );
              Radio.SetRangingCalibration( RngCalib );
              Radio.SetTxParams( 13, RADIO_RAMP_20_US );

              MeasuredChannels = 0;
              RngResultIndex   = 0;
              SendNextPacket.attach_us( &SendNextPacketEvent, RngReqDelay * 1000 );
              AppState = APP_RNG;
            }
            else
            {
              AppState = APP_RANGING_CONFIG;
            }
          }
          else
          {
            AppState = APP_RANGING_CONFIG;
          }
          break;

        case APP_TX:
          TX_LED = 0;
          if( RngStatus == RNG_INIT )
          {
            RX_LED = 1;
            Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RNG_COM_TIMEOUT } );
            AppState = APP_IDLE;
          }
          else
          {
            AppState = APP_RANGING_CONFIG;
          }
          break;

        case APP_RX_TIMEOUT:
          RX_LED = 0;
          RngStatus = RNG_TIMEOUT;
          AppState = APP_RANGING_CONFIG;
          break;

        case APP_RX_ERROR:
          RX_LED = 0;
          AppState = APP_RANGING_CONFIG;
          break;

        case APP_TX_TIMEOUT:
          TX_LED = 0;
          AppState = APP_RANGING_CONFIG;
          break;

        case APP_IDLE: // do nothing
          break;

        default:
          AppState = APP_RANGING_CONFIG;
          break;
      }
    }
    else{ //slave
      switch( AppState )
      {
        case APP_RANGING_CONFIG:
          RngStatus = RNG_INIT;
          ModulationParams.PacketType = PACKET_TYPE_LORA;
          PacketParams.PacketType     = PACKET_TYPE_LORA;

          ModulationParams.PacketType = PACKET_TYPE_LORA;
          PacketParams.PacketType = PACKET_TYPE_LORA;
          ModulationParams.Params.LoRa.SpreadingFactor = LORA_SF10;
          ModulationParams.Params.LoRa.Bandwidth = APP_CONFIG_BANDWIDTH;
          ModulationParams.Params.LoRa.CodingRate = LORA_CR_4_5;
          PacketParams.Params.LoRa.PreambleLength = 12;
          PacketParams.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH;
          PacketParams.Params.LoRa.PayloadLength = 9;
          PacketParams.Params.LoRa.Crc = LORA_CRC_ON;
          PacketParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;

          Radio.SetPacketType( ModulationParams.PacketType );
          Radio.SetModulationParams( &ModulationParams );
          Radio.SetPacketParams( &PacketParams );
          Radio.SetRfFrequency( 2402000000UL );
          RX_LED = 1;
          // use listen mode here instead of rx continuous
          Radio.SetRx( ( TickTime_t ) { RADIO_TICK_SIZE_1000_US, 0xFFFF } );
          AppState = APP_IDLE;
          break;

        case APP_RNG:
          if( SendNext == true )
          {
            SendNext = false;
            MeasuredChannels++;
            if( MeasuredChannels <= RngRequestCount )
            {
              Radio.SetRfFrequency( Channels[CurrentChannel] );
              RX_LED = 1;

              CurrentChannel++;
              if( CurrentChannel >= CHANNELS )
              {
                CurrentChannel -= CHANNELS;
              }
              AppState = APP_IDLE;
              Radio.SetRx( ( TickTime_t ){ RADIO_TICK_SIZE_1000_US, RngReqDelay } );
            }
            else
            {
              Radio.SetStandby( STDBY_RC );
              SendNextPacket.detach( );
              RngStatus = RNG_VALID;
              AppState = APP_RANGING_CONFIG;
            }
          }
          break;

        case APP_RANGING_DONE:
          RX_LED = 0;
          CntPacketRxOK++;
          AppState = APP_RNG;
          break;

        case APP_RANGING_TIMEOUT:
          RX_LED = 0;
          AppState = APP_RNG;
          break;

        case APP_RX:
          RX_LED = 0;
          if( RngStatus == RNG_INIT )
          {
            Radio.GetPayload( Buffer, &BufferSize, BUFFER_SIZE );
            Radio.GetPacketStatus( &PacketStatus );
            if( ( BufferSize > 0 ) && \
                        ( Buffer[0] == ( ( RngAddress >> 24 ) & 0xFF ) ) && \
                        ( Buffer[1] == ( ( RngAddress >> 16 ) & 0xFF ) ) && \
                        ( Buffer[2] == ( ( RngAddress >>  8 ) & 0xFF ) ) && \
                        ( Buffer[3] == (   RngAddress         & 0xFF ) ) )
            {
              RngFei    = Radio.GetFrequencyError( );
              RssiValue = PacketStatus.LoRa.RssiPkt;
              CntPacketTx++;
              CurrentChannel                                 = Buffer[4];
              RngAntenna      = Buffer[5];
              RngRequestCount = Buffer[6];
              wait_us( 10 );
              Buffer[4] = ( ( ( int32_t )RngFei ) >> 24 ) & 0xFF ;
              Buffer[5] = ( ( ( int32_t )RngFei ) >> 16 ) & 0xFF ;
              Buffer[6] = ( ( ( int32_t )RngFei ) >>  8 ) & 0xFF ;
              Buffer[7] = ( ( ( int32_t )RngFei ) & 0xFF );
              Buffer[8] = RssiValue;
              TX_LED = 1;
              Radio.SendPayload( Buffer, 9, ( TickTime_t ){ RADIO_TICK_SIZE_1000_US, RNG_COM_TIMEOUT } );
              AppState = APP_IDLE;
            }
            else
            {
              AppState = APP_RANGING_CONFIG;
            }
          }
          else
          {
            AppState = APP_RANGING_CONFIG;
          }
          break;

        case APP_TX:
          TX_LED = 0;
          if( RngStatus == RNG_INIT )
          {
            RngStatus = RNG_PROCESS;

            ModulationParams.PacketType = PACKET_TYPE_RANGING;
            PacketParams.PacketType     = PACKET_TYPE_RANGING;

            ModulationParams.Params.LoRa.SpreadingFactor = LORA_SF10;
            ModulationParams.Params.LoRa.Bandwidth = APP_CONFIG_BANDWIDTH;
            ModulationParams.Params.LoRa.CodingRate = LORA_CR_4_5;
            PacketParams.Params.LoRa.PreambleLength = 12;
            PacketParams.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH;
            PacketParams.Params.LoRa.PayloadLength = 10;
            PacketParams.Params.LoRa.Crc = LORA_CRC_ON;
            PacketParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;

            Radio.SetPacketType( ModulationParams.PacketType );
            Radio.SetModulationParams( &ModulationParams );
            Radio.SetPacketParams( &PacketParams );
            Radio.SetDeviceRangingAddress( RngAddress );
            Radio.SetRangingCalibration( RngCalib );
            Radio.SetTxParams( 13, RADIO_RAMP_20_US );
            CntPacketRxOK = 0;
            MeasuredChannels = 0;
            CntPacketRxKOSlave = 0;
            SendNextPacket.attach_us( &SendNextPacketEvent, RngReqDelay * 1000 );
            AppState = APP_RNG;
          }
          else
          {
            AppState = APP_RANGING_CONFIG;
          }
          break;

        case APP_RX_TIMEOUT:
          RX_LED = 0;
          AppState = APP_RANGING_CONFIG;
          break;

        case APP_RX_ERROR:
          RX_LED = 0;
          AppState = APP_RANGING_CONFIG;
          break;

        case APP_TX_TIMEOUT:
          TX_LED = 0;
          AppState = APP_RANGING_CONFIG;
          break;

        case APP_IDLE: // do nothing
          if( CntPacketRxKOSlave > DEMO_RNG_CHANNELS_COUNT_MAX )
          {
            CntPacketRxKOSlave = 0;
            RX_LED = 0;
            AppState = APP_RANGING_CONFIG;
            SendNextPacket.detach( );
          }
          break;

        default:
          AppState = APP_RANGING_CONFIG;
          SendNextPacket.detach( );
          break;
      }
    }
  }
}



// ************************     Radio Callbacks     ****************************
// *                                                                           *
// * These functions are called through function pointer by the Radio low      *
// * level drivers                                                             *
// *                                                                           *
// *****************************************************************************
void OnTxDone( void )
{
  AppState = APP_TX;
}

void OnRxDone( void )
{
  AppState = APP_RX;
}

void OnTxTimeout( void )
{
  AppState = APP_TX_TIMEOUT;
}

void OnRxTimeout( void )
{
  AppState = APP_RX_TIMEOUT;
}

void OnRxError( IrqErrorCode_t errorCode )
{
  AppState = APP_RX_ERROR;
}

void OnRangingDone( IrqRangingCode_t val )
{
  if( val == IRQ_RANGING_MASTER_VALID_CODE || val == IRQ_RANGING_SLAVE_VALID_CODE )
  {
    AppState = APP_RANGING_DONE;
  }
  else if( val == IRQ_RANGING_MASTER_ERROR_CODE || val == IRQ_RANGING_SLAVE_ERROR_CODE )
  {
    AppState = APP_RANGING_TIMEOUT;
  }
  else
  {
    AppState = APP_RANGING_TIMEOUT;
  }
}

void OnCadDone( bool channelActivityDetected )
{
}

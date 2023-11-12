
// Libraries includes
#include <arduino.h>
#include <FreeRTOS_SAMD21.h>
#include <thingProperties.h>
#include "ADS131M08.h"
#include "SAMD21turboPWM.h"
extern "C" {
#include <PostProcessing.h>
#include <rtwtypes.h>
}
// Type Defines and Constants
#define LED_PIN  6 //Led Pin: Typical Arduino Board
#define ERROR_LED_LIGHTUP_STATE  HIGH // the state that makes the led light up on your board, either low or high
#define SERIAL          SerialUSB //Sparkfun Samd21 Boards

// Function to get the amount of free memory
extern "C" char* sbrk(int incr);
int freeMemory() {
  char top;
  return &top - reinterpret_cast<char*>(sbrk(0));
}

// global variables Tasks
TaskHandle_t  Handle_CloudUpdate;
TaskHandle_t  Handle_LEDUpdate;
TaskHandle_t  Handle_PBUpdate;
TaskHandle_t  Handle_DACUpdate;

// LED Counters and declarations
uint16_t      LED_ON_Duration = 1;  //  Durée ou la LED est allumée en 100ms,        
uint16_t      LED_Blink_Number = 3; //  Nombre de clignotement
uint16_t      LED_Short = 6;        // Periode de clignoement courte: 10x100ms
uint16_t      LED_Long = 10*10;     // Periode de clignotement longue: 10x10x100ms secondes
uint16_t      LED_Long_Count = 1;   // Compteur qui ballaie la periode la plus long du clighotement, i.e 10sec... 
uint16_t      LED_Short_Count = 1;  // Compteur sur la petite periode 
uint16_t      LED_Blink_Count = 1;  // Nombre de blinks faits 
#define       LED_State1  1
#define       LED_State2  2
#define       LED_State3  3
void LEDHandler (void);
void LEDSetTimings(int16_t LED_State);

// Push semaphore and period
#define       PB_DigIN          21 //  3*100ms
#define       PB_ReadingPeriod  3 //  3*100ms
bool          PB_Debound_Table[3] =   {0, 0, 0};    //  Lecture de valure successive
int           PB_Debound_Counter = 1;   // Lectures de 1 à la longeure de PB_Debound_Table
bool          PB_Semaphore =    0;     //  Durée ou la LED est allumée en 100ms,        
void          PBHandler(void);

// ADC and DAC variables
//ADC SPI Parameters;
#define ADC_CS          14       // à corriger
#define ADC_CS1         6        // same as the ADC that is not used
#define ADC_XTAL_PIN    7        //
#define ADC_XTAL_VAL    2        // 2.048MHZ
#define ADC_DRDY_PIN    4        //
#define SPI_FREQ        2000000
#define RESET           2
ADS131M08 ADC_ADC131(ADC_CS, ADC_XTAL_PIN, ADC_DRDY_PIN, SPI_FREQ);


// DAC Parameters and waveforms
void DACHandler(void);
#define DAC_ANALOG_RESOLUTION 10
uint16_t DAC_Waveform[] = { 
    128,131,134,137,140,143,146,149,152,155,158,162,165,167,170,173,
    176,179,182,185,188,190,193,196,198,201,203,206,208,211,213,215,
    218,220,222,224,226,228,230,232,234,235,237,238,240,241,243,244,
    245,246,248,249,250,250,251,252,253,253,254,254,254,255,255,255,
    255,255,255,255,254,254,254,253,253,252,251,250,250,249,248,246,
    245,244,243,241,240,238,237,235,234,232,230,228,226,224,222,220,
    218,215,213,211,208,206,203,201,198,196,193,190,188,185,182,179,
    176,173,170,167,165,162,158,155,152,149,146,143,140,137,134,131,  
    128,124,121,118,115,112,109,106,103,100,97,93,90,88,85,82,
    79,76,73,70,67,65,62,59,57,54,52,49,47,44,42,40,
    37,35,33,31,29,27,25,23,21,20,18,17,15,14,12,11,
    10,9,7,6,5,5,4,3,2,2,1,1,1,0,0,0,
    0,0,0,0,1,1,1,2,2,3,4,5,5,6,7,9,
    10,11,12,14,15,17,18,20,21,23,25,27,29,31,33,35,
    37,40,42,44,47,49,52,54,57,59,62,65,67,70,73,76,
    79,82,85,88,90,93,97,100,103,106,109,112,115,118,121,124
};

/*Acquired Channels to be sebt to the Cloud */
uint32_t S_Lentgh = 4000;// Variable Cloud Number of Samples per channel
//int32_t *Samples_32b = (int32_t*)malloc(sizeof(int32_t)*S_Lentgh);   // Variable Cloud

int Commande = 0;  // Variable Cloud
static uint32_t DAC_loopCount = 0;
static uint32_t S_loopCount = S_Lentgh;
int32_t Samples_32b[8] = {}; 
int ADCSetting[] = {0b1111111100000111}; 



void ADC_SetParametres(ADS131M08 ADC_ADC131);
void ADC_Handler(ADS131M08 ADC_ADC131);

// Can use these function for RTOS delays
void myDelayUs(int us)
{
  vTaskDelay( us / portTICK_PERIOD_US );  
}
void myDelayMs(int ms)
{
  vTaskDelay( (ms * 1000) / portTICK_PERIOD_US );  
}
void myDelayMsUntil(TickType_t *previousWakeTime, int ms)
{
  vTaskDelayUntil( previousWakeTime, (ms * 1000) / portTICK_PERIOD_US );  
}



// Definition des tasks
static void CloudUpdate( void *pvParameters ) 
{
  SERIAL.println("Acquisition and Cloud Update: Started");
    while(1)
  {
    SERIAL.print("\nCloud Update\n");
    SERIAL.flush();
    //ADC_Handler(ADC_ADC131);    // Acquire Signals
    ArduinoCloud.update();
    myDelayMs(5000);
  }
  // delete ourselves.
  SERIAL.println("Cloud Update: Deleting");
  vTaskDelete( NULL );
}


static void LEDUpdate( void *pvParameters ) 
{
  SERIAL.println("LED Update: Started");

  while(1)
  {
    LEDHandler();
    myDelayMs(100);
  }
   // delete ourselves.
  SERIAL.println("Cloud Update: Deleting");
  vTaskDelete( NULL );
}


static void BPUpdate( void *pvParameters ) 
{
  SERIAL.println("Push Button reading: Started");

  while(1)
  {
    PBHandler();
    myDelayMs(100);
  }
}

static void DACUpdate( void *pvParameters ) 
{
  SERIAL.println("DAC update: Started");

  while(1)
  {
    DACHandler();
    myDelayMs(1);
  }
}

//Register Table 
uint16_t ADC_Registers_Val[49] = {
 ADS131_MODE_VAL, 
 ADS131_CLOCK_VAL,  // OSR=1024
 ADS131_GAIN1_VAL, 
 ADS131_GAIN2_VAL, 
 ADS131_CFG_VAL, 
 ADS131_THRSHLD_MSB_VAL, 
 ADS131_THRSHLD_LSB_VAL, 
 ADS131_CH0_CFG_VAL, 
 ADS131_CH0_OCAL_MSB_VAL, 
 ADS131_CH0_OCAL_LSB_VAL, 
 ADS131_CH0_GCAL_MSB_VAL, 
 ADS131_CH0_GCAL_LSB_VAL, 
 ADS131_CH1_CFG_VAL, 
 ADS131_CH1_OCAL_MSB_VAL, 
 ADS131_CH1_OCAL_LSB_VAL, 
 ADS131_CH1_GCAL_MSB_VAL, 
 ADS131_CH1_GCAL_LSB_VAL, 
 ADS131_CH2_CFG_VAL, 
 ADS131_CH2_OCAL_MSB_VAL, 
 ADS131_CH2_OCAL_LSB_VAL, 
 ADS131_CH2_GCAL_MSB_VAL, 
 ADS131_CH2_GCAL_LSB_VAL, 
 ADS131_CH3_CFG_VAL, 
 ADS131_CH3_OCAL_MSB_VAL, 
 ADS131_CH3_OCAL_LSB_VAL, 
 ADS131_CH3_GCAL_MSB_VAL, 
 ADS131_CH3_GCAL_LSB_VAL, 
 ADS131_CH4_CFG_VAL, 
 ADS131_CH4_OCAL_MSB_VAL, 
 ADS131_CH4_OCAL_LSB_VAL, 
 ADS131_CH4_GCAL_MSB_VAL, 
 ADS131_CH4_GCAL_LSB_VAL, 
 ADS131_CH5_CFG_VAL, 
 ADS131_CH5_OCAL_MSB_VAL, 
 ADS131_CH5_OCAL_LSB_VAL, 
 ADS131_CH5_GCAL_MSB_VAL, 
 ADS131_CH5_GCAL_LSB_VAL, 
 ADS131_CH6_CFG_VAL, 
 ADS131_CH6_OCAL_MSB_VAL, 
 ADS131_CH6_OCAL_LSB_VAL, 
 ADS131_CH6_GCAL_MSB_VAL, 
 ADS131_CH6_GCAL_LSB_VAL, 
 ADS131_CH7_CFG_VAL, 
 ADS131_CH7_OCAL_MSB_VAL, 
 ADS131_CH7_OCAL_LSB_VAL, 
 ADS131_CH7_GCAL_MSB_VAL, 
 ADS131_CH7_GCAL_LSB_VAL, 
 ADS131_REGMAP_CRC_VAL,
 ADS131_RESERVED_VAL
};


uint16_t ADC_Registers_Add[49] = {
 ADS131_MODE, 
 ADS131_CLOCK,
 ADS131_GAIN1, 
 ADS131_GAIN2, 
 ADS131_CFG, 
 ADS131_THRSHLD_MSB, 
 ADS131_THRSHLD_LSB, 
 ADS131_CH0_CFG, 
 ADS131_CH0_OCAL_MSB, 
 ADS131_CH0_OCAL_LSB, 
 ADS131_CH0_GCAL_MSB, 
 ADS131_CH0_GCAL_LSB, 
 ADS131_CH1_CFG, 
 ADS131_CH1_OCAL_MSB, 
 ADS131_CH1_OCAL_LSB, 
 ADS131_CH1_GCAL_MSB, 
 ADS131_CH1_GCAL_LSB, 
 ADS131_CH2_CFG, 
 ADS131_CH2_OCAL_MSB, 
 ADS131_CH2_OCAL_LSB, 
 ADS131_CH2_GCAL_MSB, 
 ADS131_CH2_GCAL_LSB, 
 ADS131_CH3_CFG, 
 ADS131_CH3_OCAL_MSB, 
 ADS131_CH3_OCAL_LSB, 
 ADS131_CH3_GCAL_MSB, 
 ADS131_CH3_GCAL_LSB, 
 ADS131_CH4_CFG, 
 ADS131_CH4_OCAL_MSB, 
 ADS131_CH4_OCAL_LSB, 
 ADS131_CH4_GCAL_MSB, 
 ADS131_CH4_GCAL_LSB, 
 ADS131_CH5_CFG, 
 ADS131_CH5_OCAL_MSB, 
 ADS131_CH5_OCAL_LSB, 
 ADS131_CH5_GCAL_MSB, 
 ADS131_CH5_GCAL_LSB, 
 ADS131_CH6_CFG, 
 ADS131_CH6_OCAL_MSB, 
 ADS131_CH6_OCAL_LSB, 
 ADS131_CH6_GCAL_MSB, 
 ADS131_CH6_GCAL_LSB, 
 ADS131_CH7_CFG, 
 ADS131_CH7_OCAL_MSB, 
 ADS131_CH7_OCAL_LSB, 
 ADS131_CH7_GCAL_MSB, 
 ADS131_CH7_GCAL_LSB, 
 ADS131_REGMAP_CRC,
 ADS131_RESERVED
};

// ************************************SetUp***********************************************
// Set up
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

void setup() 
{

  // OS Setup
  Serial.begin(9600);
  delay(1000); // prevents usb driver crash on startup, do not omit this
  while (!Serial) ;  // Wait for serial terminal to open port before starting program
  const char message1[] PROGMEM = "";
  const char message2[] PROGMEM = "******************************";
  const char message3[] PROGMEM = "        Program start         ";
  const char message4[] PROGMEM = "******************************";
  Serial.println(F(message1));
  Serial.println(F(message2));
  Serial.println(F(message3));
  Serial.println(F(message4));
  Serial.flush();

  // Push Button setup
  pinMode(PB_DigIN,INPUT);
  // ADC Setup

    /* Start the clock for the ADC*/
    ADC_ADC131.init();
    ADC_SetParametres(ADC_ADC131);
    pinMode(ADC_CS1, OUTPUT);          // Disable the CS of ADC1, use only ADC2 in the Schematics
    digitalWrite(ADC_CS1, HIGH);

    //  Set up the DAC
      pinMode( A0, OUTPUT );
      analogWriteResolution( DAC_ANALOG_RESOLUTION );
 
  //Initializing the feature extraction
   Serial.println("I am before postprocessingInit");
  PostProcessing_initialize();
     Serial.println("I am after postprocessingInit");
  // Set the led the rtos will blink when we have a fatal rtos error
  // RTOS also Needs to know if high/low is the state that turns on the led.
  // Error Blink Codes:
  //    3 blinks - Fatal Rtos Error, something bad happened. Think really hard about what you just changed.
  //    2 blinks - Malloc Failed, Happens when you couldn't create a rtos object. 
  //               Probably ran out of heap.
  //    1 blink  - Stack overflow, Task needs more bytes defined for its stack! 
  //               Use the taskMonitor thread to help gauge how much more you need
  vSetErrorLed(LED_PIN, ERROR_LED_LIGHTUP_STATE);

  // sets the serial port to print errors to when the rtos crashes
  vSetErrorSerial(&Serial);

  // Create the threads that will be managed by the rtos
  //  Amine
   xTaskCreate(CloudUpdate,     "Cloud Update",       512, NULL, tskIDLE_PRIORITY + 1, &Handle_CloudUpdate);
   Serial.println("Cloud Task created");
   xTaskCreate(LEDUpdate,       "LED Update",         256,  NULL, tskIDLE_PRIORITY + 2, &Handle_LEDUpdate);
   Serial.println("LED Task created");
   xTaskCreate(BPUpdate,        "BP Update",          256,  NULL, tskIDLE_PRIORITY + 3, &Handle_PBUpdate);
   Serial.println("PB Task created");
   xTaskCreate(DACUpdate,       "DAC Update",         256,  NULL, tskIDLE_PRIORITY + 4, &Handle_DACUpdate);
   Serial.println("DAC Task created");

  // Cloud Setup
  // Amine
  LEDSetTimings(LED_State2);
  initProperties();
  Serial.println("Wifi Connected");
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
  Serial.println("Cloud COnnected");

//Calibration Parameters
Serial.println("I am before Calibration");
rtU.CalI1 = 325755; 
Serial.println(rtU.CalI1);
rtU.CalI2 = (1.25)/(2^23);
rtU.CalI3 = (1.25)/(2^23);
rtU.CalI4 = (1.25)/(2^23);
rtU.CalV1 = (1.25*5004.7)/(4.7*(2^23)); //Multiply 5004.7/
rtU.CalV2 = (1.25*5004.7)/(4.7*(2^23)); 
rtU.CalV3 = (1.25*5004.7)/(4.7*(2^23)); 
rtU.CalV4 = (1.25*5004.7)/(4.7*(2^23)); 
   Serial.println("I am after calibration");

  // Start the RTOS, this function will never return and will schedule the tasks.
  vTaskStartScheduler();

  // error scheduler failed to start
  // should never get here
  while(1)
  {
	  SERIAL.println("Scheduler Failed! \n");
	  SERIAL.flush();
	  delay(1000);
  }

}

//*****************************************************************
// This is now the rtos idle loop
// No rtos blocking functions allowed!
//*****************************************************************
void loop() 
{
    // Optional commands, can comment/uncomment below
    SERIAL.print("."); //print out dots in terminal, we only do this when the RTOS is in the idle state
    SERIAL.flush();
    delay(1000); //delay is interrupt friendly, unlike vNopDelayMS
}

void onCloudPBChange()  {
      //Synchronisation Cloud
      ADC_Handler(ADC_ADC131);    // Acquire Signals
      ArduinoCloud.update();      // Send data back
      digitalWrite(LED_BUILTIN,HIGH);
      SERIAL.print("\n Cloud Updat base on Cloud\n");
      SERIAL.flush();
      delay(2000);
      digitalWrite(LED_BUILTIN, LOW);
}

void onCloudADCParamChange()  {

}
//*****************************************************************


// Gestion de la LED;
// Mise à jour des frequences de clignotement en fonction de l'état de la LED
// incrémentation des compteurs et application de l'état d la LED,

void LEDHandler (void){
  LED_Long_Count = LED_Long_Count+1;
    if (LED_Long_Count==LED_Long){  // Incrémenter le compteur LED et le remettre à zero à la buttée
      LED_Long_Count = 1;
      LED_Blink_Count = 1;}
    else{}   
    if (LED_Blink_Count<=LED_Blink_Number){
      if(LED_Short_Count <= LED_ON_Duration){digitalWrite(LED_PIN, HIGH);}else{digitalWrite(LED_PIN, LOW);}
      LED_Short_Count = LED_Short_Count+1;
      if (LED_Short_Count==LED_Short)
      {
        LED_Short_Count = 1;
        LED_Blink_Count = LED_Blink_Count+1;
      } else{}
    } else {}
}

void LEDSetTimings(int16_t LED_State){
  switch (LED_State)
  {
  case LED_State1:
      LED_ON_Duration = 1;  //  Durée ou la LED est allumée en 100ms,        
      LED_Blink_Number = 1; //  Nombre de clignotement
      LED_Short = 6;        // Periode de clignoement courte: 10x100ms
      LED_Long = 10*10;     // Periode de clignotement longue: 10x10x100ms secondes
    break;
   case LED_State2:
      LED_ON_Duration = 1;  //  Durée ou la LED est allumée en 100ms,        
      LED_Blink_Number = 2; //  Nombre de clignotement
      LED_Short = 6;        // Periode de clignoement courte: 10x100ms
      LED_Long = 10*10;     // Periode de clignotement longue: 10x10x100ms secondes
    break;

   case LED_State3:
      LED_ON_Duration = 1;  //  Durée ou la LED est allumée en 100ms,        
      LED_Blink_Number = 3; //  Nombre de clignotement
      LED_Short = 6;        // Periode de clignoement courte: 10x100ms
      LED_Long = 10*10;     // Periode de clignotement longue: 10x10x100ms secondes
    break;

  default:
    break;
  }
}

// Push Button handler
void PBHandler(void){
  int PB_Semaphore_TMP =    1;      // une variable tmp qui reflete la table des lecture,  
  PB_Debound_Table[PB_Debound_Counter] = digitalRead(PB_DigIN);    //  Le bouton est cablé en logique négative
  PB_Debound_Counter = PB_Debound_Counter+1;
  if (PB_Debound_Counter == sizeof(PB_Debound_Table)+1){PB_Debound_Counter=1;}else{}  // RAZ une fois le conteur au bout
  for (uint16_t i = 1; i <= sizeof(PB_Debound_Table); i++)
  {
    PB_Semaphore_TMP = PB_Semaphore_TMP * PB_Debound_Table[i];
  }
  if (PB_Semaphore_TMP==1){
      PB_Semaphore=1;
      
      //Synchronisation Cloud
      ADC_Handler(ADC_ADC131);    // Acquire Signals
   
      //Amine
      ArduinoCloud.update();      // Send data back
      digitalWrite(LED_BUILTIN,HIGH);
      SERIAL.print("\n Cloud Update base on PB\n");
      SERIAL.flush();
      delay(2000);
      digitalWrite(LED_BUILTIN, LOW);
    }else{PB_Semaphore=0;}
}

void ADC_SetParametres(ADS131M08 ADC_ADC131){

  /* Reset the ADC */
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, LOW);
  delay(10);
  digitalWrite(RESET, HIGH);
   delay(10);

  /* Write Setting Registers*/
  ADC_ADC131.writeReg(ADS131_CLOCK,ADCSetting[0]); //Clock register (page 55 in datasheet)
  //ADC_ADC131.setGain(1);
  //ADC_ADC131.globalChop(true,2);
  /* Read and check Register values*/
  //uint16_t clkreg = ADC_ADC131.readReg(ADS131_CLOCK);
  //uint16_t gainreg = ADC_ADC131.readReg(ADS131_GAIN1);
  Serial.print("CLOCK: ");
  //Serial.println(clkreg,BIN);
  };

  void ADC_Handler(ADS131M08 ADC_ADC131){
      S_loopCount = S_Lentgh;
      

      while (S_loopCount > 0)
      { 
        if(digitalRead(ADC_DRDY_PIN)) {
            //Serial.println(S_loopCount);
            // Read Data
            ADC_ADC131.readAllChannels(Samples_32b);
           
            rtU.I1 = Samples_32b[0];
            //Serial.println(rtU.I1 );
            rtU.I2 = Samples_32b[1];
            rtU.I3 = Samples_32b[2];
            rtU.I4 = Samples_32b[3];
            rtU.V1 = Samples_32b[4];
            rtU.V2 = Samples_32b[5];
            rtU.V3 = Samples_32b[6];
            rtU.V4 = Samples_32b[7];
          

            PostProcessing_step();
            //Serial.println(Samples_32b[0]);          // Print data in channel 1.
        
            S_loopCount = S_loopCount-1;
            if (S_loopCount==0)
            {
            Serial.println("This is the RMS Values:"); 
            Serial.println(rtY.RMSDisplay[0]); 
            Serial.println(rtY.RMSDisplay[1]); 
            Serial.println(rtY.RMSDisplay[2]); 
            Serial.println(rtY.RMSDisplay[3]); 

            Serial.println("This is the Active Power Values:"); 
            Serial.println(rtY.PowerDisplay[0]); 
            Serial.println(rtY.PowerDisplay[2]); 
            Serial.println(rtY.PowerDisplay[4]); 


            Serial.println("This is the Reactive Power Values:"); 
            Serial.println(rtY.PowerDisplay[1]); 
            Serial.println(rtY.PowerDisplay[3]); 
            Serial.println(rtY.PowerDisplay[5]); 
            
            }
          }
          else {}
          }
      }


void DACHandler(void){
            // Output to the DAC
            analogWrite( A0, DAC_Waveform[DAC_loopCount]);
            DAC_loopCount = DAC_loopCount + 1;
            if (DAC_loopCount==255) {DAC_loopCount=0;}
            
};
/*Modified by bratindustries.net feb 2023 striped out unnessary parts, just reads soc, cell voltages, temps
 * Modified by Bryan Inkster May 2022 to do basic decodes from Model 3 battery pack   *************************************************************
 copyright 2016 Jack Rickard and Collin Kidder.  GPL license.  Use it for whatever you like, but attribute it.

  This program uses the EVTVDue Microcontroller and is compatible with Arduino Due  Use BOARD select Arduino Due (Native USB port).
It should work with any Arduino Due with CAN but relies heavily on Collin Kidder's due_can library http://github.com/collin80
jack@evtv.me

*/

#include <due_can.h>  //https://github.com/collin80/due_can

template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; } //Sets up serial streaming Serial<<someshit;
 

//*********GENERAL VARIABLE   DATA ******************

CAN_FRAME outframe;  //A structured variable according to due_can library for transmitting CAN data.

//Other ordinary variables
float Version=1.50;
uint16_t page=300;    //EEPROM page to hold variable data.  We save time by setting up a structure and saving a large block
int i;
unsigned long elapsedtime, time228,timestamp,startime, lastime;  //Variables to compare millis for timers
boolean debug=false;
boolean testing=false;
uint8_t logcycle=0;
uint8_t framecycle=0;
uint16_t transmitime=30;
float carvoltage=0.0f;
float minvolts;
float maxvolts;
float mintemp;
float maxtemp;
float soc;
float amps;
float volts;
uint8_t minvno;
uint8_t maxvno;
float cell[97];



//******* END OF GENERAL VARIABLE DATA***********



//********************SETUP FUNCTION*******I*********
/*
 * The SETUP function in the Arduino IDE simply lists items that must be performed prior to entering the main program loop.  In this case we initialize Serial 
 * communications via the USB port, set an interrupt timer for urgent outbound frames, zero our other timers, and load our EEPROM configuration data saved during the 
 * previous session.  If no EEPROM data is found, we initialize EEPROM data to default values
 * 
 */
void setup() 
  {
    SerialUSB.begin(115200);  //Initialize our USB port which will always be redefined as SerialUSB to use the Native USB port tied directly to the SAM3X processor.  
     lastime=startime=timestamp=millis();  //Zero our other timers
     initializeCAN();     
     delay(5000);
   
     SerialUSB<<"\n\n Startup successful. Tesla Model S Battery Read Program "<<Version<<"\n\n";
      
  }
   
//********************END SETUP FUNCTION*******I*********

//********************MAIN PROGRAM LOOP*******I*********

void loop()
{ 
   
  if(millis()-lastime >transmitime)  //Nominally set for 120ms - do stuff on 120 ms non-interrupt clock
    {
     lastime=millis();        //Zero our timer     
     if(testing)
      {
       
       send332frame();
       send332frame2();
      }
      
    
      
       printBattery();    
    }     
            
}
//********************END MAIN PROGRAM LOOP*******I*********



//******************** USB SERIAL OUTPUT TO SCREEN ****************
/*  These functions are used to send data out the USB port for display on an ASCII terminal screen.  Menus, received frames, or variable status for example
 *   
 */


void printFrame(CAN_FRAME *frame,int sent)
{ 
  char buffer[300];
  sprintf(buffer,"msgID 0x%03X; %02X; %; %02X; %02X; %02X; %02X; %02X; %02X  %02d:%02d:%02d.%04d\n", frame->id, frame->data.bytes[0], 
  frame->data.bytes[1],frame->data.bytes[2], frame->data.bytes[3], frame->data.bytes[4], frame->data.bytes[5], frame->data.bytes[6],
  frame->data.bytes[7], hours(), minutes(), seconds(), milliseconds());
  
   if(sent)SerialUSB<<"Sent ";
    else SerialUSB<<"Received ";       
   SerialUSB<<buffer<<"\n";
}

void printBattery()
{
  /* This function prints an ASCII statement out the SerialUSB port summarizing various data from program variables.  Typically, these variables are updated
   *  by received CAN messages that operate from interrupt routines. This routine also time stamps the moment at which it prints out.
   */
   char buffer[300];
  

  for(int i=1;i<96;i=i+6)
    {       
          sprintf(buffer,"  Cell %2d : %1.3fv  Cell %2d : %1.3fv  Cell %2d : %1.3fv  Cell %2d : %1.3fv  Cell %2d : %1.3fv  Cell %2d : %1.3fv  \n",i,cell[i],
          i+1,cell[i+1],i+2,cell[i+2],i+3,cell[i+3],i+4,cell[i+4],i+5,cell[i+5]);
          SerialUSB<<buffer;
     }
    
    
  
      SerialUSB<<"\n";

      
 
       
      SerialUSB.print(" ");
      SerialUSB.print("     SOC = ");
      SerialUSB.print(soc,1);
      SerialUSB.print("KWh     Volts = ");      
      SerialUSB.print(volts,1);
      SerialUSB.print("     Amps = ");      
      SerialUSB.println(amps,1);


      
      
      SerialUSB.print("  max Cell No ");
      SerialUSB.print(maxvno);
      SerialUSB.print(" = ");       
      SerialUSB.print(maxvolts,3);
      SerialUSB.print("     min Cell No ");
      SerialUSB.print(minvno);
      SerialUSB.print(" = ");      
      SerialUSB.print(minvolts,3);   

      SerialUSB.print("     max Cell Temp = ");
      SerialUSB.print(maxtemp,1);
      SerialUSB.print("     min Cell Temp = ");
      SerialUSB.println(mintemp,1);

                           
}


int milliseconds(void)
{
  int milliseconds = (int) (micros()/100) %10000 ;
  return milliseconds;
}


 int seconds(void)
{
    int seconds = (int) (micros() / 1000000) % 60 ;
    return seconds;
}


int minutes(void)
{
    int minutes = (int) ((micros() / (1000000*60)) % 60);
    return minutes;
}

    
int hours(void)
{    
    int hours   = (int) ((micros() / (1000000*60*60)) % 24);
    return hours;
}  



//******************** END USB SERIAL OUTPUT TO SCREEN ****************


//******************** CAN ROUTINES ****************************************
/* This section contains CAN routines to send and receive messages over the CAN bus
 *  INITIALIZATION routines set up CAN and are called from program SETUP to establish CAN communications.
 *  These initialization routines allow you to set filters and interrupts.  On RECEIPT of a CAN frame, an interrupt stops execution of the main program and 
 *  sends the frame to the specific routine used to process that frame by Message ID. Once processed, the main program is resumed.
 *  
 */

void initializeCAN()
{
  //Initialize CAN bus 0 or 1 and set filters to capture incoming CAN frames and route to interrupt service routines in our program.
 
     pinMode(50,OUTPUT);
     if (Can0.begin(500000,50)) 
        {
          SerialUSB.println("Using CAN0 - initialization completed.\n");
          Can0.setNumTXBoxes(3);
          Can0.setRXFilter(0, 0x332, 0x7FF, false);       //min/max cell volts and temps
          Can0.setCallback(0, handle332frame);
          Can0.setRXFilter(2, 0x401, 0x7FF, false);       //cell voltages
          Can0.setCallback(2, handle401frame);
          Can0.setRXFilter(3, 0x352, 0x7FF, false);       // SOC
          Can0.setCallback(3, handle352frame);
          Can0.setRXFilter(4, 0x132, 0x7FF, false);       // battery amps/volts
          Can0.setCallback(4, handle132frame);  
          Can0.setGeneralCallback(handleCANframe);              
          }
        else SerialUSB.println("CAN0 initialization (sync) ERROR\n");
    
}   

void handleCANframe(CAN_FRAME *frame)
//This routine handles CAN interrupts from a capture of any other CAN frames from the inverter not specifically targeted.  
//If you add other specific frames, do a setRXFilter and CallBack mailbox for each and do a method to catch those interrupts after the fashion
//of this one.
{  
    //This routine basically just prints the general frame received IF debug is set.  Beyond that it does nothing.
    
    if(debug) printFrame(frame,0); //If DEBUG variable is 1, print the actual message frame with a time stamp showing the time received.      
}






//***********  additional routines for Model 3 battery  *****************************************************************************




void handle332frame(CAN_FRAME *frame)
//This routine handles CAN interrupts from 0x332 CAN frame  = max/min cell Volts and Temps
{   
  uint16_t wolts;
  uint8_t mux;


    mux=(frame->data.bytes[0]);           //check mux
    mux=mux&0x03;

    
  if (mux==1)         // then pick out max/min cell volts
    {    
    wolts=(word(frame->data.bytes[1],frame->data.bytes[0]));
    wolts >>=2;
    wolts = wolts&0xFFF;  
    maxvolts=wolts/500.0f;

    wolts=(word(frame->data.bytes[3],frame->data.bytes[2]));
    wolts = wolts&0xFFF;  
    minvolts=wolts/500.0f;

    wolts=(frame->data.bytes[4]);
    maxvno=1+(wolts&0x007F);
    
    wolts=(frame->data.bytes[5]);
    minvno=1+(wolts&0x007F);
       
    }


  if (mux==0)         // then pick out max/min temperatures
  {
    wolts=(byte(frame->data.bytes[2]));
    maxtemp=(wolts*0.5f)-40;    
    
    wolts=(byte(frame->data.bytes[3])); 
    mintemp=(wolts*0.5f)-40;    
  }
         
    if(debug)printFrame(frame,0); //If DEBUG variable is 1, print the actual message frame with a time stamp showing the time received.    
}


//***************************************************************************
void handle401frame(CAN_FRAME *frame)
//This routine handles CAN interrupts from 0x401 CAN frame = cell voltages
{
  uint16_t wolts;
  uint8_t mux;


    mux=(frame->data.bytes[0]);           //get mux
 //   mux=mux&0x03;

    wolts=(frame->data.bytes[1]);         //status byte must be 0x02A

    if (wolts==0x02A)
    {
      wolts=(word(frame->data.bytes[3],frame->data.bytes[2]));
      cell[1+mux*3]=wolts/10000.0f;
       wolts=(word(frame->data.bytes[5],frame->data.bytes[4]));
      cell[2+mux*3]=wolts/10000.0f;
       wolts=(word(frame->data.bytes[7],frame->data.bytes[6]));
      cell[3+mux*3]=wolts/10000.0f;           
    }

}



//************************************************************************************************
void handle352frame(CAN_FRAME *frame)
//This routine handles CAN interrupts from 0x352 CAN frame  = state of charge SOC
{   
  uint16_t wolts;

    wolts=(word(frame->data.bytes[4],frame->data.bytes[5]));
    wolts >>=1;
    wolts = wolts&0x03FF;  
    soc=wolts/10.0f;
}


//************************************************************************************************
void handle132frame(CAN_FRAME *frame)
//This routine handles CAN interrupts from 0x132 CAN frame  = battery amps / volts
{   
  uint16_t wolts;
  int16_t wamps;
  
    wolts=(word(frame->data.bytes[1],frame->data.bytes[0]));  
    volts=wolts/100.0f;

    wamps=(word(frame->data.bytes[3],frame->data.bytes[2])); 
    amps=(wamps)/10.0f;  
}





//*********************************************************************************************
void send332frame()
{
          outframe.id = 0x332;            // Set our transmission address ID
        outframe.length = 6;            // Data payload 6 bytes
        outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bi
        outframe.rtr=1;                 //No request
        outframe.data.bytes[0]=0x81;
        outframe.data.bytes[1]=0x1F;  
        outframe.data.bytes[2]=0xDD;
        outframe.data.bytes[3]=0x07;
        outframe.data.bytes[4]=0x10;
        outframe.data.bytes[5]=0x48;
       
        //if(debug) 
        {printFrame(&outframe,1);} //If the debug variable is set, show our transmitted frame
                      
        Can0.sendFrame(outframe);    //Mail it
         
handle332frame(&outframe);
}


//*******************************************************************************************
void send332frame2()
{
          outframe.id = 0x332;            // Set our transmission address ID
        outframe.length = 6;            // Data payload 6 bytes
        outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bi
        outframe.rtr=1;                 //No request
        outframe.data.bytes[0]=0x3C;
        outframe.data.bytes[1]=0x0C;  
        outframe.data.bytes[2]=0x85;
        outframe.data.bytes[3]=0x80;
        outframe.data.bytes[4]=0x89;
        outframe.data.bytes[5]=0x86;
       
        //if(debug) 
        {printFrame(&outframe,1);} //If the debug variable is set, show our transmitted frame
                      
        Can0.sendFrame(outframe);    //Mail it
         
handle332frame(&outframe);
}

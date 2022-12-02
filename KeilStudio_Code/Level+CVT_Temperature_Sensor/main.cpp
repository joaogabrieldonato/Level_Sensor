#include "mbed.h"
#include <cmath>
#include <cstdint>
#include "CANMsg.h"
//#include "TextLCD.h"
#include "definitions.h"
#include "BmuDefs.h"
#include "MLX90614.h"
#include <math.h>

#define default_addr   (0x00)

//Variables Iniciation
uint8_t MeasureCVTtemperature;
double Fuel_Level;



/*Pins*/
AnalogIn a0 (PB_1); 
DigitalIn Led(PC_13);   
 


/* Communication protocols */
I2C i2c(PB_7, PB_6);   //sda,scl
CAN can(PB_8, PB_9, 1000000);
Serial serial(PA_9, PA_10, 115200);

//I2C i2c_lcd(PB_7, PB_6); // SDA, SCL
//TextLCD_I2C lcd(&i2c_lcd, 0x4E, TextLCD::LCD16x2); // I2C bus, PCF8574 Slaveaddress, LCD Type, Device Type 


/*General Libraries*/
MLX90614 mlx(&i2c);

/* General functions*/
void setupInterrupts();
double CVT_Temperature();
float moving_averages();
double mode();

/* Debug variables */
Timer t;
bool buffer_full = false;
unsigned int t0, t1;

/* Mbed OS tools */
Thread eventThread;
EventQueue queue(1024);
Ticker ticker66mHz;
Ticker ticker80mHz;
CircularBuffer <state_t, BUFFER_SIZE> state_buffer;
state_t current_state = IDLE_ST;

/* Interrupt services routine */
void ticker66mHzISR();
void ticker80mHzISR();

// main() runs in its own thread in the OS

int main()
{

    /* Main variables */
    CANMsg txMsg;
    setupInterrupts();  

    while (true) {
                 
       if (state_buffer.full())
        {
            buffer_full = true;
            //Led = 0;
            state_buffer.pop(current_state);
        }
       else
        {
            //Led = 1;
            buffer_full = false;
            if (!state_buffer.empty())
                state_buffer.pop(current_state);
            else
                current_state = IDLE_ST;
        }

      switch(current_state) {
        
        case IDLE_ST:
           
            break;

        case FUEL_LEVEL_ST:

             moving_averages();

             Fuel_Level = mode(); 
             thread_sleep_for(200);

            txMsg.clear(FUEL_LEVEL_ID);
            txMsg << Fuel_Level;

            can.write(txMsg);
            
            break;            

        case CVTtemperature_ST:
            
            
            MeasureCVTtemperature = (uint8_t) CVT_Temperature(); 
            if(MeasureCVTtemperature < 1 ){
                break;
            }
            txMsg.clear(TempCVT_ID);
            txMsg << MeasureCVTtemperature;
            
            if (can.write(txMsg)){
               //Led =!Led;
            }           
            
            break;       

        
        }
    
     
    
        //print na serial MED 
        
        serial.printf("temp_med Ambiente = ");
        serial.printf("%d",MeasureCVTtemperature);
        serial.printf("  Fuel Percentage = ");
        serial.printf("%2f", Fuel_Level);
        serial.printf("\n");

        ThisThread::sleep_for(200);

    }
 }

/* General functions */
void setupInterrupts()
{
    ticker66mHz.attach(&ticker66mHzISR, 6);
    ticker80mHz.attach(&ticker80mHzISR, 8);
}


void ticker66mHzISR()
{
    state_buffer.push(CVTtemperature_ST);
}


void ticker80mHzISR()
{
    state_buffer.push(FUEL_LEVEL_ST);
}

double CVT_Temperature(){
  int i,j;
    double AverageObjectTemp, AverageEnviromentTemp = 0.0;
    double temp_amb,med_amb,x_amb;
    double temp_obj,med_obj,x_obj; 

  //teste comunicação i2c
  char ucdata_write[2];

  ucdata_write [0] = 0;
  ucdata_write [1] = 0;

  if (!i2c.write((default_addr|0x00), ucdata_write, 1, 0)){// Check for ACK from i2c Device

    for(j = 0; j < (CVTsample); j++){ 

        for(i = 0; i < (CVTsample) ; i++){
                      
            // temp_amb = mlx.read_temp(0);
           

                temp_obj = mlx.read_temp(1);
                x_obj += temp_obj;
                med_obj = x_obj/ (double)CVTsample;
    
                if(med_obj > AverageObjectTemp){
                AverageObjectTemp = med_obj;
                 }
            }
        }
    }
  else {
        AverageObjectTemp = 0;
    }

    //return value;
    return AverageObjectTemp/(double)CVTsample; // I don't know why we need divide by sample, but works.

}

float moving_averages(){
    // 2 filtros no formato de médias móveis

    int i,j; 
    float V, Rx, Rx_acc, R, R_acc, r_final = 0.0;
    int n = 30; // Tamanho do Vetor
    int R_average[n]; // vetor 
    uint16_t read = 0;

    for (i = 0; i < (sample); i++) {

      read = a0.read_u16();       // Saída no Serial
      V = (read * ADCVoltageLimit) / 65535.0; // V = Voltagem da Boia
      Rx = (120 * V) / (3.3 - V); // Rx = Resistência da Boia
      Rx_acc += Rx;               // Rx_acc = acumulador do filtro 1
    }

      R = Rx_acc / (sample); // faz a média
      Rx_acc = 0.0; // reseta o acc 
      thread_sleep_for(200);
    
    for (int i = 1; i < n; i++) // mover pelo vetor (shift left)
    {
      R_average[0] = R;
      R_average[i] = R_average[i-1];
    }
    
    
    
    for (int j = 0; j < n; j++) // soma os valores no acumulador
    {
      R_acc += R_average[j]; 
    }
    
    r_final = R_acc / 30;  // faz a média
    R_acc = 0.0;   // reseta o acumulador 
    
    return r_final;
}

double mode(){ 
    // encontra a moda

    int T = 20, cont[T], i, j;
    float R_final_average[T];
    float moda, R_final, R_final_mode;
    float count = 15.00; // o valor deve aparecer 15 vezes em 20 para ser lido
    double fuel_level;
    

    for (i = 1; i < T; i++) // mover enquanto acrescenta ao vetor (shift left)
    {
      R_final = moving_averages();
      R_final_average[0] = R_final;
      R_final_average[i] = R_final_average[i-1];
    }
  
    for(i = 0; i < T; i++){                 
        
        for(j = i+1; j < T; j++){
        	
			if(R_final_average[i]==R_final_average[j]){  // compara os valores
                cont[i]++;
				
                if(cont[i] > count){  // se o valor aparecer em 15 das 20 vezes
                    moda = R_final_average[i];
			    }
        	 
            }
      
        }
    cont[i] = 0; // reseta a lista
    }
  R_final_mode = moda;
  
 // respostas possíveis

  if (R_final_mode >= 175) // Acima de 60% no tanque (início da medição), 5.68L
    {
        fuel_level = 100.00;
    } 
  else if (115 <= R_final_mode && R_final_mode < 175) // Cerca de 50% do tanque, 2.84L
    {
        fuel_level = 50.00;
    }
  else if (75 <= R_final_mode && R_final_mode < 115) // Cerca de 35% do tanque, 2L
    { 
        fuel_level = 35.00;
    } 
  else if (35 <= R_final_mode && R_final_mode < 75) // Cerca de 25% do tanque, 1.42L
    {
        fuel_level = 25.00;
    }
  else // Alcance da reserva, cerca de 18% do tanque, 1L
    {
        fuel_level = 18.00;
    }
    return fuel_level;
}
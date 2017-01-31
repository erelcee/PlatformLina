/*  TURNTEST_PLATFORM_LINA_31_JAN_2017
 *   
 *  Joystick_Speed_BLDC_CONTROLLER.ino
 *   
 *  ARDUINO UNO - Joystick_Speed_Sensored_BLDC_Controller
 *  With Chinese BLDC sensored controller, Arduino Uno and Joystick.
 *  ( with de Joystick I produce PWM signals - R and L - that goes to a low pass filter to make a  Voltage 
 *  from 0.2V to 5V, and connect this to the throttle input of the controller.)
 *  To drive Rackward I use Outputs - L and -R that activate two relais' to switch two fases from each motor when the
 *  joystick goes to x-.
 *  Pushbutton from Joy_stick canceld !
 *
 *  williamdob@hotmail.com
*/
 
int16_t Joy_y = analogRead(A0);   // range is 0 to 1023 (+y = L   /  -y = R)   
int16_t Joy_x = analogRead(A1);   // range is 0 to 1023 (+x = FW /   -x = BW )  
int16_t Safety_L = analogRead(A2);// feed back from the speed voltage to Controller_L < Speed_Limiet
int16_t Safety_R = analogRead(A3);// feed back from the speed voltage to Controller_R < Speed_Limiet

const int Button_Turn_L = A5;
const int Button_Turn_R = A4;
const int Dir_L_Motor = 4;// relais via transistor BD 137 
const int Dir_R_Motor = 7;// relais via transistor BD 137
const int L_Controller_ON = 12;// to white wire ( break) L controller, LOW = OFF
const int R_Controller_ON = 13;//  to white wire ( break) R controller, LOW = OFF
const int PWM_L_Motor = 10;// *DAC - 420Hz PWM_Output for the L_Motor on Pin 3
                           // with low pass filter 4K7/3,3µF - 0...5V
const int PWM_R_Motor = 9; // *DAC - 420Hz PWM_Output for the R Motor on Pin 9
                           // with low pass filter 4K7/3,3µF - 0...5V 
//*DAC - Digital Analog Converter : the PWM (0 to 1023)is converted to a voltage between 0V and 5V.  

int StateControllers;// (if == 0, Controllers Disabeld)
int Forward_Bit, Backward_Bit, Turn_L_Bit, Turn_R_Bit;
int ButtonState_Turn_L, ButtonState_Turn_R ;
int Speed_Fault = 0;
int Reset_Index = 0;
int16_t Speed_FW_Mapped, Speed_BW_Mapped;
int Extra_Speed_L, Extra_Speed_R;

const int Max_Joy_Speed_FW = 122;//(122 + Speed_Up = 132)  
const int Max_Joy_Speed_BW = 119;//(119 + Speed_Up = 129) 
const int Max_L = 101;
const int Max_R = 101;
const int Max_Turn_Speed = 88;//(88 + Speed_Up = 98)
const int Speed_Limiet = 410;// 410

// Slow_Start parameters
const int16_t Max_Count = 55;//55
int Speed_Up;
const int Max_Speed_Up = 10;//10

// Interrupt handler - if interrupts(); is on - every time Timer1 ( 65535 ticks ) has an overflow Count++ adds up with 1  
volatile int16_t Count;
ISR(TIMER1_OVF_vect){   
     Count++;
} 
    
/************************************************* FUNCTIONS *******************************************************/
void Direction_FW(){ 
                     
                     if(( Forward_Bit == 0)||(Backward_Bit == 1)){
                     delay(200); 
                     digitalWrite (L_Controller_ON,LOW);// Disable L Controller
                     digitalWrite (R_Controller_ON,LOW);// Disable R Controller           
                     delay(200);
                     digitalWrite (Dir_L_Motor, LOW);// Relais BW L OFF
                     digitalWrite (Dir_R_Motor, LOW);// Relais BW R OFF
                     delay(50);
                     digitalWrite(L_Controller_ON, HIGH);// Enable L Controller
                     digitalWrite(R_Controller_ON, HIGH);// Enable R Controller 
                     Forward_Bit = 1;
                     Backward_Bit = 0;
                     } 
}
         
void Direction_BW(){
                       
                     if (( Backward_Bit == 0 )||( Forward_Bit == 1)){
                     delay(200); 
                     digitalWrite (L_Controller_ON, LOW);// Disable L Controller
                     digitalWrite (R_Controller_ON, LOW);// Disable R Controller           
                     delay(200);
                     digitalWrite (Dir_L_Motor, HIGH );// Relais BW L ON
                     digitalWrite (Dir_R_Motor, HIGH); // Relais BW R ON
                     delay(50);
                     digitalWrite(L_Controller_ON, HIGH);// Enable L Controller 
                     digitalWrite (Dir_L_Motor, HIGH);// Relais BW L ON
                     digitalWrite (Dir_R_Motor, HIGH);// Relais BW R ON 
                     Backward_Bit = 1;
                     Forward_Bit = 0;
                     }
}
                  
void Direction_Turn_L(){
  
                     if (Turn_L_Bit == 0){ 
                     delay(50); 
                     digitalWrite (L_Controller_ON, LOW);// Disable L Controller
                     digitalWrite (R_Controller_ON, LOW);// Disable R Controller           
                     delay(50);
                     digitalWrite (Dir_L_Motor, LOW );// Relais BW L  OFF
                     digitalWrite (Dir_R_Motor, HIGH);// Relais BW R ON
                     delay(50);
                     digitalWrite(L_Controller_ON, HIGH);// Enable L Controller
                     digitalWrite(R_Controller_ON, HIGH);// Enable R Controller
                     Turn_L_Bit = 1;
                     Turn_R_Bit = 0;
                     }
}
                    
void Direction_Turn_R(){
  
                     if (Turn_R_Bit == 0){ 
                     delay(50); 
                     digitalWrite (L_Controller_ON, LOW);// Disable L Controller
                     digitalWrite (R_Controller_ON, LOW);// Disable R Controller           
                     delay(50);
                     digitalWrite (Dir_L_Motor, HIGH );//  Relais BW L  ON
                     digitalWrite (Dir_R_Motor, LOW);  //  Relais BW R OFF
                     delay(50);
                     digitalWrite(L_Controller_ON, HIGH);// Enable L Controller
                     digitalWrite(R_Controller_ON, HIGH);// Enable R Controller
                     Turn_R_Bit = 1;
                     Turn_L_Bit = 0;
                     }
}

void Define_Extra_Speed(){ // Y+ = turn R (= extra speed L wheel) , Y- = turn L (= extra speed R wheel)
                 
                  Joy_y = analogRead (A0); 
                  
                  if((Joy_y < 550)&&(Joy_y >= 450)){
                    Extra_Speed_L = 0;
                    Extra_Speed_R = 0;
                    }
                  else if((Joy_y >= 550)&&(Joy_y < 650)){
                    Extra_Speed_L = 1;
                    Extra_Speed_R = 0;
                    }
                  else if((Joy_y >= 650)&&(Joy_y < 750)){
                    Extra_Speed_L = 2;
                    Extra_Speed_R = 0;
                    }
                  else if((Joy_y >= 750)&&(Joy_y < 850)){
                    Extra_Speed_L = 3;
                    Extra_Speed_R = 0;
                    }
                  else if((Joy_y >= 850)&&(Joy_y < 1050)){
                    Extra_Speed_L = 5;
                    Extra_Speed_R = 0;
                    } 
                                      
                  else if((Joy_y < 450)&&(Joy_y >= 350)){
                    Extra_Speed_L = 0;
                    Extra_Speed_R = 1;
                    }
                 else if((Joy_y < 350)&&(Joy_y >= 250)){
                    Extra_Speed_L = 0;
                    Extra_Speed_R = 2;
                    }
                 else if((Joy_y < 250)&&(Joy_y >= 150)){
                    Extra_Speed_L = 0;
                    Extra_Speed_R = 3;
                    }
                 else if((Joy_y < 150)&&(Joy_y >= 0)){
                    Extra_Speed_L = 0;
                    Extra_Speed_R = 5;
                    }  
} 

void Enable_Controllers(){
                       
                    if ( StateControllers == 0){
                    interrupts(); // start InteruptHandler == start Count++; --> Increment_Speed();
                    digitalWrite (L_Controller_ON, HIGH);// Enable L Controller
                    digitalWrite (R_Controller_ON, HIGH);// Enable R Controller 
                    StateControllers = 1;
                    } 
}

void Disable_Controllers(){
                         
                    if ( StateControllers == 1){
                    noInterrupts();
                    Reset_Index = 0;   
                    digitalWrite (L_Controller_ON, LOW);// Disable L Controller
                    digitalWrite (R_Controller_ON, LOW);// Disable R Controller
                    StateControllers = 0;
                    }
}

void Speed_Limiet_Control(){
  
                    Safety_L = analogRead(A2);
                    Safety_R = analogRead (A3);
                    
                    if (( Safety_L > Speed_Limiet)||(Safety_R > Speed_Limiet)){
                    Speed_Fault = 1;
                    }
                      
                    if (Speed_Fault = 1){
                    digitalWrite (L_Controller_ON, LOW); // Disable L Controller
                    digitalWrite (R_Controller_ON, LOW); // Disable R Controller
                    }
} 

void Read_Analog_Values(){

                    Joy_y = analogRead (A0);
                    Joy_x = analogRead (A1);
                    Safety_L = analogRead(A2);
                    Safety_R = analogRead (A3);
                    ButtonState_Turn_L = digitalRead(Button_Turn_L);
                    ButtonState_Turn_R = digitalRead(Button_Turn_R);
}
 
void Reset_Values(){

                    if (Reset_Index == 0){
                    Count = 0;
                    Speed_Up = 0;
                    Reset_Index = 1;
                    }
}

void Increment_Speed(){
  
                    if (Count >= Max_Count){
                    Count = 0;
                    Speed_Up++;
                    }
                    if (Speed_Up > Max_Speed_Up)  Speed_Up = Max_Speed_Up;
                    
}
/********************************************************************************************************************************/

void setup() {
  
    TIMSK1 |= (1 << TOIE1); // enable Interrupts for Timer1 overflow detection ( 65535 Ticks)
             
    pinMode(Button_Turn_R, INPUT_PULLUP); // Active Low  
    pinMode(Button_Turn_L, INPUT_PULLUP); // Active Low
    pinMode(L_Controller_ON, OUTPUT); //  Active High
    pinMode(R_Controller_ON, OUTPUT); //  Active High
    pinMode(Dir_L_Motor, OUTPUT); //    (7) HIGH = BW, LOW = FW, relais BW L 
    pinMode(Dir_R_Motor, OUTPUT); //    (4) HIGH = BW, LOW = FW, relais BW R
    pinMode(PWM_L_Motor, OUTPUT); //    (9) PWM_Output for the L Motor 
    pinMode(PWM_R_Motor, OUTPUT);//     (10)PWM_Output for the R Motor 
    
    //Serial.begin(9600);
    }
    
void loop() {
    
    delay(200);// wait to init relais' L and R
    Read_Analog_Values();
    Disable_Controllers();
    Direction_FW();
       
   while((ButtonState_Turn_L == HIGH)&&(ButtonState_Turn_R== HIGH )){ // Push Buttons OFF
            
              Read_Analog_Values(); 
     
              // BOTH CONTROLLERS DISABLED  AND MOTORS 0 IF JOYSTICK IS IN THE MIDDLE POSITION
              // *****************************************************************************
                         
                            if ((Joy_x >= 450)&&(Joy_x < 550)){
                                                       
                            Read_Analog_Values();
                            Disable_Controllers();
                            Reset_Values ();  
                            Speed_Fault = 0;// Reset the Speed_Fault in Speed_Limiet_Control();
                            analogWrite (PWM_L_Motor, 0);
                            analogWrite (PWM_R_Motor, 0);
                            }
                                                 
              // BOTH MOTORS  FW, x+ 
              // *******************
                                 
                            else if ((Joy_x >= 550)){
                            //Serial.println(Speed_FW_Mapped + Speed_Up);
                                                        
                            Read_Analog_Values();    
                            Reset_Values ();         
                            Enable_Controllers();  
                            Direction_FW();
                            Increment_Speed();
                            Define_Extra_Speed();
                            Speed_FW_Mapped = map(Joy_x,0,1023, 0, Max_Joy_Speed_FW);
                            analogWrite ( PWM_L_Motor,((Speed_FW_Mapped + Speed_Up) + (Extra_Speed_L)-(Extra_Speed_R)));
                            analogWrite ( PWM_R_Motor,((Speed_FW_Mapped + Speed_Up) + (Extra_Speed_R)-(Extra_Speed_L)));
                            Speed_Limiet_Control();
                            }
             
                            
                // BOTH MOTORS BW, x-
               // *****************
                       
                             else if ((Joy_x < 450)){
                             //Serial.println(Joy_x);
                                                          
                             Read_Analog_Values(); 
                             Reset_Values(); 
                             Enable_Controllers(); 
                             Direction_BW();
                             Increment_Speed(); 
                             Define_Extra_Speed();
                             Speed_BW_Mapped = map(Joy_x,0,1023,Max_Joy_Speed_BW,0 ); 
                             analogWrite ( PWM_L_Motor,((Speed_BW_Mapped + Speed_Up) + (Extra_Speed_L)-(Extra_Speed_R)));
                             analogWrite ( PWM_R_Motor,((Speed_BW_Mapped + Speed_Up) + (Extra_Speed_R)-(Extra_Speed_L)));
                             Speed_Limiet_Control();
                             }
 }
   
   // TURN L WITH PUSHBUTTON (active LOW)
   // ***********************************
      
    while (ButtonState_Turn_L == LOW){
                            //Serial.println(Max_Turn_Speed); 
                            
                            Read_Analog_Values();
                            Direction_FW();                                      
                            Enable_Controllers();  
                            Direction_Turn_L();
                            Increment_Speed();
                            analogWrite(PWM_L_Motor,(Max_Turn_Speed + Speed_Up));
                            analogWrite(PWM_R_Motor,(Max_Turn_Speed + Speed_Up));
                            Speed_Limiet_Control();
    }                            
                            
   // TURN R WITH PUSHBUTTON (active LOW)
   // *********************************** 
    
   while (ButtonState_Turn_R == LOW){
                              
                            Read_Analog_Values();
                            Direction_FW();                                                         
                            Enable_Controllers();  
                            Direction_Turn_R();
                            Increment_Speed();
                            analogWrite(PWM_L_Motor,(Max_Turn_Speed + Speed_Up));
                            analogWrite(PWM_R_Motor,(Max_Turn_Speed + Speed_Up));
                            Speed_Limiet_Control();
   }
     delay(2);
 }   



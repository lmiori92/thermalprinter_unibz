/*
The main loop has a cycle time of 1ms (1000uS).
Everything shall be run within this CYCLE_TIME;
otherwise the system won't be predictable anymore.
*/

#include <stdarg.h>

#define CYCLE_TIME    100UL

#define MS_TO_CYCLES(x)  x * 1000UL / CYCLE_TIME

/**
 * Motor pins
 */
#define MOTOR_ENABLE_HS   A5  /**< Motor High Side */
#define MOTOR_ENABLE_LS   A4  /**< Motor Low Side */

/**
 * Sensors: - x2 for sensing head position
 *          - x1 for motor sensing (aka crude stepper motor)
 *  The analog output seems to be active low, pullup resistor needed
 */

#define PHOTO_HEAD_L    A0
#define PHOTO_HEAD_R    A1
#define PHOTO_MOTOR     A2

#define MOTOR_STEP_PER_REVOLUTION   10U
#define MOTOR_STEP_VERT_LINE        5U

#define PRINTHEAD_NUM    14

typedef struct
{
  bool printhead_power[PRINTHEAD_NUM];
  bool direction;
  uint32_t vertical_line_buffer;
} t_printhead;

typedef struct
{
  uint8_t state;
  bool photo_dx;
  uint32_t photo_dx_count;
  bool photo_sx;
  uint32_t photo_sx_count;
  bool photo_motor;
  uint32_t photo_motor_count;
  uint32_t photo_motor_diff; /* difference with previous cycle */
} t_motor;

typedef struct
{
  bool data;
  uint32_t data_count;
} t_serial;

typedef struct
{
  uint8_t state;
  t_motor motor;
  t_printhead printhead;
  t_serial serial;
} t_operational;

uint32_t dbg_counter = 0;

enum
{
  MOTOR_OFF,    /**< Motor is OFF, no voltage applied */
  MOTOR_BRAKE,    /**< Motor is being braked, both polarities are set to Vcc */
  MOTOR_ON,    /**< Motor runs normally */
  MOTOR_REVERSE  /**< Motor runs reversed (do NOT use this normally) */
};

  enum
  {
    STATE_IDLE,
    STATE_STEP,
    STATE_GOING_HOME,
    STATE_GOING_SX,
    STATE_SIDE_STEP
    
  };

t_operational g_operational;

uint32_t timestamp = 0;
uint32_t cycle_time = 0;


void p(char *fmt, ... )
{
        char buf[128]; // resulting string limited to 128 chars
        va_list args;
        va_start (args, fmt );
        vsnprintf(buf, 128, fmt, args);
        va_end (args);
        Serial.print(buf);
}

void motor_control(uint8_t state)
{
  
  switch(state)
  {
    case MOTOR_OFF:
      digitalWrite(MOTOR_ENABLE_HS, LOW);
      digitalWrite(MOTOR_ENABLE_LS, LOW);
      break;
    case MOTOR_BRAKE:
      digitalWrite(MOTOR_ENABLE_HS, HIGH);
      digitalWrite(MOTOR_ENABLE_LS, HIGH);
      break;
    case MOTOR_ON:
      digitalWrite(MOTOR_ENABLE_HS, HIGH);
      digitalWrite(MOTOR_ENABLE_LS, LOW);
      break;
    case MOTOR_REVERSE:
      digitalWrite(MOTOR_ENABLE_HS, LOW);
      digitalWrite(MOTOR_ENABLE_LS, HIGH);
      break;
    default:
      digitalWrite(MOTOR_ENABLE_HS, LOW);
      digitalWrite(MOTOR_ENABLE_LS, LOW);
      break;
  }
}

void hw_init()
{
    Serial.begin(115200);

    /** Digital Pins **/
    pinMode(MOTOR_ENABLE_HS, OUTPUT);
    pinMode(MOTOR_ENABLE_LS, OUTPUT);

    pinMode(PHOTO_HEAD_L, INPUT_PULLUP);
    pinMode(PHOTO_HEAD_R, INPUT_PULLUP);
    pinMode(PHOTO_MOTOR, INPUT_PULLUP);
}

void hw_input()
{
  bool tmp;
  tmp = digitalRead(PHOTO_HEAD_L) == HIGH ? true : false;
  if (g_operational.motor.photo_sx != tmp)
  {
    g_operational.motor.photo_sx_count++;
    g_operational.printhead.direction = false;
  }
  g_operational.motor.photo_sx = tmp;
  
  tmp = digitalRead(PHOTO_HEAD_R) == HIGH ? true : false;
  if (g_operational.motor.photo_dx != tmp)
  {
      g_operational.motor.photo_dx_count++;
      g_operational.printhead.direction = true;
  }
  g_operational.motor.photo_dx = tmp;
  
  tmp = digitalRead(PHOTO_MOTOR) == HIGH ? true : false;
  if (g_operational.motor.photo_motor != tmp)
  {
    g_operational.motor.photo_motor_count++;
    g_operational.motor.photo_motor_diff = 1;
  }
  else
  {
    g_operational.motor.photo_motor_diff = 0;
  }
  g_operational.motor.photo_motor = tmp;

}

void hw_output()
{
    motor_control(g_operational.motor.state);
}

void hw_dbg()
{
  dbg_counter++;

  if (dbg_counter > MS_TO_CYCLES(1000UL))
  {
//   p("Motor revolutions %ld\n", g_operational.motor.photo_motor_count);
//   p("Photo SX %ld\n", g_operational.motor.photo_sx_count);
//   p("Photo DX %ld\n", g_operational.motor.photo_dx_count);
   p("PhotoDX %d\n", g_operational.motor.photo_dx);
   p("State %d\n", g_operational.state);
   p("Cycle time %ld\n", cycle_time);
   p("Buffer %d\n", *((uint8_t*)(&g_operational.printhead.vertical_line_buffer)));
   p("Buffer %d\n", *((uint8_t*)(&g_operational.printhead.vertical_line_buffer) + 1));
   p("Buffer %d\n", *((uint8_t*)(&g_operational.printhead.vertical_line_buffer) + 2));
   p("%d\n", g_operational.serial.data_count);
   dbg_counter = 0;
  }

  /* Serial get: debug and commands */
         if (Serial.available() > 0) {
                uint8_t incomingByte = Serial.read();

                if (g_operational.serial.data == true)
                {
                    *(((uint8_t*)(&g_operational.printhead.vertical_line_buffer)) + (g_operational.serial.data_count)) = incomingByte;
                    g_operational.serial.data_count++;
                  if (g_operational.serial.data_count > 2)
                  {
                    g_operational.serial.data = false;
                  }
                }
                else
                {
                switch(incomingByte)
                {
                  case 'd':
                    g_operational.state = STATE_GOING_HOME;
                    break;
                  case 's':
                    g_operational.state = STATE_GOING_SX;
                    break;
                  case 'l':
                    g_operational.state = STATE_STEP;
                    break;
                  case 'x':
                    g_operational.serial.data_count = 0;
                    g_operational.serial.data = true;
                    break;
                }
                }
        }
  
}

bool rising_edge(bool *latch, bool input)
{
  bool tmp = false;
  if (*latch == false && input == true)
  {
    tmp = true;
    *latch = true;
  }
  else
  {
    *latch = input;
  }

  return tmp;
}
static int state = 0;
void app()
{
  static uint32_t test_counter = 0;
  /*
  static uint16_t count = 0;
  static uint16_t prev;
  static bool toggle = false;

  count += g_operational.motor.photo_motor_count - prev;
  if (count > 1 && !toggle)
  {
    g_operational.motor.state = MOTOR_ON;
    toggle = true;
  }
  else
  {
    toggle = false;
    g_operational.motor.state = MOTOR_BRAKE;
  }

  prev = g_operational.motor.photo_motor_count;
  */

  switch(g_operational.state)
  {
    case STATE_IDLE:
      /* ready receive command here */
      g_operational.motor.state = MOTOR_BRAKE;
      break;
    case STATE_STEP:
      /* move forward a single step */
      static uint16_t rev_count = 0;
      static uint16_t macheoh = 0;
      rev_count += g_operational.motor.photo_motor_diff;
      if (rev_count > 5U)
      {
        g_operational.motor.state = MOTOR_BRAKE;
        macheoh++;
        if (macheoh >= 100)
        {
          rev_count = 0;
          macheoh = 0;
          g_operational.state = STATE_IDLE;
        }
      }
      else
      {
        g_operational.motor.state = MOTOR_ON;
      }
      break;
    case STATE_GOING_HOME:
      /* initial calibration - the printhead will be found at the right side (DX) */
      if (!g_operational.motor.photo_dx)
      {
        g_operational.state = STATE_SIDE_STEP;
        g_operational.motor.state = MOTOR_BRAKE;
//        g_operational.printhead.direction = true; // i.e. read as .home = true! ;=)
      }
      else
      {
        g_operational.motor.state = MOTOR_ON;
      }

      break;
    case STATE_GOING_SX:
      if (!g_operational.motor.photo_sx)
      {
        g_operational.state = STATE_SIDE_STEP;
        g_operational.motor.state = MOTOR_BRAKE;
      }
      else
      {
        g_operational.motor.state = MOTOR_ON;
      }

      break;
    case STATE_SIDE_STEP:
      static uint32_t counter = 0;
      /* an extra movement is needed to place the head on the paper tray again */
      if (g_operational.printhead.direction == true) // home position, wait for photo DX
      {
        if (g_operational.motor.photo_dx)
        {
          /* go to idle and brake the motor meanwhile */
          counter++;
          if (counter > 100)
          {
            g_operational.state = STATE_IDLE;
            counter = 0;
          }
          g_operational.motor.state = MOTOR_BRAKE;
        }
        else
        {
          g_operational.motor.state = MOTOR_ON;
        }
      }
      else
      {
        if (g_operational.motor.photo_sx)
        {
          counter++;
          if (counter > 100)
          {
            /* go to idle and brake the motor meanwhile */
            g_operational.state = STATE_IDLE;
            counter = 0;
          }
          g_operational.motor.state = MOTOR_BRAKE;
        }
        else
        {
          g_operational.motor.state = MOTOR_ON;
        }
      }

      break;
    default:
      break;
  }
  
  test_counter++;
  if (test_counter >= MS_TO_CYCLES(2000))
 {
//   g_operational.state = STATE_STEP;
 } 
}

void setup()
{
  
  /* initialize data structures */
  memset(&g_operational, 0, sizeof(g_operational));
  
  g_operational.state = STATE_GOING_SX;
  
  hw_init();

}

void loop()
{

  timestamp = micros();
  
  /* Application starts here */
  hw_input();

  app();

  hw_output();

  hw_dbg();

  /* Application ends here */

  cycle_time = micros() - timestamp;
  /* wait for the next cycle, do nothing */  
  delayMicroseconds(CYCLE_TIME - cycle_time);

}

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

#define PRINTHEAD_NUM    7 //14
#define PRINTHEAD_STEPS  250 // TODO define me pliz

const uint8_t PRINTHEAD_PIN[PRINTHEAD_NUM] = { 7, 8, 2, 3, 4, 5, 6 };

typedef struct
{
  bool print;
  bool power[PRINTHEAD_NUM];
  bool direction;
  uint16_t step;
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
  bool rcv_data;  /**< receiving data*/
  uint8_t data_count; /**< current received byte */
  uint8_t vertical_line_buffer[3];  /**< buffer */
} t_serial;

typedef struct
{
  bool simulation;  /**< No actual HW output; fake motor spin count etc */
} t_settings;

typedef struct
{
  uint8_t state;
  uint32_t cycle_time;
  uint32_t timestamp;
  t_motor motor;
  t_printhead printhead;
  t_serial serial;
  t_settings settings;
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
  STATE_IDLE,  /**< Do nothing; receive commands*/
  STATE_STEP,  /**< Do a single vertical line step */
  STATE_GOING_DX,  /**< Move printhead to the DX side */
  STATE_GOING_SX,  /**< Move printhead to the SX side */
  STATE_SIDE_STEP,  /**< Additional step to prepare the head in start position */
  STATE_PRINT
};

/** Global operational variable */
t_operational g_operational;

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
    
    for (uint8_t i = 0; i < PRINTHEAD_NUM; i++)
    {
      pinMode(PRINTHEAD_PIN[i], OUTPUT);
    }
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
    uint8_t i = 0;
  
    /* Set motor output value */
    motor_control(g_operational.motor.state);

    /* Set printhead output value */
    for (i = 0; i < PRINTHEAD_NUM; i++)
    {
      if (g_operational.printhead.print == true)
      {
        digitalWrite(PRINTHEAD_PIN[i], g_operational.printhead.power[i]);
      }
      else
      {
        digitalWrite(PRINTHEAD_PIN[i], false);
      }
    }
}

void hw_dbg()
{
  
  /* buffer for the incoming vertical line data*/
  uint8_t i = 0;
  dbg_counter++;

  if (dbg_counter > MS_TO_CYCLES(1000UL))
  {
//   p("Motor revolutions %ld\n", g_operational.motor.photo_motor_count);
//   p("Photo SX %ld\n", g_operational.motor.photo_sx_count);
//   p("Photo DX %ld\n", g_operational.motor.photo_dx_count);
//   p("PhotoDX %d\n", g_operational.motor.photo_dx);
//   p("steps %d\n", g_operational.printhead.step);
//   p("State %d\n", g_operational.state);
//   p("Cycle time %ld\n", g_operational.cycle_time);
//   p("Buffer %d\n", *((uint8_t*)(&g_operational.printhead.power)));
//   p("Buffer %d\n", *((uint8_t*)(&g_operational.printhead.vertical_line_buffer) + 1));
//   p("Buffer %d\n", *((uint8_t*)(&g_operational.printhead.vertical_line_buffer) + 2));
 //  p("%d\n", g_operational.serial.data_count);
   dbg_counter = 0;
  }

  /* Serial get: debug and commands */
    if (Serial.available() > 0) {
      uint8_t incomingByte = Serial.read();

      if (g_operational.serial.rcv_data == true)
      {
//                    *(((uint8_t*)(&g_operational.printhead.vertical_line_buffer)) + (g_operational.serial.data_count)) = incomingByte;
       g_operational.serial.vertical_line_buffer[g_operational.serial.data_count] = incomingByte;
       g_operational.serial.data_count++;
                  
                  /* data received */
       if (g_operational.serial.data_count > 2)
                  {
                    /* stop receiving */
                    g_operational.serial.rcv_data = false;
                    /* transfer buffer to printhead power state array */
                    for (i = 0; i < PRINTHEAD_NUM; i++)
                    {
                      g_operational.printhead.power[i] = (g_operational.serial.vertical_line_buffer[(i / 7)] >> (i % 7)) & 1;
                    }
                  }
                }
                else
                {
                switch(incomingByte)
                {
                  case 'd':
                    g_operational.state = STATE_GOING_DX;
                    break;
                  case 's':
                    g_operational.state = STATE_GOING_SX;
                    break;
                  case 'l':
                    g_operational.state = STATE_STEP;
                    break;
                  case 'p':
                    g_operational.state = STATE_PRINT;
                    break;
                  case 'x':
                    g_operational.serial.data_count = 0;
                    g_operational.serial.rcv_data = true;
                    break;
                  case 'o':
//                    Serial.write(g_operational.state);
                    p("%d", g_operational.state);
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
        static uint32_t counter = 0;
      static uint16_t rev_count = 0;
      static uint16_t macheoh = 0;
  static uint32_t print_counter = 0;
  switch(g_operational.state)
  {
    case STATE_IDLE:
      /* doing nothing, ready to receive commands */
      g_operational.motor.state = MOTOR_OFF;
      g_operational.printhead.print = false;
      break;
    case STATE_PRINT:
      print_counter++;
      if (print_counter >= MS_TO_CYCLES(250))
      {
        g_operational.state = STATE_IDLE;
        g_operational.printhead.print = false;
      }
      else
      {
        g_operational.printhead.print = true;
      }
      break;
    case STATE_STEP:
      /* move forward a single step */
      rev_count += g_operational.motor.photo_motor_diff;
      if (rev_count > 5U)
      {
        g_operational.motor.state = MOTOR_BRAKE;
        macheoh++;

        /* step movement complete, go idle */
        if (macheoh >= 10)
        {
          rev_count = 0;
          macheoh = 0;
          g_operational.state = STATE_IDLE;
          g_operational.printhead.step++;
        }
      }
      else
      {
        g_operational.motor.state = MOTOR_ON;
      }

      /* check if we are out of steps OR the photo(dx or sx) is reached */
      if (g_operational.printhead.step > PRINTHEAD_STEPS || g_operational.motor.photo_dx == true || g_operational.motor.photo_sx == true)
      {
        if (g_operational.motor.photo_dx == true)
        {
         // g_operational.state = STATE_GOING_DX;
        }

        if (g_operational.motor.photo_sx == true)
        {
         // g_operational.state = STATE_GOING_SX;
        }

        /* reset steps */
        g_operational.printhead.step = 0;
      }
      else
      {
        /* continue */
      }

      break;
    case STATE_GOING_DX:
      /* initial calibration - the printhead will be found at the right side (DX) */
      if (!g_operational.motor.photo_dx)
      {
        g_operational.state = STATE_SIDE_STEP;
        g_operational.motor.state = MOTOR_BRAKE;
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
      /* an extra movement is needed to place the head on the paper tray again */
      if (g_operational.printhead.direction == true) /* DX, wait for photo-dx to go low */
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
        /* SX, wait for photo-sx go to low */
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

  g_operational.timestamp = micros();
  
  /* Application starts here */
  hw_input();

  app();

  hw_output();

  hw_dbg();

  /* Application ends here */

  g_operational.cycle_time = micros() - g_operational.timestamp;
  /* wait for the next cycle, do nothing */  
  delayMicroseconds(CYCLE_TIME - g_operational.cycle_time);

}

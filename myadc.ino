#include <EEPROM.h>

/* ---------------------------------------------------------------------------- */
/* ---- define constants  ----------------------------------------------------- */
/* set 1 to enable and 0 to disable debug messages on the serial bus */
#define DEBUG_PRINT         (1)

#if DEBUG_PRINT == 1
/* define decimal places for double type values for printing */
#define DPRECISION          3
/* define to convert a double type to a C based string */
#define DTOSTR(d_arg)     String((d_arg), DPRECISION).c_str()
/* define to print a formatted message on the serial bus */
#define dprint(fmt, ...)    do { \
  snprintf(ser_tx_buff, SER_TXBUFF_SZ, fmt, ##__VA_ARGS__); \
  Serial.println(ser_tx_buff); \
  Serial.flush(); \
} while (0)
#else
#define DTOSTR(...)
#define dprint(...)
#endif

/* magic value to init the state machine */
#define INIT_STATE          0x2f

/* serial transmit/receive buffer size */
#define SER_TXBUFF_SZ       80
#define SER_RXBUFF_SZ       16
#define SER_RXBUFF_STRLEN   15

/* total adc channels */
#define TOTAL_ADC           3
#define ADCX                0
#define ADCY                1
#define ADCZ                2

/* total bits of an adc channel */
#define ADC_BITS            10

/* step1: (oversample + accumulate) => moving average.
 * 4 pow NBITS samples are needed to get (ADC_BITS + NBITS) adc resolution.
 */
#define NBITS               4
#define OVER_SAMPLE         (word)256

/* step2: right shift a moving average adc value by NBITS => decimation */
#define DECIMATE_NBITS      16

/* step3: normal average decimated adc values for the white noise removal */
#define ADC_NORMAL_AVERAGE  (byte)8

/* ---------------------------------------------------------------------------- */
/* ---- user defined data types ----------------------------------------------- */
/* enum type defines program states */
typedef enum {
  PR_START = INIT_STATE,
  PR_POLL_SERIAL_RX,
  PR_CALIBRATE,
  PR_ADC_READ,
  PR_ADC_FINISH,
  PR_ADC_CANBUS_SEND,
} state_t;

/* enum type defines identifiers of serial read string commands */
typedef enum {
  SER_CMD_ADC_CALIBRATE = 0,
  SER_CMD_LAST,
} ser_cmd_id_t;

/* function pointer type defines callback for a serial read string command */
typedef void (*ser_cmd_cb_t)(void);

/* struct type defines a serial read string command and a callback to call
 * if a string command is received. */
typedef struct {
  char ser_cmd_str[SER_RXBUFF_SZ];
  ser_cmd_cb_t ser_cmd_cb;
} ser_cmd_t;

/* struct type to store adc calibration parameters in eeprom */
typedef union {
  struct {
    double offset_err[TOTAL_ADC];
    double gain_err[TOTAL_ADC];
  };
  byte eeprom_buff[sizeof(double)*TOTAL_ADC*2];
} eeprom_adc_t;

/* struct type for an adc channel */
typedef struct {
  /* accumulates adc samples (oversampling) */
  double accumulate;
  byte chan;
} adc_t;

/* ---------------------------------------------------------------------------- */
/* ---- function prototypes --------------------------------------------------- */
static void ser_cmd_adc_calibrate(void);

/* ---------------------------------------------------------------------------- */
/* ---- shared variables ------------------------------------------------------ */
/* stores current program state */
static state_t cur_state;

/* allocate serial transmit/receive shared variables */
static char ser_tx_buff[SER_TXBUFF_SZ];
static char ser_rx_buff[SER_RXBUFF_SZ];
/* add serial read string commands to receive from a host system */
static ser_cmd_t ser_rx_cmds[SER_CMD_LAST] = {
  {"calibrate", ser_cmd_adc_calibrate},
};

/* allocate adc calibration parameters */
static eeprom_adc_t eeprom_adc;

/* allocate adc channels */
static adc_t adc_channels[TOTAL_ADC];

/* stores current adc channel */
static byte cur_adc;

/* counts accumulated adc samples */
static word adc_samples;

/* ---------------------------------------------------------------------------- */
/* ---- interrupt routines ---------------------------------------------------- */
ISR(ADC_vect)
{
  /* read and add the current adc sample to previous accumulated adc samples of
   * an adc channel. */
  adc_channels[cur_adc].accumulate += (double)(ADC);
  
  /* set a next adc channel to read */
  cur_adc++;
  
  if (cur_adc == TOTAL_ADC) {
    /* restart from ADCX channel */
    cur_adc = 0;
    
    /* count total adc samples */
    adc_samples++;
  }
  
  /* select a next adc channel for reading */
  ADMUX = (ADMUX & 0xf0) | (adc_channels[cur_adc].chan & 0x07);
  
  if (adc_samples == OVER_SAMPLE) {
    /* disable adc interrupt */
    ADCSRA &= ~(1 << ADIE);
    
    /* set next state */
    cur_state = PR_ADC_FINISH;
  }
}

/* ---------------------------------------------------------------------------- */
/* ---- user code ------------------------------------------------------------- */
static void eeprom_read(void) {
  for (byte addr = 0; addr < sizeof(eeprom_adc_t); addr++) {
    EEPROM.get(addr, eeprom_adc.eeprom_buff[addr]);
  }
}

static void eeprom_write(void) {
  for (byte addr = 0; addr < sizeof(eeprom_adc_t); addr++) {
    EEPROM.update(addr, eeprom_adc.eeprom_buff[addr]);
  }
}

static void ser_cmd_adc_calibrate(void) {
  /* disable adc interrupt */
  ADCSRA &= ~(1 << ADIE);
  
  /* set next state */
  cur_state = PR_CALIBRATE;
}

static void serial_read(ser_cmd_id_t ser_cmd_id) {
  char rx;
  static byte rx_match_idx = 0;
  char *ser_cmd_str = NULL;
  
  /* handle invalid input arguments */
  if (ser_cmd_id < 0 || ser_cmd_id >= SER_CMD_LAST) {
    /* discard received bytes before a next read on the serial bus */
    while ((UCSR0A & (1 << RXC0)) != 0) {
      UDR0;
    }
  } else if ((UCSR0A & ((1 << FE0) | (1 << DOR0) | (1 << UPE0))) != 0) {
    /* discard received byte if there is a serial bus error */
    UDR0;
  } else {
    /* receive data if available */
    if ((UCSR0A & (1 << RXC0)) != 0) {
      /* no error occurred, read a byte */
      rx = UDR0;
      
      /* set ser_cmd_str */
      ser_cmd_str = ser_rx_cmds[ser_cmd_id].ser_cmd_str;
      
      /* handle received byte mismatch */
      ser_rx_buff[rx_match_idx] = rx;
      if (rx != ser_cmd_str[rx_match_idx]) {
        ser_rx_buff[rx_match_idx + 1] = '\0';
        dprint("matched: %d cmdlen: %d received: %s",
          rx_match_idx, strlen(ser_cmd_str), ser_rx_buff);
        /* reset read on the serial bus */
        rx_match_idx = 0;
      } else {
        /* update rx_match_idx before a next read on the serial bus */
        rx_match_idx++;
      }
      
      /* handle received command match */
      if (rx_match_idx == strlen(ser_cmd_str)) {
        ser_rx_buff[rx_match_idx] = '\0';
        /* call callback of a received serial command */
        ser_rx_cmds[ser_cmd_id].ser_cmd_cb();
        /* reset read on the serial bus */
        rx_match_idx = 0;
      }
      
      /* discard remaining received bytes before a next read on the serial bus */
      while (rx_match_idx == 0 && (UCSR0A & (1 << RXC0)) != 0) {
        UDR0;
      }
    }
  }
}

void setup() {
  /* setup serial */
  Serial.begin(115200);
  while (!Serial);
  
  delay(1000);
  
  /* disable serial read interrupt */
  UCSR0B &= ~(1 << RXCIE0);
  
  /* setup adc */
  /* use voltage at AREF pin as reference for the maximum adc value */
  ADMUX = (0 << REFS1) | (0 << REFS0) | (0 << ADLAR) | (0 << MUX3);
  
  /* start adc at clock rate of 125 KHz (system clock / 128 adc clock divider) */
  ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  
  /* set auto trigger to free running mode */
  ADCSRB = 0;
  
  /* init adc channels */
  adc_channels[ADCX].chan = ADCX;
  adc_channels[ADCY].chan = ADCY;
  adc_channels[ADCZ].chan = ADCZ;
  
  /* get adc calibration parameters from eeprom */
  eeprom_read();
  
  /* print adc calibration parameters */
  dprint("\r\narduino setup complete...");
  for (cur_adc = ADCX; cur_adc <= ADCZ; cur_adc++) {
    dprint("ADC%c calibration offset_err: %s gain_err: %s",
      (char)('X' + cur_adc),
      DTOSTR(eeprom_adc.offset_err[cur_adc]),
      DTOSTR(eeprom_adc.gain_err[cur_adc]));
  }
}

void loop() {
  double adc_data[TOTAL_ADC];
  double adc_nbit[TOTAL_ADC];
  byte adc_average = 0;
  
  /* init state machine */
  cur_state = PR_START;
  
  /* state machine */
  dprint("starting state machine...");
  while (1) {
    switch(cur_state) {
      case PR_START:
        /* reset adc data */
        adc_average = 0;
        for (cur_adc = ADCX; cur_adc <= ADCZ; cur_adc++) {
          adc_data[cur_adc] = 0;
          adc_nbit[cur_adc] = 0;
        }
        
        /* set next state */
        cur_state = PR_ADC_READ;
        break;
      case PR_POLL_SERIAL_RX:
        serial_read(SER_CMD_ADC_CALIBRATE);
        break;
      case PR_CALIBRATE:
        /* update adc calibration parameters in eeprom */
        for (cur_adc = ADCX; cur_adc <= ADCZ; cur_adc++) {
          eeprom_adc.offset_err[cur_adc] = (double)0;
          eeprom_adc.gain_err[cur_adc] = (double)1;
        }
        eeprom_write();
        
        /* print adc calibration parameters */
        for (cur_adc = ADCX; cur_adc <= ADCZ; cur_adc++) {
          dprint("ADC%c calibration offset_err: %s gain_err: %s",
            (char)('X' + cur_adc),
            DTOSTR(eeprom_adc.offset_err[cur_adc]),
            DTOSTR(eeprom_adc.gain_err[cur_adc]));
        }
        
        delay(10000);
        
        /* set next state */
        cur_state = PR_START;
        break;
      case PR_ADC_READ:
        /* set next state */
        cur_state = PR_POLL_SERIAL_RX;
        
        /* init adc channel */
        adc_samples = 0;
        for (cur_adc = ADCX; cur_adc <= ADCZ; cur_adc++) {
          adc_channels[cur_adc].accumulate = 0;
        }
        
        /* select ADCX channel */
        cur_adc = 0;
        ADMUX = (ADMUX & 0xf0) | (adc_channels[ADCX].chan & 0x07);
        
        /* enable adc interrupt */
        ADCSRA |= (1 << ADIE);
        break;
      case PR_ADC_FINISH:
        /*dprint("ADC_RAW ADCX: %s ADCY: %s ADCZ: %s",
          DTOSTR(adc_channels[ADCX].accumulate),
          DTOSTR(adc_channels[ADCY].accumulate),
          DTOSTR(adc_channels[ADCZ].accumulate));*/
        
        /* set next state */
        cur_state = PR_ADC_READ;
        
        /* synthesize (ADC_BITS + NBITS) adc values */
        adc_average++;
        for (cur_adc = ADCX; cur_adc <= ADCZ; cur_adc++) {
          /* offset error correction */
          adc_channels[cur_adc].accumulate += eeprom_adc.offset_err[cur_adc];
          
          /* gain error correction */
          adc_channels[cur_adc].accumulate *= eeprom_adc.gain_err[cur_adc];
          
          /* decimate a moving average adc value to get (ADC_BITS + NBITS) adc
           * resolution. */
          adc_data[cur_adc] += (adc_channels[cur_adc].accumulate / DECIMATE_NBITS);
          
          /* normal average decimated adc values for the white noise removal */
          if (adc_average == ADC_NORMAL_AVERAGE) {
            adc_data[cur_adc] /= ADC_NORMAL_AVERAGE;
            adc_nbit[cur_adc] = adc_data[cur_adc];
            
            /* update next state */
            cur_state = PR_ADC_CANBUS_SEND;
          }
        }
        break;
      case PR_ADC_CANBUS_SEND:
        dprint("ADC_%uBITS ADCX: %s ADCY: %s ADCZ: %s",
          (ADC_BITS + NBITS),
          DTOSTR(adc_nbit[ADCX]),
          DTOSTR(adc_nbit[ADCY]),
          DTOSTR(adc_nbit[ADCZ]));
        
        /* set next state */
        cur_state = PR_START;
        break;
      default:
        dprint("unknown program state, restarting state machine...");
        return;
    }
  }
}

#include <stdio.h>
#include <string.h>

#include <can.h>
#include <mcp2515.h>

#include <EEPROM.h>
#include <SPI.h>

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

/* magic value to init the program state machine */
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

/* define can bus SPI chip select pin */
#define CAN_CS_PIN          10
/* define can bus interrupt pin */
#define CAN_INTR_PIN        2
/* use a free adc channel as a seed for the random number generator */
#define CAN_RAND_ADC        4
/* define maximum number of ECUs that are supported on a can bus network */
#define CAN_MAX_NODES       24
#define CAN_PERNODE_MSGIDS	TOTAL_ADC
#define CAN_ALLNODES_MSGIDS	(CAN_MAX_NODES * CAN_PERNODE_MSGIDS)
#define CAN_MASTER_MSGIDS	5
#define CAN_TOTAL_MSGIDS	(CAN_MASTER_MSGIDS + CAN_ALLNODES_MSGIDS)
/* standard can bus network supports 11 bits message identifiers */
#define CAN_VALID_MSGIDS	0x7ff
/* define min and max can bus message identifiers */
#define CAN_MIN_MSGID       1
#define CAN_MAX_MSGID       (CAN_VALID_MSGIDS - CAN_TOTAL_MSGIDS)
/* define to find a free message identifier from CAN_ALLNODES_MSGIDS */
#define CAN_NODE_FREE		0x7ff

/* ---------------------------------------------------------------------------- */
/* ---- user defined data types ----------------------------------------------- */
/* enum type defines program states */
typedef enum {
  PR_START = INIT_STATE,
  PR_POLL_RECEIVE,
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

/* enum type defines can bus states */
typedef enum {
  /* following states till CAN_CMD_LAST send defined data on can bus.
   * CAN_CMD_ADC_SEND states sends an adc channel's data on can bus.
   * CAN_CMD_TRANSIT state represents a can message was transmitted.  
   */
  CAN_CMD_MASTER_REG = 0,
  CAN_CMD_SLAVE_REG,
  CAN_CMD_MASTER_SET,
  CAN_CMD_LAST,
  CAN_CMD_SLAVE_SET,
  CAN_CMD_ADC_SEND,
  CAN_CMD_TRANSIT,
  CAN_ROLE_START,
  CAN_ROLE_MASTER,
  CAN_ROLE_SLAVE,
  CAN_ROLE_LAST,
} can_cmd_id_t;

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

/* allocate can bus library object */
MCP2515 mcp2515(CAN_CS_PIN);
/* add can bus commands.
 * a command of exactly 8 bytes is supported on the can bus network.
 * add char # in place of an unused byte.
 */
static byte can_cmds[CAN_CMD_LAST][CAN_MAX_DLEN] = {
  /*
  {'0', '1', '2', '3', '4', '5', '6', '7'},
  */
  {'M', 'S', 'T', '#', 'R', 'E', 'G', '#'}, /* master register command */
  {'S', 'L', 'V', '#', 'R', 'E', 'G', '#'}, /* slave register command */
  {'M', 'S', 'T', '#', 'S', 'E', 'T', '#'}, /* master set command */
};
/* detects can bus interrupt */
static byte canbus_intr = 0;
/* stores can bus state */
static can_cmd_id_t can_state;
/* store a can bus node's role as a master or slave */
static can_cmd_id_t can_role;
/* allocate can bus frames */
static struct can_frame can_tx;
static struct can_frame can_ans;
/* stores a start message identifier that a master node uses to register can nodes */
static canid_t can_start_msgid;
/* stores message identifiers of registered slaves on a master node */
static canid_t can_init_msgids[CAN_MAX_NODES];
/* stores message identifiers of adc data sending can nodes */
static canid_t can_adc_msgids[CAN_ALLNODES_MSGIDS];
/* stores adc data from each can node */
static double can_adc_data[CAN_ALLNODES_MSGIDS];

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

void canbus_intr_cb(void) {
  canbus_intr++;
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
  
  /* handle invalid input argument */
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

static void canbus_handler(can_cmd_id_t can_cmd_id) {
  struct can_frame *can_p = NULL;
  byte irq;
  MCP2515::RXBn rx_sel;
  boolean rx_intr = false;
  static byte can_mst_reg_cnt = 0;
  
  /* handle valid input argument */
  if ((can_cmd_id >= CAN_CMD_MASTER_REG && can_cmd_id < CAN_ROLE_LAST))
  {
    /* select a can frame to transmit */
    can_p = &can_ans;
    if (can_cmd_id <= CAN_CMD_SLAVE_REG) {
      can_p = &can_tx;
    }
    
    /* handle can bus tx */
    if (can_cmd_id < CAN_CMD_TRANSIT) {
      /* select data of a can frame */
      if (can_cmd_id <= CAN_CMD_MASTER_SET) {
        memcpy(can_p->data, can_cmds[can_cmd_id], CAN_MAX_DLEN);
      }
      
      /* send a can frame */
      while (mcp2515.sendMessage(can_p) != MCP2515::ERROR_OK) {
        delay(100);
      }
      
      /* update can master register count after sending of a can frame */
      if (can_state == CAN_CMD_MASTER_REG) {
        can_mst_reg_cnt++;
      }
      
      /* mark sending of a can frame */
      can_state = CAN_CMD_TRANSIT;
    }
    
    /* exit if there is no interrupt to handle */
    if (canbus_intr == 0) {
      return;
    }
    
    /* check can bus interrupt */
    irq = mcp2515.getInterrupts();
    if (irq & MCP2515::CANINTF_RX0IF) {
      rx_sel = MCP2515::RXB0;
      rx_intr = true;
    }
    if (irq & MCP2515::CANINTF_RX1IF) {
      rx_sel = MCP2515::RXB1;
      rx_intr = true;
    }
    
    /* handle can bus rx */
    if (rx_intr == true && (mcp2515.readMessage(rx_sel, &can_ans) == MCP2515::ERROR_OK)) {
      if (memcmp(can_ans.data, can_cmds[CAN_CMD_MASTER_REG], CAN_MAX_DLEN) == 0) {
        /* handle a can master register */
        if (can_role == CAN_ROLE_START) {
          /* update can master register count after receiving of a can frame */
          can_mst_reg_cnt++;
          if (can_mst_reg_cnt >= 2) {
            /* a master role is assumed if a can node detects two master register can frames */
            can_role = CAN_ROLE_MASTER;
          }
        }
        if (can_role == CAN_ROLE_MASTER) {
          /* send a can message that a can master exists with the highest priority message identifier 0 */
          can_ans.can_id = 0;
          can_state = CAN_CMD_MASTER_SET;
        }
      } else if (memcmp(can_ans.data, can_cmds[CAN_CMD_MASTER_SET], CAN_MAX_DLEN) == 0) {
        /* handle a can master set */
        if (can_role == CAN_ROLE_START) {
          /* reset can master register count */
          can_mst_reg_cnt = 0;
          /* send a can message to register as a slave */
          can_state = CAN_CMD_SLAVE_REG;
        }
      } else if (memcmp(can_ans.data, can_cmds[CAN_CMD_SLAVE_REG], CAN_MAX_DLEN) == 0) {
        /* handle a can slave register */
        if (can_role == CAN_ROLE_MASTER) {
          /* send a can message to set a master assigned message identifier as a slave
           * with the highest priority message identifier 0.
           */
          can_ans.can_dlc = 4;
          can_ans.can_id = 0;
          for (int idx = 0; idx < CAN_MAX_NODES; idx++) {
		  }
            if (pool_can_id[idx][0] == 0) {
              pool_can_id[idx][0] = can_ans.can_id;
              pool_can_id[idx][1] = can_start_msgid;
              pool_can_id[idx][1] = can_start_msgid + ADCY;
              pool_can_id[idx][1] = can_start_msgid + ADC;
              /* update master side message identifier for assigning to a next can node.
               * CAN_PERNODE_MSGIDS is added for the message identifier of each adc channel's data.
               */
              can_start_msgid += CAN_PERNODE_MSGIDS;
            }
            if (can_ans.can_id == pool_can_id[idx][0]) {
              /* select data of a can frame */
              memcpy(can_ans.data, &pool_can_id[idx][0], can_ans.can_dlc);
              break;
            }
          }
          /* protocol handles at maximum CAN_TOTAL_MSGIDS message identifiers */
          if (can_free_id >= CAN_TOTAL_MSGIDS) {
            /* send can error message */
          }
          can_state = CAN_CMD_SLAVE_SET;
        }
      } else if (can_ans.can_dlc == 4 && can_ans.can_id == 0) {
        /* handle a can slave set */
        int data[2];
        memcpy(data, can_ans.data, can_ans.can_dlc);
        if (data[0] == can_tx.can_id) {
          can_tx.can_id = data[1];
          can_role = CAN_ROLE_SLAVE;
        }
      } else {
        /* handle can adc data receive */
          for (int idx = 0; idx < CAN_MAX_NODES; idx++) {
          if (can_ans.can_id == can_pool_can_id[idx][1] ||
              == can_ans.can_id) {
            memcpy(can_adc_data[idx], 
            pool_can_id[idx][0] = can_ans.can_id;
            pool_can_id[idx][1] = can_free_id;
            /* update master side message identifier for assigning to a next slave.
             * 3 is added for the message identifier of each adc channel's data.
             */
            can_free_id += 3;
          }
          if (can_ans.can_id == pool_can_id[idx][0]) {
            /* select data of a can frame */
            memcpy(can_ans.data, &pool_can_id[idx][0], can_ans.can_dlc);
            break;
          }
        }
      }
    }
  }
}

static void canbus_init(void) {
  /* init random number generator */
  randomSeed(analogRead(CAN_RAND_ADC));
  
  /* init can bus */
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();
  can_tx.can_id = random(CAN_MIN_MSGID, CAN_MAX_MSGID);
  can_tx.can_dlc = CAN_MAX_DLEN;
  can_start_msgid = can_tx.can_id + CAN_MASTER_MSGIDS;
  
  /* init can shared variables */
  can_state = CAN_CMD_MASTER_REG;
  can_role = CAN_ROLE_START;
  canbus_intr = 0;
  memset(can_init_msgids, 0, sizeof(canid_t));
  for (int idx = 0; idx < CAN_MAX_NODES; idx++) {
	can_adc_msgids[idx] = CAN_NODE_FREE;
  }
  memset(can_adc_data, 0, sizeof(can_adc_data));
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

  /* setup canbus */
  /* enable can bus interrupt */
  pinMode(CAN_INTR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CAN_INTR_PIN), canbus_intr_cb, FALLING);
  
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

  /* init can bus */
  canbus_init();
  
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
        
        /* set a can bus node as a master or slave.
         * a can bus node that succeeds first in sending the can master register command assumes the mater role.
         * other can bus nodes request their slave message identifier from a master node.  
         */
        do {
          canbus_handler(can_state);
        } while (can_role == CAN_ROLE_START);
        
        /* set next state */
        cur_state = PR_ADC_READ;
        break;
      case PR_POLL_RECEIVE:
        while (canbus_intr > 0) {
          canbus_handler(can_state);
          canbus_intr = canbus_intr - 1;
        }
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
        cur_state = PR_POLL_RECEIVE;
        
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
        
        for (cur_adc = ADCX; cur_adc <= ADCZ; cur_adc++) {
          /* wait until can transmit channel is free */
          while (can_state != CAN_CMD_TRANSIT) {
            canbus_handler(can_state);
          }
          /* send an adc channel data on a can bus */
          can_ans.can_id = can_tx.can_id + cur_adc;
          memcpy(can_ans.data, adc_nbit[cur_adc], sizeof(double));
          canbus_handler(CAN_CMD_ADC_SEND);
        }
        
        /* set next state */
        cur_state = PR_START;
        break;
      default:
        dprint("unknown program state, restarting state machine...");
        return;
    }
  }
}

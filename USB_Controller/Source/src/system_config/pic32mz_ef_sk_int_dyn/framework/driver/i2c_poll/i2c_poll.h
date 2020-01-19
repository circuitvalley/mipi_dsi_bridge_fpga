/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _I2C_POLL_    /* Guard against multiple inclusion */
#define _I2C_POLL_
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct i2c_poll_system_s i2c_poll_system_t;

typedef enum
{
    // Indicates that a non-system defined error has occurred.  The caller
    // must call the extended status routine for the module in question to
    // identify the error.
    I2C_POLL_STATUS_ERROR_EXTENDED   = -10,

    /*An unspecified error has occurred.*/
    I2C_POLL_STATUS_ERROR            = -1,

    // The module has not yet been initialized
    I2C_POLL_STATUS_UNINITIALIZED    = 0,

    // An operation is currently in progress
    I2C_POLL_STATUS_BUSY             = 1,

    // Any previous operations have succeeded and the module is ready for
    // additional operations
    I2C_POLL_STATUS_READY            = 2,

    // Indicates that the module is in a non-system defined ready/run state.
    // The caller must call the extended status routine for the module in
    // question to identify the state.
    I2C_POLL_STATUS_READY_EXTENDED   = 10

} I2C_POLL_STATUS;

typedef enum {

    I2C_POLL_ID_1 = _I2C1_BASE_ADDRESS,
    I2C_POLL_ID_3 = _I2C3_BASE_ADDRESS,
    I2C_POLL_ID_4 = _I2C4_BASE_ADDRESS,
    I2C_POLL_ID_5 = _I2C5_BASE_ADDRESS,
    I2C_POLL_NUMBER_OF_MODULES = 4

} I2C_POLL_MODULE_ID;


typedef enum {

    I2C_PORT_CHANNEL_B = 0x01,
    I2C_PORT_CHANNEL_C = 0x02,
    I2C_PORT_CHANNEL_D = 0x03,
    I2C_PORT_CHANNEL_E = 0x04,
    I2C_PORT_CHANNEL_F = 0x05,
    I2C_PORT_CHANNEL_G = 0x06

} I2C_POLL_PORTS_CHANNEL;


typedef enum {

    I2C_PORTS_BIT_POS_0 = 0,
    I2C_PORTS_BIT_POS_1 = 1,
    I2C_PORTS_BIT_POS_2 = 2,
    I2C_PORTS_BIT_POS_3 = 3,
    I2C_PORTS_BIT_POS_4 = 4,
    I2C_PORTS_BIT_POS_5 = 5,
    I2C_PORTS_BIT_POS_6 = 6,
    I2C_PORTS_BIT_POS_7 = 7,
    I2C_PORTS_BIT_POS_8 = 8,
    I2C_PORTS_BIT_POS_9 = 9,
    I2C_PORTS_BIT_POS_10 = 10,
    I2C_PORTS_BIT_POS_11 = 11,
    I2C_PORTS_BIT_POS_12 = 12,
    I2C_PORTS_BIT_POS_13 = 13,
    I2C_PORTS_BIT_POS_14 = 14,
    I2C_PORTS_BIT_POS_15 = 15

} I2C_PORTS_BIT_POS;

typedef enum
{
    /* I2C Mode Master */
    I2C_POLL_MODE_MASTER     /*DOM-IGNORE-BEGIN*/  = 0 /*DOM-IGNORE-END*/,

    /* I2C Mode Slave */
    I2C_POLL_MODE_SLAVE      /*DOM-IGNORE-BEGIN*/  = 1 /*DOM-IGNORE-END*/

} I2C_POLL_MODE;





typedef unsigned short int I2C_MODULE_INDEX;
typedef uintptr_t I2C_POLL_HANDLE;
#define I2C_POLL_SYSTEM_INVALID      ((I2C_POLL_HANDLE) -1 )


typedef enum
{

   /* I2C Slave 7 bit  */
    I2C_POLL_7BIT_SLAVE      /*DOM-IGNORE-BEGIN*/  = 1 /*DOM-IGNORE-END*/,

    /* I2C Slave 10 bit  */
    I2C_POLL_10BIT_SLAVE     /*DOM-IGNORE-BEGIN*/  = 2 /*DOM-IGNORE-END*/

} I2C_POLL_ADDRESS_WIDTH;

typedef enum
{
    I2C_POLL_MODULE_IDLE = 0x00,

    /* Master reads data from slave */
    I2C_POLL_MASTER_TX_TO_SLAVE = 0x01,

    /* Master reads data from slave */
    I2C_POLL_MASTER_RX_FROM_SLAVE = 0x02,

    /* I2C Slave ready to receive data from Master */
    I2C_POLL_SLAVE_READY_TO_RX_FROM_MASTER = 0x03,

    /* I2C Slave ready to receive data from Master */
    I2C_POLL_SLAVE_READY_TO_TX_TO_MASTER = 0x04,

   /* I2C Stop condition */
   I2C_POLL_MASTER_STOP = 0x05

} I2C_POLL_MODULE_MAIN_STATE;




typedef enum
{
    /* normal I2C with not ignoring any error */
    I2C_POLL_HALT_ON_ERROR = 0x00,
            
    /* ignore bus collision error */
    I2C_POLL_BUS_IGNORE_COLLISION_ERROR = 0x01,
            
    /* ignore overflow error */
    I2C_POLL_BUS_IGNORE_OVERFLOW_ERROR = 0x02,
    
}I2C_POLL_BUS_ERROR_EVENT;

// *****************************************************************************
/* I2C Buffer Events

  Summary:
    Lists the different conditions that happens during a buffer transfer.

  Description:
    This enumeration identifies the different conditions that can happen during
    a buffer transaction. Callbacks can be made with the appropriate buffer
    condition passed as a parameter to execute the desired action.
    The application can also poll the BufferStatus flag to check the status of
    transfer.

    The values act like flags and multiple flags can be set.

  Remarks:
    None.
*/

typedef enum
{
    /* Buffer is pending to get processed */
    I2C_POLL_BUFFER_EVENT_PENDING,

    /* All data from or to the buffer was transferred successfully. */
    I2C_POLL_BUFFER_EVENT_COMPLETE,

    /* There was an error while processing the buffer transfer request. */
    I2C_POLL_BUFFER_EVENT_ERROR,

    /*  Send Stop by Master */
    I2C_POLL_SEND_STOP_EVENT,

    /* Send Restart Event by Master */
    I2C_POLL_SEND_RESTART_EVENT,

    /* Master sends data to slave */
    I2C_POLL_BUFFER_SLAVE_READ_REQUESTED,

    /* Master requests data from slave */
    I2C_POLL_BUFFER_SLAVE_WRITE_REQUESTED,

    /* Slave read byte send by Master */
    I2C_POLL_BUFFER_SLAVE_READ_BYTE,
            
    /* sending ACK to Slave for more bytes */
    I2C_POLL_BUFFER_MASTER_ACK_SEND,
            
    /* sending ACK to Slave for more bytes */
    I2C_POLL_BUFFER_MASTER_NACK_SEND,

   /* Slave send byte to Master */
    I2C_POLL_BUFFER_SLAVE_WRITE_BYTE

} I2C_POLL_BUFFER_EVENT;


// *****************************************************************************
/* I2C Driver task states

  Summary
    Lists the different states that I2C task routine can have.

  Description
    This enumeration lists the different states that I2C task routine can have.

  Remarks:
    None.
*/

typedef enum
{
    /* Process queue */
    I2C_POLL_TASK_SEND_DEVICE_ADDRESS,
            
    /* state where 10-bit address scheme is used */
    I2C_POLL_SEND_DEVICE_ADDRESS_BYTE_2,

   /* I2C task handle write only buffer request */
    I2C_POLL_TASK_PROCESS_WRITE_ONLY,

   /* I2C task handle read only buffer request */
    I2C_POLL_TASK_PROCESS_READ_ONLY,

   /* I2C task handle for Read Write function */
    I2C_POLL_TASK_PROCESS_WRITE_READ,

   /* Set I2C Task to set RCEN flag */
    I2C_POLL_TASK_SET_RCEN_ONLY,

   /* Set I2C Task to issue RESTART */
    I2C_POLL_TASK_PROCESS_RESTART,

   /* I2C Bus Transaction Complete */
    I2C_POLL_BUS_SILENT,
            
   /* Set I2C Task to issue STOP */
    I2C_POLL_TASK_PROCESS_STOP,
            
    /*I2C sending Read request after Repeated Start*/
    I2C_POLL_RESEND_READ_DEVICE_ADDRESS,
            

} I2C_POLL_DATA_OBJECT_TASK;

typedef enum
{

    /* I2C operation write only buffer request  */
    I2C_POLL_OP_WRITE = 0x00,

    /* I2C operation read only buffer request   */
    I2C_POLL_OP_READ = 0x01,

    /* I2C operation write followed by read     */
    I2C_POLL_OP_WRITE_READ = 0x02

} I2C_POLL_OPERATIONS;




// *****************************************************************************
/* I2C Driver Data Object

  Summary:
    Defines the object required for the maintainence of driver operation queue.

  Description:
    This defines the object required for the maintainence of the software
    clients instance. This object exists once per client instance.

  Remarks:
    None.
*/

typedef struct _I2C_POLL_BUFFER_OBJECT
{

    /* A flag states whether the buffer is in use or not */
    bool                                                    inUse;

    /* A reference to the client instance */
    i2c_poll_system_t *                                             clientHandle;

    /* Slave address high byte, stores 7-bit address 
       and high byte of 10-bit  address */
    uint8_t                                                 slaveaddresshighbyte;
    
    /* Stores low byte of 10 bit address, 
       stores 0 when 7-bit address is used */
    uint8_t                                                 slaveaddresslowbyte;

    /* Specifies the operation */
    I2C_POLL_OPERATIONS                                      operation;

    /* Pointer to transmit buffer */
    uint8_t *                                               txBuffer;

    /* Pointer to receive buffer */
    uint8_t *                                               rxBuffer;

    /* size of buffer for transfer */
    uint32_t                                                transferSize;

    /* size of data to be received */
    uint32_t                                                readtransferSize;

    /*size of buffer actually transferred - used in Slave mode */
    uint32_t                                                actualtransfersize;

    /* perform a force write */
    bool                                                    transmitForced;
    
    /* ignore any error events */
    I2C_POLL_BUS_ERROR_EVENT                                 errorEvent;
    
    
    /* Transfer status */
    I2C_POLL_BUFFER_EVENT                                    status;
    
    

    /* Pointer to the next element in the queue */
    void                                                    *next;

    void                                                    *context;

} i2c_poll_buffer_object_t;

typedef enum
{

    /* I2C BUS LEVEL */
    I2C_POLL_OPEN_COLLECTOR_LEVEL    /*DOM-IGNORE-BEGIN*/  = 0 /*DOM-IGNORE-END*/,

    /* SMBus level */
    I2C_POLL_SMBus_LEVEL             /*DOM-IGNORE-BEGIN*/  = 1 /*DOM-IGNORE-END*/

} I2C_POLL_BUS_LEVEL;

typedef enum
{
    I2C_POLL_NORMAL_SPEED    /*DOM-IGNORE-BEGIN*/  = 0 /*DOM-IGNORE-END*/,

    I2C_POLL_HIGH_SPEED      /*DOM-IGNORE-BEGIN*/  = 1 /*DOM-IGNORE-END*/

}I2C_POLL_BUS_SPEED;


typedef uint8_t                     I2C_POLL_MUTEX_TYPE;
#define I2C_POLL_MUTEX_DECLARE(mutexID)     I2C_POLL_MUTEX_TYPE    mutexID



typedef void ( *I2C_POLL_BUFFER_EVENT_HANDLER ) (I2C_POLL_BUFFER_EVENT event,
                    i2c_poll_buffer_object_t * bufferHandle, uintptr_t context );

typedef struct i2c_poll_system_s
{
    /* The status of the driver */
    I2C_POLL_STATUS                                  status;

    /* The peripheral Id associated with the object */
    I2C_POLL_MODULE_ID                               i2cId; 
    
    
    /* PORT which SCL belongs */
    I2C_POLL_PORTS_CHANNEL                          portSCL;
    
    /* Bit position in the port for SCL */
    I2C_PORTS_BIT_POS                               pinSCL;
    
    /* PORT which SDA belongs */
    I2C_POLL_PORTS_CHANNEL                          portSDA;
    
    /* Bit position in the port for SDA */
    I2C_PORTS_BIT_POS                               pinSDA;

    /* Save the index of the driver. Important to know this
    as we are using reference based accessing */
    I2C_MODULE_INDEX                            drvIndex;

    /* Flag to indicate in use  */
    bool                                        inUse;

    /* Flag to indicate that I2C is used in exclusive access mode */
    bool                                        isExclusive;

    /* Number of clients possible with the hardware instance */
    uint8_t                                     numClients;

    /* Hardware initialization parameters */
    /* I2C Usage Mode Type */
    I2C_POLL_MODE                                i2cMode;

    /* Address Width */
    I2C_POLL_ADDRESS_WIDTH                       addWidth;

    /* Reserved Address rule enable */
    bool                                        reservedaddenable;

    
    /* Baud Rate Value */
    uint32_t                                baudRate;
    
    /* I2C Clock mode */
    I2C_POLL_BUS_LEVEL                           buslevel;
    
     /*  I2C module main state */
    I2C_POLL_MODULE_MAIN_STATE                   modulemainstate;

    /* State of the task */
    I2C_POLL_DATA_OBJECT_TASK                    task;

    /* Queue head is specific to the instance */
    i2c_poll_buffer_object_t                       *queueHead;         
    
    I2C_POLL_BUS_SPEED                       busspeed;

    /*Queue tail is specific to the instance */
    i2c_poll_buffer_object_t                       *queueTail;      
    
    uint32_t                                    queueIn;
    
    uint32_t                                    queueOut;

    uint8_t                                  mstrInterruptSource;

    /* Store the last client handle for every task */
    I2C_POLL_HANDLE                                  lastClientHandle;

    i2c_poll_buffer_object_t                       *taskLObj;

    /* Hardware instance mutex */
    I2C_POLL_MUTEX_DECLARE(mutexDriverInstance);

    /* This callback is fired when an operation is about to start on the
       I2C bus.  This allows the user to set any pins that need to be set.
       This callback may be called from an ISR so should not include OSAL
       calls.  The context parameter is the same one passed into the
       BufferAddRead, BufferAddWrite, BufferAddWriteRead function.
     */
    I2C_POLL_BUFFER_EVENT_HANDLER                            callback;
    
   
    uint32_t                                    timeout;

} i2c_poll_system_t;

#define I2C_POLL_BUS_IDLE_BITS						0x1F

#define I2C_MAX_QUEUE_LENGTH       		7       //multiple of 2 -1
#define DRV_I2C_INSTANCES_NUMBER 3

#define ADDRESS_7BIT_UPPER_LIMIT                    0xFF

void i2c_poll_task(i2c_poll_system_t *i2c_poll_system);

//for tx only supply readbuffer == NULL
i2c_poll_buffer_object_t * I2C_POLL_TransmitThenReceive (   i2c_poll_system_t * dObj,
                                                        uint16_t address,
                                                        void *writeBuffer,
                                                        size_t writeSize,
                                                        void *readBuffer,
                                                        size_t readSize,
                                                        void * callbackContext);


I2C_POLL_BUFFER_EVENT I2C_POLL_TransferStatusGet( i2c_poll_buffer_object_t *  bufferHandle);
I2C_POLL_BUFFER_EVENT I2C_POLL_BufferStatus (  i2c_poll_buffer_object_t *  bufferHandle );
i2c_poll_system_t * I2C_POLL_Initialize ( I2C_POLL_MODULE_ID drvIndex, const i2c_poll_system_t * i2cInit );
uint8_t I2C_POLL_BAUD_Reinit(i2c_poll_system_t *i2cInit);

typedef struct i2c_poll_regs {
   __I2C1CONbits_t I2CxCON;
   volatile unsigned int I2CxCONCLR;
   volatile unsigned int I2CxCONSET;
   volatile unsigned int I2CxCONINV;
   __I2C1STATbits_t I2CxSTAT;
   volatile unsigned int I2CxSTATCLR;
   volatile unsigned int I2CxSTATSET;
   volatile unsigned int I2CxSTATINV;
   volatile unsigned int I2CxADD;
   unsigned int DONTUSE1[3];
   volatile unsigned int I2CxMSK;
   unsigned int DONTUSE2[3];
   volatile unsigned int I2CxBRG;
   unsigned int DONTUSE3[3];
   volatile unsigned int I2CxTRN;
   unsigned int DONTUSE4[3];
   volatile unsigned int I2CxRCV;
   unsigned int DONTUSE5[3];
} i2c_poll_register_t;

#define I2CxCON_SEN_MASK	_I2C1CON_SEN_MASK    /* start */
#define I2CxCON_RSEN_MASK	_I2C1CON_RSEN_MASK   /* repeat-start*/
#define I2CxCON_PEN_MASK	_I2C1CON_PEN_MASK    /* stop */
#define I2CxCON_RCEN_MASK	_I2C1CON_RCEN_MASK   /* receive */
#define I2CxCON_ACKEN_MASK	_I2C1CON_ACKEN_MASK  /* ack */
#define I2CxCON_ACKDT_MASK	_I2C1CON_ACKDT_MASK  /* ack data */
#define I2CxCON_STREN_MASK	_I2C1CON_STREN_MASK  /* clock stretch */
#define I2CxCON_A10M_MASK	_I2C1CON_A10M_MASK   /* 10-bit address */
#define I2CxCON_SCLREL_MASK _I2C1CON_SCLREL_MASK /* hold clock */
#define I2CxCON_ON_MASK     _I2C1CON_ON_MASK     /* Enable */
#define I2CxCON_SIDL_MASK   _I2C1CON_SIDL_MASK   /* idle */
#define I2CxCON_STRICT_MASK _I2C1CON_STRICT_MASK /* Strict */
#define I2CxCON_DISSLW_MASK _I2C1CON_DISSLW_MASK /* Slew */
#define I2CxCON_SMEN_MASK   _I2C1CON_SMEN_MASK   /* SMBus */
#define I2CxCON_GCEN_MASK   _I2C1CON_GCEN_MASK   /* General Call */
#define I2CxCON_PCIE_MASK   _I2C1CON_PCIE_MASK   /* Interrupt on Stop */
#define I2CxCON_SCIE_MASK   _I2C1CON_SCIE_MASK   /* Interrupt on Start */
#define I2CxCON_BOEN_MASK   _I2C1CON_BOEN_MASK   /* Interrupt on Buffer Over */
#define I2CxCON_SBCDE_MASK  _I2C1CON_SBCDE_MASK  /* Int on Bus collision */
#define I2CxCON_AHEN_MASK   _I2C1CON_AHEN_MASK   /* Address Hold Enable */
#define I2CxCON_DHEN_MASK   _I2C1CON_DHEN_MASK   /* Hold enable */

/* Status */
#define I2CxSTAT_I2COV_MASK _I2C1STAT_I2COV_MASK /* overflow */
#define I2CxSTAT_IWCOL_MASK _I2C1STAT_IWCOL_MASK /* Write Collision */
#define I2CxSTAT_BCL_MASK   _I2C1STAT_BCL_MASK   /* Bus Collision */
#define I2CxSTAT_P_MASK     _I2C1STAT_P_MASK     /* stop */
#define I2CxSTAT_S_MASK     _I2C1STAT_S_MASK     /* start */


#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */

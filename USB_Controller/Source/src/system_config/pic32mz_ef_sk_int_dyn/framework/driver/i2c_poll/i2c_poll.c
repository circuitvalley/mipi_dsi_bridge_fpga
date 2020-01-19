/* ************************************************************************** */
/** Descriptive File Name

  @Company
 * Gaurav Singh
   www.circuitvalley.com

  @File Name
    i2c_poll.c

  @Summary
    minimum overhead nonblocking i2c driver for pic32

 */
/* ************************************************************************** */
#include "i2c_poll.h"
#include "system_config.h"
static i2c_poll_buffer_object_t I2C_POLL_Buffer[DRV_I2C_INSTANCES_NUMBER][I2C_MAX_QUEUE_LENGTH];

static i2c_poll_system_t             gDrvI2CObj[DRV_I2C_INSTANCES_NUMBER] ;

i2c_poll_buffer_object_t* _I2C_POLL_QueueSlotGet ( i2c_poll_system_t *dObj );
i2c_poll_buffer_object_t* _I2C_POLL_QueuePop( i2c_poll_system_t *dObj);
void _I2C_POLL_QueuePush(i2c_poll_system_t *dObj, i2c_poll_buffer_object_t *buf);
void _I2C_POLL_Advance_Queue( i2c_poll_system_t *dObj );


bool I2C_POLL_MUTEX_lock(I2C_POLL_MUTEX_TYPE* mutexID)
{
    if (*mutexID == 1)
    {
        *mutexID = 0;
        return true;
    }
    
    return false;
}

bool  I2C_POLL_MUTEX_unlock(I2C_POLL_MUTEX_TYPE* const mutexID)
{
    *mutexID = 1;
    return true;
}


// *****************************************************************************
/* Function:
    I2C_POLL_BUFFER_OBJECT* _I2C_POLL_QueueSlotGet ( I2C_POLL_OBJ *dObj )

  Summary:
    Adds an element to the queue.

  Description:
    This API adds an element to the queue.

  Parameters:
    i2cDataObj   - Pointer to the structure which holds the data which is to be
    				added to the queue.

  Returns:
    I2C_POLL_BUFFER_HANDLE - Handle, a pointer to the allocated element in the
    						queue.
*/
i2c_poll_buffer_object_t* _I2C_POLL_QueueSlotGet ( i2c_poll_system_t *dObj )
{
    uint8_t numOfFreeQueueSlots;
    i2c_poll_buffer_object_t *lQueueObj;
    
    if (I2C_POLL_MUTEX_lock(&(dObj->mutexDriverInstance)))
    {
        
        /* if position of item populated in the queue (Queue-In pointer) is greater
         * than the position where the item is to be taken out of the queue,
         * (Queue-Out pointer) then the number of free slots is the remainder of the
         * slots excluding the slot index where Queue Out pointer resides to the
         * slot index where Queue-In pointer resides.
         * Ex: # of Queue Slots = 6, Queue-In = 5 and Queue-Out = 2
         * Number of free queue available  = 6 -(5-2) - 1 = 2 (free slots - 6 & 1)
         *
         * if Queue-Out pointer is greater than Queue-In pointer but the Queue Out
         * pointer is adjacent to Queue-In pointer (Queue-Out - Queue-In == 1)
         * then buffer is full. Return 0 slots available if it's the case.
         * If Queue-Out pointer is greater than Queue-In pointer then slots starting
         * from Queue-In pointer to Queue-Out pointer is number of available queue
         * slots
         * Ex: # of Queue Slots = 6, Queue-Out = 4 and Queue-In = 2
         * Number of free queue slots available  = 4-2-1 (free slots - 3)
         */
        
        if (dObj->queueIn >= dObj->queueOut)
        {
            numOfFreeQueueSlots =  (I2C_MAX_QUEUE_LENGTH - (dObj->queueIn - dObj->queueOut) -1);
        }
        else
        {
            numOfFreeQueueSlots = ((dObj->queueOut - dObj->queueIn) -1);
        }
        
        if (numOfFreeQueueSlots > 0)
        {
            lQueueObj = &I2C_POLL_Buffer[ dObj->drvIndex ][ dObj->queueIn ];
            
            dObj->queueIn = (dObj->queueIn+1) & I2C_MAX_QUEUE_LENGTH;
            
            
            /* Release mutex */
            I2C_POLL_MUTEX_unlock(&(dObj->mutexDriverInstance));
            
            return lQueueObj;
        }
        
    }
        /* Release mutex */
        I2C_POLL_MUTEX_unlock(&(dObj->mutexDriverInstance));
        
       return (i2c_poll_buffer_object_t*)NULL;
}


// *****************************************************************************
/* Function:
    I2C_POLL_BUFFER_OBJ* _I2C_POLL_QueuePush (I2C_POLL_OBJ *dObj, I2C_POLL_BUFFER_OBJECT *buf)

  Summary:
 Adds a I2C_POLL_OBJ into a I2C Buffer Object

  Description:
    This API adds an I2C_POLL_OBJ into a buffer containing I2C objects

  Parameters:
    i2cDataObj   - Pointer to the data structure to be removed from the queue
    i2cbufferObj - Pointer to buffer where i2c objects are stored
  Returns:
    None
*/

void _I2C_POLL_QueuePush(i2c_poll_system_t *dObj, i2c_poll_buffer_object_t *buf)
{
    dObj->queueHead = &I2C_POLL_Buffer[dObj->drvIndex][dObj->queueIn];
}



// *****************************************************************************
/* Function:
    I2C_POLL_BUFFER_OBJ* _I2C_POLL_QueuePop ( I2C_POLL_OBJ *dObj )

  Summary:
    Removes a element from the queue

  Description:
    This API removes an element from the queue

  Parameters:
    i2cDataObj   - Pointer to the data structure to be removed from the queue

  Returns:
    I2C_POLL_BUFFER_HANDLE - Handle, a pointer to the next allocated element in the
    						queue.
*/

i2c_poll_buffer_object_t * _I2C_POLL_QueuePop( i2c_poll_system_t *dObj)
{

    // Make sure that the pointer is valid inside out allocated space.
    if (dObj < &gDrvI2CObj[0] || dObj > &gDrvI2CObj[DRV_I2C_INSTANCES_NUMBER-1])
    {
            return NULL;
    }
    // Make sure that this driver instance is actually in use.
    if (dObj->inUse != true)
    {
            return NULL;
    }

    i2c_poll_buffer_object_t * ret = &I2C_POLL_Buffer [ dObj->drvIndex ][ dObj->queueOut ];
    dObj->queueHead = NULL;
    


    return ret;
}


void _I2C_POLL_Advance_Queue( i2c_poll_system_t *dObj )
{

    if (dObj->i2cMode == I2C_POLL_MODE_MASTER && (dObj->queueOut != dObj->queueIn))
    {
        dObj->queueOut = (dObj->queueOut + 1) & I2C_MAX_QUEUE_LENGTH;
    }
}

// *****************************************************************************
/* Function:
    I2C_POLL_BUFFER_OBJ* _I2C_POLL_IsQueueEmpty (I2C_POLL_OBJ *dObj)

  Summary:
    Checks if the queue is empty

  Description:
    This API checks if the end of queue has been reached or if the tail pointer
    is at the same location as the head pointer. If the tail pointer and head
    pointer coincide, it implies that the queue is empty

  Parameters:
    i2cDataObj   - Pointer to the data structure to be removed from the queue
    i2cbufferObj - Pointer to buffer where i2c objects are stored
  Returns:
    None
*/

bool _I2C_POLL_IsQueueEmpty(i2c_poll_system_t *dObj)
{

//    I2C_POLL_BUFFER_OBJECT * temp;
    // Make sure that the pointer is valid inside out allocated space.
    if (dObj < &gDrvI2CObj[0] || dObj > &gDrvI2CObj[DRV_I2C_INSTANCES_NUMBER-1])
    {
        return false;
    }
    // Make sure that this driver instance is actually in use.
    if (dObj->inUse != true)
    {
        return false;
    }


    if (dObj->queueOut == dObj->queueIn)
        return true;

    return false;


}


//******************************************************************************
/* Function:
    i2c_poll_buffer_object_t * I2C_POLL_TransmitReceive ( DRV_HANDLE handle,
                                                    uint8_t* address,
                                                    void *txBuffer,
                                                    size_t wsize,
                                                    void *rxBuffer,
                                                    size_t rsize,
                                                    void * context);

  Summary:
    This function writes data to Slave, inserts restart and requests read from
    slave.

  Description:
    Master calls this function to send a register address value to the slave and
    then queries the slave with a read request to read the contents indexed by
    the register location. The Master sends a restart condition after the
    initial write before sending the device address with R/W = 1. The restart
    condition prevents the Master from relinquishing the control of the bus. The
    slave should not use this function. Driver will process this request
    in the task routine.

  Parameters:
    handle      - A valid open-instance handle, returned from the driver's open
                  routine
    address     - Device address of slave. If this API is used in Slave mode,
                  then a dummy value can be used
    writeBuffer - Contains data to be transferred
    writeSize   - The number of bytes that the Master expects to write to Slave.
                  This value can be kept as the MAX BUFFER SIZE for slave.
                  This is because the Master controls when the WRITE operation
                  is terminated.
    readBuffer  - This buffer holds data that is send back from slave after
                  read operation.
    readSize    - The number of bytes the Master expects to be read from the
                  slave
    callbackContext     - Not implemented, future expansion

  Returns:
    i2c_poll_buffer_object_t * using which application can track the current status of
    the buffer.
*/

i2c_poll_buffer_object_t * I2C_POLL_TransmitThenReceive   (   i2c_poll_system_t * dObj,
                                                        uint16_t address,
                                                        void *writeBuffer,
                                                        size_t writeSize,
                                                        void *readBuffer,
                                                        size_t readSize,
                                                        void * callbackContext)
{
    i2c_poll_buffer_object_t *bufferObject;

     /* Get a slot in the queue */
    bufferObject = _I2C_POLL_QueueSlotGet (dObj);

    if ( bufferObject != NULL )
    {
      /* Fill the data directly to the queue. Set the inUse flag only at the end */
        bufferObject->clientHandle        = dObj;
        if (address > ADDRESS_7BIT_UPPER_LIMIT )
        {
            bufferObject->slaveaddresshighbyte = (uint8_t)((address & 0xFF00)>>8);
            bufferObject->slaveaddresslowbyte  = (uint8_t)(address & 0x00FF);
        }
        else
        {
            bufferObject->slaveaddresshighbyte = (uint8_t)(address & 0x00FF);
            bufferObject->slaveaddresslowbyte  = 0;
        }
        if (readBuffer == NULL)
        {
            bufferObject->operation = I2C_POLL_OP_WRITE;
            bufferObject->readtransferSize    = 0;
        }
        else
        {
            bufferObject->operation    = I2C_POLL_OP_WRITE_READ;
            bufferObject->readtransferSize    = readSize;
        }
        bufferObject->txBuffer            = writeBuffer;
        bufferObject->transferSize        = writeSize;
        bufferObject->rxBuffer            = readBuffer;
        bufferObject->actualtransfersize  = 0;
        bufferObject->status              = I2C_POLL_BUFFER_EVENT_PENDING;
        bufferObject->context             = callbackContext;
        bufferObject->transmitForced      = false;

        _I2C_POLL_QueuePush( dObj, bufferObject);

        if ( dObj->i2cMode == I2C_POLL_MODE_MASTER)
        {

                /*  if either START and STOP were not detected which is true the
                first time OR if STOP was detected, then it assumed the
                transaction on the bus is complete */

            volatile i2c_poll_register_t *regs = (i2c_poll_register_t *)dObj->i2cId;
            
            if ( (!((bool)regs->I2CxSTAT.S) && !((bool)regs->I2CxSTAT.P)) || (bool)regs->I2CxSTAT.P ) //if start and stop not detected or just stop detected
            {
                volatile uint32_t *i2c_ctrl_reg = (volatile uint32_t *)&regs->I2CxCON;
                
              //  volatile uint32_t *IFSx = (volatile uint32_t *)(&IFS0 + ((0x10 * (dObj->mstrInterruptSource / 32)) / 4));                
                /* if Bus IDLE and I2CxMIF = 0, then I2C is not running*/

                if (( (bool)!((*i2c_ctrl_reg & I2C_POLL_BUS_IDLE_BITS) || regs->I2CxSTAT.TRSTAT) ))
                {
                    dObj->task = I2C_POLL_TASK_SEND_DEVICE_ADDRESS;
                    regs->I2CxCONSET = I2CxCON_SEN_MASK;        //send start
                }else
                {
                    return (i2c_poll_buffer_object_t *)NULL; //i2c took too long to get idle               
                }
            }
        }

        return bufferObject;
    }
     return (i2c_poll_buffer_object_t *)NULL;
} /* I2C_POLL_TransmitThenReceive */





void i2c_poll_task(i2c_poll_system_t *i2c_poll_system)
{
    i2c_poll_buffer_object_t * bufferObject = i2c_poll_system->taskLObj;

    if ((uint32_t)i2c_poll_system == I2C_POLL_SYSTEM_INVALID )
    {
        return;
    }
    volatile i2c_poll_register_t * regs = (i2c_poll_register_t *)i2c_poll_system->i2cId;
         volatile uint32_t *i2c_ctrl_reg = (volatile uint32_t *)&regs->I2CxCON;

    switch(i2c_poll_system->task)
    {
        case I2C_POLL_TASK_SEND_DEVICE_ADDRESS:
        {

            if (i2c_poll_system->i2cMode == I2C_POLL_MODE_MASTER && i2c_poll_system->queueHead != NULL && (bool)!(*i2c_ctrl_reg & I2C_POLL_BUS_IDLE_BITS)) //master mode , there is someting in queue and start bit is idle
            {
                i2c_poll_system->taskLObj = _I2C_POLL_QueuePop(i2c_poll_system);
                bufferObject = i2c_poll_system->taskLObj;

                if (bufferObject->operation == I2C_POLL_OP_READ)
                {
                    regs->I2CxTRN = (I2C_POLL_OP_READ | bufferObject->slaveaddresshighbyte);
                    
                    bufferObject->status = I2C_POLL_BUFFER_SLAVE_READ_REQUESTED;
                    i2c_poll_system->modulemainstate = I2C_POLL_MASTER_RX_FROM_SLAVE;
                    i2c_poll_system->task = I2C_POLL_TASK_SET_RCEN_ONLY;
                }
                else
                {
                    regs->I2CxTRN = bufferObject->slaveaddresshighbyte;
                    
                    if (bufferObject->slaveaddresslowbyte)
                    {
                        i2c_poll_system->task = I2C_POLL_SEND_DEVICE_ADDRESS_BYTE_2;
                    }
                    else
                    {
                        bufferObject->status = I2C_POLL_BUFFER_SLAVE_WRITE_REQUESTED;
                        i2c_poll_system->task = bufferObject->operation + I2C_POLL_TASK_PROCESS_WRITE_ONLY;
                    }
                }
            }
            
            break;
        }
        case I2C_POLL_SEND_DEVICE_ADDRESS_BYTE_2:
        {
                 if (bufferObject->operation == I2C_POLL_OP_READ)
                {
                     bufferObject->status = I2C_POLL_BUFFER_SLAVE_READ_REQUESTED;
                     i2c_poll_system->modulemainstate = I2C_POLL_MASTER_RX_FROM_SLAVE;                     
                     i2c_poll_system->task = I2C_POLL_TASK_SET_RCEN_ONLY;
                }
                else
                {
                     bufferObject->status = I2C_POLL_BUFFER_SLAVE_WRITE_REQUESTED;
                     i2c_poll_system->task = bufferObject->operation + I2C_POLL_TASK_PROCESS_WRITE_ONLY;
                }
                 regs->I2CxTRN = bufferObject->slaveaddresslowbyte>>1;

            break;
        }
        case I2C_POLL_RESEND_READ_DEVICE_ADDRESS:
        {
            i2c_poll_system->modulemainstate = I2C_POLL_MASTER_RX_FROM_SLAVE;
            uint16_t delay = 1500;
            while(delay--);
            regs->I2CxTRN = (I2C_POLL_OP_READ | bufferObject->slaveaddresshighbyte);
            i2c_poll_system->task = I2C_POLL_TASK_SET_RCEN_ONLY;
            break;
        }
        case I2C_POLL_TASK_PROCESS_READ_ONLY:
        {
            if (i2c_poll_system->i2cMode == I2C_POLL_MODE_MASTER && bufferObject->transferSize)
            {
                i2c_poll_system->modulemainstate = I2C_POLL_MASTER_RX_FROM_SLAVE;
                
                if ((bool)regs->I2CxSTAT.RBF) //if received byte available 
                {
                    *bufferObject->rxBuffer++ = (uint8_t)regs->I2CxRCV;
                   
                    if (bufferObject->transferSize > 1)
                    {
                        i2c_poll_system->task = I2C_POLL_TASK_SET_RCEN_ONLY;
                        if ((bool)!(*i2c_ctrl_reg & I2C_POLL_BUS_IDLE_BITS))
                        {
                            bufferObject->status = I2C_POLL_BUFFER_MASTER_ACK_SEND;
                            regs->I2CxCONCLR = I2CxCON_ACKDT_MASK;
                            regs->I2CxCONSET = I2CxCON_ACKEN_MASK; 
                        }
                    }
                    else
                    {
                        volatile uint32_t *i2c_ctrl_reg = (volatile uint32_t *)&regs->I2CxCON;
                        
                        if ((bool)!(*i2c_ctrl_reg & I2C_POLL_BUS_IDLE_BITS)) //if master receiver read to ack
                        {
                            bufferObject->status = I2C_POLL_BUFFER_MASTER_NACK_SEND;
                            
                            regs->I2CxCONSET = I2CxCON_ACKDT_MASK | I2CxCON_ACKEN_MASK; //send Nack
                        }
                           
                        i2c_poll_system->task = I2C_POLL_BUS_SILENT;
                    }

                    bufferObject->transferSize--;
                    bufferObject->actualtransfersize++;
                }
            }
            break;
        }
        case I2C_POLL_TASK_PROCESS_WRITE_ONLY:
        case I2C_POLL_TASK_PROCESS_WRITE_READ:
        {
            if (i2c_poll_system->i2cMode == I2C_POLL_MODE_MASTER)
            {
                if (bufferObject->transferSize)
                {
                    if (!regs->I2CxSTAT.TRSTAT)//transmit byte has completed
                    {
                        if ((bool)!regs->I2CxSTAT.ACKSTAT || bufferObject->transmitForced) //if tramitter byte was Acked
                        {
                            regs->I2CxTRN = *bufferObject->txBuffer++;
                            
                            if (bufferObject->transferSize == 1)
                            {    

                              i2c_poll_system->task = I2C_POLL_BUS_SILENT ; 
                            }
                            
                            bufferObject->actualtransfersize++;
                            bufferObject->transferSize --;
                        }
                        else
                        {
                            i2c_poll_system->task = I2C_POLL_TASK_PROCESS_STOP;
                            regs->I2CxCONSET = I2CxCON_PEN_MASK;  //Master Stop Condition
                        }
                        
                    }
                }
                else
                {
                    i2c_poll_system->task = I2C_POLL_TASK_PROCESS_STOP;
                    regs->I2CxCONSET = I2CxCON_PEN_MASK;  //Master Stop Condition
                }
            }
            break;
        }
        case I2C_POLL_BUS_SILENT:
        {
            /*  The Bus is Silent/Idle when the last byte is either ACK'ed  OR
             in the event of slave unexpectedly aborting operation, check
             if transmission is complete and NACK is received   */
            uint16_t delay = 1500;
            while(delay--);
            

             if ( (!(regs->I2CxSTAT.TRSTAT)) )
             {
                 if (bufferObject->operation == I2C_POLL_OP_WRITE_READ)
                 {
                    i2c_poll_system->task = I2C_POLL_RESEND_READ_DEVICE_ADDRESS;
                    bufferObject->operation = I2C_POLL_OP_READ;
                    
                    bufferObject->transferSize = bufferObject->readtransferSize;
                    
                    bufferObject->status = I2C_POLL_SEND_RESTART_EVENT;
                    
                    regs->I2CxCONSET = I2CxCON_RSEN_MASK; //Master Send restart condition

                 }
                 else
                 {    
                         i2c_poll_system->task = I2C_POLL_TASK_PROCESS_STOP;
                         regs->I2CxCONSET = I2CxCON_PEN_MASK;  //Master Stop Condition
                 }
             }
             
            break;
        }
        case I2C_POLL_TASK_SET_RCEN_ONLY:
        {
            if ((bool)!regs->I2CxSTAT.TRSTAT)//byte has completed
            {
                if ((bool)!regs->I2CxSTAT.ACKSTAT) // txbyte was acked
                {
                    if ((bool)!regs->I2CxCON.ACKEN) //rx byte ack completed
                    {
                        
                            regs->I2CxCONSET = I2CxCON_RCEN_MASK; //clock to receive 1 byte
                            i2c_poll_system->task = I2C_POLL_TASK_PROCESS_READ_ONLY;
                    }
                }
                else
                {
                    i2c_poll_system->task = I2C_POLL_TASK_PROCESS_STOP;
                    regs->I2CxCONSET = I2CxCON_PEN_MASK;  //Master Stop Condition
                }
            }
            
            break;
        }
        case I2C_POLL_TASK_PROCESS_STOP:
        {
            static uint8_t wait_for_stop = 0;
            if (wait_for_stop++ == 10)
            {
                wait_for_stop = 0;
                i2c_poll_system->task = I2C_POLL_TASK_SEND_DEVICE_ADDRESS;
                bufferObject->status = I2C_POLL_BUFFER_EVENT_ERROR;   
            }
            
            if ((bool)!((*i2c_ctrl_reg & I2C_POLL_BUS_IDLE_BITS)))
            {
                wait_for_stop = 0;
                i2c_poll_system->task = I2C_POLL_TASK_SEND_DEVICE_ADDRESS;
                if (bufferObject->transferSize)
                {
                    bufferObject->status = I2C_POLL_BUFFER_EVENT_ERROR;   
                }
                else
                {
                    bufferObject->status = I2C_POLL_BUFFER_EVENT_COMPLETE;
                }
                
                if ( bufferObject->clientHandle->callback != NULL)
                {
                    bufferObject->clientHandle->callback(bufferObject->status, bufferObject, 0x0);
                }
                
                if (bufferObject)
                {
                    bufferObject->inUse = false;
                    Nop();
                }
            }
            break;    
        }
        default:
        {
            break;
        }
    }

    if (i2c_poll_system->i2cMode == I2C_POLL_MODE_MASTER)
    {
        if ((bufferObject != NULL) && ( (bufferObject->status == I2C_POLL_BUFFER_EVENT_COMPLETE) || (bufferObject->status == I2C_POLL_BUFFER_EVENT_ERROR)))
        {
            _I2C_POLL_Advance_Queue(i2c_poll_system);
            i2c_poll_system->taskLObj = NULL;
            volatile uint32_t *i2c_ctrl_reg = (volatile uint32_t *)&regs->I2CxCON;

            if (_I2C_POLL_IsQueueEmpty(i2c_poll_system) == false && (bool)!((*i2c_ctrl_reg & I2C_POLL_BUS_IDLE_BITS) || regs->I2CxSTAT.TRSTAT))
            {
                    regs->I2CxCONSET = I2CxCON_SEN_MASK; //send start
                    Nop();
            }
        }
    }
}






//******************************************************************************
/* Function:
    static void _I2C_POLL_SetupHardware ( const I2C_MODULE_ID   plibId,
                                        I2C_POLL_OBJ_HANDLE     dObj,
                                        I2C_POLL_INIT         * i2cInit )

  Summary:
    Sets up the hardware from the initialization structure

  Description:
    This routine sets up the hardware from the initialization structure.

  Remarks:
    None.
*/

static void _I2C_POLL_SetupHardware ( const I2C_POLL_MODULE_ID plibId,
                                     i2c_poll_system_t *dObj,
                                     const i2c_poll_system_t * i2cInit )
{
  volatile i2c_poll_register_t *regs = (i2c_poll_register_t *)plibId;

    
    /* Power state initialization */
  regs->I2CxCONSET = I2CxCON_SIDL_MASK;

    /* Set I2C operational mode -- Master or Slave */
    dObj->i2cMode = i2cInit->i2cMode;
    /* assign SCL and SDA ports for bit banging purposes*/
    dObj->portSCL   = i2cInit->portSCL;
    dObj->pinSCL    = i2cInit->pinSCL;
    dObj->portSDA   = i2cInit->portSDA;
    dObj->pinSDA    = i2cInit->pinSDA;


     /* Set Baud Rate */
     if ( I2C_POLL_MODE_MASTER == i2cInit->i2cMode)
     {

        regs->I2CxBRG = (SYS_CLK_BUS_PERIPHERAL_1 / i2cInit->baudRate / 2) - (SYS_CLK_BUS_PERIPHERAL_1 / 10000000) - 2;
        dObj->modulemainstate = I2C_POLL_MODULE_IDLE;
     }

     /* Set SLEW rate based on baud-rate; if baud-rate is either <= 100k
        OR baud-rate = 1M; I2C2xCON.DISSLW = 1  */
    if (i2cInit->baudRate <= 100000 ||i2cInit->baudRate == 1000000 )
    {
        regs->I2CxCONSET = I2CxCON_DISSLW_MASK; //High freq On 

    }
     else
    {
        regs->I2CxCONCLR = I2CxCON_DISSLW_MASK; //High freq Off
    }

    /* SMBus Input Level */
    dObj->buslevel = i2cInit->buslevel;

    /* Allow reserved slave address */
     dObj->reservedaddenable = i2cInit->reservedaddenable;

} /* _I2C_POLL_SetupHardware */



// *****************************************************************************
// *****************************************************************************
// Section: Driver Interface Function Definitions
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
/* Function:
    SYS_MODULE_OBJ I2C_POLL_Initialize ( const SYS_MODULE_INDEX  index,
                                       const SYS_MODULE_INIT * const init )

  Summary:
    Initializes hardware and data for the given instance of the I2C module

  Description:
    This routine initializes hardware for the instance of the I2C module,
    using the hardware initialization given data.  It also initializes all
    necessary internal data.

  Parameters:
    index           - Identifies the driver instance to be initialized

    init            - Pointer to the data structure containing all data
                      necessary to initialize the hardware. This pointer may
                      be null if no data is required and static initialization
                      values are to be used.

  Returns:
    If successful, returns a valid handle to a driver instance object.
    Otherwise, it returns SYS_MODULE_OBJ_INVALID.
*/

uint8_t I2C_POLL_BAUD_Reinit(i2c_poll_system_t *i2cInit)
{
    volatile i2c_poll_register_t *regs = (i2c_poll_register_t *)i2cInit->i2cId;
    
    if (i2cInit->task == I2C_POLL_TASK_SEND_DEVICE_ADDRESS) //i2c is idle
    {
        regs->I2CxCONCLR = I2CxCON_ON_MASK;
        if ( I2C_POLL_MODE_MASTER == i2cInit->i2cMode)
        {   
            regs->I2CxBRG = (SYS_CLK_BUS_PERIPHERAL_1 / i2cInit->baudRate / 2) - (SYS_CLK_BUS_PERIPHERAL_1 / 10000000) - 2;
            
        }
        
        
        /* Set SLEW rate based on baud-rate; if baud-rate is either <= 100k
         OR baud-rate = 1M; I2C2xCON.DISSLW = 1  */
        if (i2cInit->baudRate <= 100000 || i2cInit->baudRate == 1000000 )
        {
            regs->I2CxCONSET = I2CxCON_DISSLW_MASK; //High freq On 
            
        }
        else
        {
            regs->I2CxCONCLR = I2CxCON_DISSLW_MASK; //High freq Off
        }
        regs->I2CxCONSET = I2CxCON_ON_MASK;
        
        return 0;
    }
    return (uint8_t)I2C_POLL_SYSTEM_INVALID;
}

i2c_poll_system_t * I2C_POLL_Initialize ( I2C_POLL_MODULE_ID drvIndex,
                                   const i2c_poll_system_t * i2cInit )
{
    volatile i2c_poll_register_t *regs = (i2c_poll_register_t *)i2cInit->i2cId;
    I2C_POLL_MODULE_ID i2cId;

    uint16_t index;
    
    i2c_poll_buffer_object_t *lBufferObj;

    /* Validate the driver index */
    if ( drvIndex > I2C_POLL_NUMBER_OF_MODULES )
    {
        return (i2c_poll_system_t *)I2C_POLL_SYSTEM_INVALID;
    }
    
    i2c_poll_system_t *i2c_poll_system_obj = &gDrvI2CObj[drvIndex];

    /* Cheap dead man's mutex during initialization to make sure two different
       tasks don't try to initialize the same driver at the same time.*/

    if (i2c_poll_system_obj->inUse)
    {
        return (i2c_poll_system_t *)I2C_POLL_SYSTEM_INVALID;
    }

 
    i2c_poll_system_obj->mstrInterruptSource = i2cInit->mstrInterruptSource;
    /* Object is valid, set it in use */
    i2c_poll_system_obj->inUse = true;

    i2c_poll_system_obj->task = I2C_POLL_TASK_SEND_DEVICE_ADDRESS;
    /* Save the index of the driver. Important to know this
    as we are using reference based accessing */
    i2c_poll_system_obj->drvIndex = drvIndex;

    /* Update the I2C Module Index */
    i2c_poll_system_obj->i2cId = i2cInit->i2cId;

    /* set QueueHead to NULL */
    i2c_poll_system_obj->queueHead = NULL;

    /* initialize QueueTail to NULL */
    i2c_poll_system_obj->queueTail = NULL;

    i2c_poll_system_obj->queueIn = 0;

    i2c_poll_system_obj->queueOut = 0;

    /* Speed up accessing, take it to a local variable */
    i2cId = i2c_poll_system_obj->i2cId;
    /* Setup the Hardware */
    _I2C_POLL_SetupHardware ( i2cId, i2c_poll_system_obj, i2cInit );

    /* Reset the number of clients */
    i2c_poll_system_obj->numClients = 0;

    /* Reset the locally used variables */
    i2c_poll_system_obj->lastClientHandle  = DRV_I2C_INSTANCES_NUMBER+1;

    i2c_poll_system_obj->callback = i2cInit->callback;


    for ( index=0; index<I2C_MAX_QUEUE_LENGTH; index++ )
    {
        lBufferObj = &I2C_POLL_Buffer[ drvIndex ][ index ];

        lBufferObj->inUse   = false;
        lBufferObj->next    = NULL;
    }

    /* Set the current driver state */
    i2c_poll_system_obj->status = I2C_POLL_STATUS_READY;


        /* Create the hardware instance mutex. */
    if(I2C_POLL_MUTEX_unlock(&(i2c_poll_system_obj->mutexDriverInstance)) != true)
    {
       return (i2c_poll_system_t *)I2C_POLL_SYSTEM_INVALID;
    }

    /* Enable the I2C module */
    regs->I2CxCONSET = I2CxCON_ON_MASK; //enable i2c
    /* Return the driver handle */
    return (i2c_poll_system_obj );
} /* DRV_I2C_Initialize */



// *****************************************************************************
/* Function:
    I2C_POLL_BUFFER_EVENT I2C_POLL_BufferStatus ( I2C_POLL_BUFFER_HANDLE bufferHandle )

  Summary:
    Returns the transmitter and receiver transfer status

  Description:
    This returns the transmitter and receiver transfer status.

  Parameters:

    handle          - A valid open-instance handle, returned from the driver's
                      open routine
    bufferHandle    - A valid open-instance handle, returned when calling the
                      BufferAddRead/BufferAddWrite/BufferAddReadWrite function

  Returns:
    A I2C_POLL_TRANSFER_STATUS value describing the current status of the
    transfer.
*/

I2C_POLL_BUFFER_EVENT I2C_POLL_TransferStatusGet  ( i2c_poll_buffer_object_t *  bufferHandle )
{
    /* return the transfer status. This doesn't have any protection */

    return bufferHandle->status;

} /* I2C_POLL_TransferStatus */



/* *****************************************************************************
 End of File
 */

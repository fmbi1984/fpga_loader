#ifndef __RING_BUFFER_H__
#define __RING_BUFFER_H__

/* Includes: */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>
  
  /* Type Defines: */
  /** \brief Ring Buffer Management Structure.
   *      *
   *     *  Type define for a new ring buffer object. Buffers should be initialized via a call to
   *     *  \ref RingBuffer_InitBuffer() before use.
       */
  typedef struct
  {
    uint8_t* In; /**< Current storage location in the circular buffer. */
    uint8_t* Out; /**< Current retrieval location in the circular buffer. */
    uint8_t* Start; /**< Pointer to the start of the buffer's underlying storage array. */
    uint8_t* End; /**< Pointer to the end of the buffer's underlying storage array. */
    uint16_t Size; /**< Size of the buffer's underlying storage array. */
    uint16_t Count; /**< Number of bytes currently stored in the buffer. */
  } 
  RingBuffer_t;

  /* Inline Functions: */
  /** Initializes a ring buffer ready for use. Buffers must be initialized via this function
   *      *  before any operations are called upon them. Already initialized buffers may be reset
   *     *  by re-initializing them using this function.
   *     *
   *     *  \param[out] Buffer   Pointer to a ring buffer structure to initialize.
   *     *  \param[out] DataPtr  Pointer to a global array that will hold the data stored into the ring buffer.
   *     *  \param[out] Size     Maximum number of bytes that can be stored in the underlying data array.
       */
  static inline void RingBuffer_InitBuffer(RingBuffer_t* Buffer,
  uint8_t* const DataPtr,
  const uint16_t Size);
  static inline void RingBuffer_InitBuffer(RingBuffer_t* Buffer,
  uint8_t* const DataPtr,
  const uint16_t Size)
  {
    Buffer->In     = DataPtr;
    Buffer->Out    = DataPtr;
    Buffer->Start  = &DataPtr[0];
    Buffer->End    = &DataPtr[Size];
    Buffer->Size   = Size;
    Buffer->Count  = 0;
  }

    /** Retrieves the current number of bytes stored in a particular buffer. This value is computed
   *      *  by entering an atomic lock on the buffer, so that the buffer cannot be modified while the
   *     *  computation takes place. This value should be cached when reading out the contents of the buffer,
   *     *  so that as small a time as possible is spent in an atomic lock.
   *     *
   *     *  \note The value returned by this function is guaranteed to only be the minimum number of bytes
   *     *        stored in the given buffer; this value may change as other threads write new data, thus
   *     *        the returned number should be used only to determine how many successive reads may safely
   *     *        be performed on the buffer.
   *     *
   *     *  \param[in] Buffer  Pointer to a ring buffer structure whose count is to be computed.
   *     *
   *     *  \return Number of bytes currently stored in the buffer.
       */
  static inline uint16_t RingBuffer_GetCount(RingBuffer_t* const Buffer);
  static inline uint16_t RingBuffer_GetCount(RingBuffer_t* const Buffer)
  {
    uint16_t Count;

    Count = Buffer->Count;

    return Count;
  }

  /** Retrieves the free space in a particular buffer. This value is computed by entering an atomic lock
   *      *  on the buffer, so that the buffer cannot be modified while the computation takes place.
   *     *
   *     *  \note The value returned by this function is guaranteed to only be the maximum number of bytes
   *     *        free in the given buffer; this value may change as other threads write new data, thus
   *     *        the returned number should be used only to determine how many successive writes may safely
   *     *        be performed on the buffer when there is a single writer thread.
   *     *
   *     *  \param[in] Buffer  Pointer to a ring buffer structure whose free count is to be computed.
   *     *
   *     *  \return Number of free bytes in the buffer.
       */
  static inline uint16_t RingBuffer_GetFreeCount(RingBuffer_t* const Buffer);
  static inline uint16_t RingBuffer_GetFreeCount(RingBuffer_t* const Buffer)
  {
    return (Buffer->Size - RingBuffer_GetCount(Buffer));
  }

  /** Atomically determines if the specified ring buffer contains any data. This should
   *      *  be tested before removing data from the buffer, to ensure that the buffer does not
   *     *  underflow.
   *     *
   *     *  If the data is to be removed in a loop, store the total number of bytes stored in the
   *     *  buffer (via a call to the \ref RingBuffer_GetCount() function) in a temporary variable
   *     *  to reduce the time spent in atomicity locks.
   *     *
   *     *  \param[in,out] Buffer  Pointer to a ring buffer structure to insert into.
   *     *
   *     *  \return Boolean \c true if the buffer contains no free space, \c false otherwise.
       */
  static inline bool RingBuffer_IsEmpty(RingBuffer_t* const Buffer);
  static inline bool RingBuffer_IsEmpty(RingBuffer_t* const Buffer)
  {
    return (RingBuffer_GetCount(Buffer) == 0);
  }

    /** Atomically determines if the specified ring buffer contains any free space. This should
   *      *  be tested before storing data to the buffer, to ensure that no data is lost due to a
   *     *  buffer overrun.
   *     *
   *     *  \param[in,out] Buffer  Pointer to a ring buffer structure to insert into.
   *     *
   *     *  \return Boolean \c true if the buffer contains no free space, \c false otherwise.
       */
  static inline bool RingBuffer_IsFull(RingBuffer_t* const Buffer);
  static inline bool RingBuffer_IsFull(RingBuffer_t* const Buffer)
  {
    return (RingBuffer_GetCount(Buffer) == Buffer->Size);
  }

  /** Inserts an element into the ring buffer.
   *      *
   *     *  \warning Only one execution thread (main program thread or an ISR) may insert into a single buffer
   *     *           otherwise data corruption may occur. Insertion and removal may occur from different execution
   *     *           threads.
   *     *
   *     *  \param[in,out] Buffer  Pointer to a ring buffer structure to insert into.
   *     *  \param[in]     Data    Data element to insert into the buffer.
       */
  static inline void RingBuffer_Insert(RingBuffer_t* Buffer, const uint8_t Data);
  static inline void RingBuffer_Insert(RingBuffer_t* Buffer, const uint8_t Data)
  {
    *Buffer->In = Data;

    if (++Buffer->In == Buffer->End)
      Buffer->In = Buffer->Start;

    Buffer->Count++;
  }

  /** Removes an element from the ring buffer.
   *      *
   *     *  \warning Only one execution thread (main program thread or an ISR) may remove from a single buffer
   *     *           otherwise data corruption may occur. Insertion and removal may occur from different execution
   *     *           threads.
   *     *
   *     *  \param[in,out] Buffer  Pointer to a ring buffer structure to retrieve from.
   *     *
   *     *  \return Next data element stored in the buffer.
       */
  static inline uint8_t RingBuffer_Remove(RingBuffer_t* Buffer);
  static inline uint8_t RingBuffer_Remove(RingBuffer_t* Buffer)
  {

    uint8_t Data = *Buffer->Out;

    if (++Buffer->Out == Buffer->End)
      Buffer->Out = Buffer->Start;


    Buffer->Count--;


    return Data;
  }

  /** Returns the next element stored in the ring buffer, without removing it.
   *      *
   *     *  \param[in,out] Buffer  Pointer to a ring buffer structure to retrieve from.
   *     *
   *     *  \return Next data element stored in the buffer.
       */
  static inline uint8_t RingBuffer_Peek(RingBuffer_t* const Buffer);
  static inline uint8_t RingBuffer_Peek(RingBuffer_t* const Buffer)
  {
    return *Buffer->Out;
  }

#endif

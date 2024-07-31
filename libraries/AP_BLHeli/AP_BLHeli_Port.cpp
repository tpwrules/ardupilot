#include "AP_BLHeli.h"

#if HAVE_AP_BLHELI_SUPPORT

#include <AP_Math/AP_Math.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#ifndef AP_BLHELI_SERIAL_MIN_TXSIZE
#define AP_BLHELI_SERIAL_MIN_TXSIZE 2048
#endif

#ifndef AP_BLHELI_SERIAL_MIN_RXSIZE
#define AP_BLHELI_SERIAL_MIN_RXSIZE 2048
#endif

/*
  initialise port
 */
void AP_BLHeli::Port::init(void)
{
    begin(1000000, 0, 0); // assume 1MBaud rate even though it's a bit meaningless
}

void AP_BLHeli::Port::clear(void)
{
    WITH_SEMAPHORE(sem);
    if (readbuffer) {
        readbuffer->clear();
    }
    if (writebuffer) {
        writebuffer->clear();
    }
}

size_t AP_BLHeli::Port::device_write(const uint8_t *buffer, size_t size)
{
    WITH_SEMAPHORE(sem);
    if (readbuffer) {
        return readbuffer->write(buffer, size);
    }
    return 0;
}

ssize_t AP_BLHeli::Port::device_read(uint8_t *buffer, uint16_t count)
{
    WITH_SEMAPHORE(sem);
    if (writebuffer) {
        return writebuffer->read(buffer, count);
    }
    return 0;
}

uint32_t AP_BLHeli::Port::device_available(void)
{
    WITH_SEMAPHORE(sem);
    if (writebuffer) {
        return writebuffer->available();
    }
    return 0;
}

/*
  available space in outgoing buffer
 */
uint32_t AP_BLHeli::Port::txspace(void)
{
    WITH_SEMAPHORE(sem);
    return writebuffer != nullptr ? writebuffer->space() : 0;
}

void AP_BLHeli::Port::_begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    rxS = MAX(rxS, AP_BLHELI_SERIAL_MIN_RXSIZE);
    txS = MAX(txS, AP_BLHELI_SERIAL_MIN_TXSIZE);
    init_buffers(rxS, txS);
}

size_t AP_BLHeli::Port::_write(const uint8_t *buffer, size_t size)
{
    WITH_SEMAPHORE(sem);
    return writebuffer != nullptr ? writebuffer->write(buffer, size) : 0;
}

ssize_t AP_BLHeli::Port::_read(uint8_t *buffer, uint16_t count)
{
    WITH_SEMAPHORE(sem);
    return readbuffer != nullptr ? readbuffer->read(buffer, count) : -1;
}

uint32_t AP_BLHeli::Port::_available()
{
    WITH_SEMAPHORE(sem);
    return readbuffer != nullptr ? readbuffer->available() : 0;
}


bool AP_BLHeli::Port::_discard_input()
{
    WITH_SEMAPHORE(sem);
    if (readbuffer != nullptr) {
        readbuffer->clear();
    }
    return true;
}

/*
  initialise read/write buffers
 */
bool AP_BLHeli::Port::init_buffers(const uint32_t size_rx, const uint32_t size_tx)
{
    if (size_tx == last_size_tx &&
        size_rx == last_size_rx) {
        return true;
    }
    WITH_SEMAPHORE(sem);
    if (readbuffer == nullptr) {
        readbuffer = NEW_NOTHROW ByteBuffer(size_rx);
    } else {
        readbuffer->set_size_best(size_rx);
    }
    if (writebuffer == nullptr) {
        writebuffer = NEW_NOTHROW ByteBuffer(size_tx);
    } else {
        writebuffer->set_size_best(size_tx);
    }
    last_size_rx = size_rx;
    last_size_tx = size_tx;
    return readbuffer != nullptr && writebuffer != nullptr;
}

#endif // HAVE_AP_BLHELI_SUPPORT

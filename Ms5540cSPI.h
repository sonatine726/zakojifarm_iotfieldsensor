/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file Ms5540cSPI.h
 * @brief High-level SPI interface
 *
 * This is a "bare essentials" polling driver for now.
 */

/* TODO [0.1.0] Remove deprecated methods. */

#include "libmaple_types.h"
#include "spi.h"

#include "boards.h"
#include "wirish.h"

#include "HardwareSPI.h"

#ifndef _MS5540CSPI_H_
#define _MS5540CSPI_H_

/**
 * @brief Wirish SPI interface.
 *
 * This implementation uses software slave management, so the caller
 * is responsible for controlling the slave select line.
 */
class Ms5540cSPI {
public:
    /**
     * @param spiPortNumber Number of the SPI port to manage.
     */
    Ms5540cSPI(uint32 spiPortNumber);

    /*
     * Set up/tear down
     */

    /**
     * @brief Turn on a SPI port and set its GPIO pin modes for use as master.
     *
     * SPI port is enabled in full duplex mode, with software slave management.
     *
     * @param frequency Communication frequency
     * @param bitOrder Either LSBFIRST (little-endian) or MSBFIRST (big-endian)
     * @param mode SPI mode to use, one of SPI_MODE_0, SPI_MODE_1,
     *             SPI_MODE_2, and SPI_MODE_3.
     */
    void begin(SPIFrequency frequency, uint32 bitOrder, uint32 mode);

    /**
     * @brief Equivalent to begin(SPI_1_125MHZ, MSBFIRST, 0).
     */
    void begin(void);

    /**
     * @brief Turn on a SPI port and set its GPIO pin modes for use as a slave.
     *
     * SPI port is enabled in full duplex mode, with software slave management.
     *
     * @param bitOrder Either LSBFIRST (little-endian) or MSBFIRST(big-endian)
     * @param mode SPI mode to use
     */
    void beginSlave(uint32 bitOrder, uint32 mode);

    /**
     * @brief Equivalent to beginSlave(MSBFIRST, 0).
     */
    void beginSlave(void);

    /**
     * @brief Disables the SPI port, but leaves its GPIO pin modes unchanged.
     */
    void end(void);

    /*
     * I/O
     */

    /**
     * @brief Return the next unread byte.
     *
     * If there is no unread byte waiting, this function will block
     * until one is received.
     */
    uint8 read(void);

    /**
     * @brief Read length bytes, storing them into buffer.
     * @param buffer Buffer to store received bytes into.
     * @param length Number of bytes to store in buffer.  This
     *               function will block until the desired number of
     *               bytes have been read.
     */
    void read(uint8 *buffer, uint32 length);

	/**
     * @brief Read length bytes, storing them into buffer.
     * @param buffer Buffer to store received bytes into.
     * @param length Number of bytes to store in buffer.  This
     *               function will block until the desired number of
     *               bytes have been read.
     */
    void readMaster(uint8 *buffer, uint32 length);


	void waitReady();
    /**
     * @brief Transmit a byte.
     * @param data Byte to transmit.
     */
    void write(uint8 data);

    /**
     * @brief Transmit multiple bytes.
     * @param buffer Bytes to transmit.
     * @param length Number of bytes in buffer to transmit.
     */
    void write(const uint8 *buffer, uint32 length);

    /**
     * @brief Transmit a byte, then return the next unread byte.
     *
     * This function transmits before receiving.
     *
     * @param data Byte to transmit.
     * @return Next unread byte.
     */
    uint8 transfer(uint8 data);

    /*
     * Pin accessors
     */

    /**
     * @brief Return the number of the MISO (master in, slave out) pin
     */
    uint8 misoPin(void);

    /**
     * @brief Return the number of the MOSI (master out, slave in) pin
     */
    uint8 mosiPin(void);

    /**
     * @brief Return the number of the SCK (serial clock) pin
     */
    uint8 sckPin(void);

    /**
     * @brief Return the number of the NSS (slave select) pin
     */
    uint8 nssPin(void);

    /* -- The following methods are deprecated --------------------------- */

    /**
     * @brief Deprecated.
     *
     * Use Ms5540cSPI::transfer() instead.
     *
     * @see Ms5540cSPI::transfer()
     */
    uint8 send(uint8 data);

    /**
     * @brief Deprecated.
     *
     * Use Ms5540cSPI::write() in combination with
     * Ms5540cSPI::read() (or Ms5540cSPI::transfer()) instead.
     *
     * @see Ms5540cSPI::write()
     * @see Ms5540cSPI::read()
     * @see Ms5540cSPI::transfer()
     */
    uint8 send(const uint8 *data, uint32 length);

    /**
     * @brief Deprecated.
     *
     * Use Ms5540cSPI::read() instead.
     *
     * @see Ms5540cSPI::read()
     */
    uint8 recv(void);
    
//    void beginTransaction(SPISettings settings) { beginTransaction(BOARD_SPI_DEFAULT_SS, settings); }
//    void beginTransaction(uint8 pin, SPISettings settings);
    void endTransaction(void) { }
                            
private:
    spi_dev *spi_d;
};

#endif //_MS5540CSPI_H_

#pragma once
#ifndef _STWI2C_H_
#define _STWI2C_H_

#include <utility>
#include <Wire.h>

extern TwoWire i2c;

inline void WriteArgumentsToI2c() {}

template <typename Head, typename... Tail>
void WriteArgumentsToI2c(Head&& head, Tail&&... tail)
{
	i2c.write(head);
	WriteArgumentsToI2c(std::forward<Tail>(tail)...);
}

template <typename... ArgsType>
uint8 I2cTransmit(uint8 address, ArgsType... args)
{
	i2c.beginTransmission(address);
	WriteArgumentsToI2c(args...);
	return i2c.endTransmission();
}

#endif //_STWI2C_H_

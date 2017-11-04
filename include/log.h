#pragma once

#include <stdio.h>

#define ERROR(fmt, ...) printf("[Error] "); printf(fmt, ##__VA_ARGS__); printf("\n")
#define DEBUG(fmt, ...) printf("[Debug] "); printf(fmt, ##__VA_ARGS__); printf("\n")

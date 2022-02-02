// Copyright (c) 2020 SURGE, All rights reserved.
/**
 * @file       RvizMarker.cpp
 * @class      RvizMarker
 * @brief      Responsible for publishes rviz markers.
 */
/*=============================================================================
 * Change history:
 * ----------------------------------------------------------------------------
  * Revision DATE            BY           		SPR/US        REMARKS
 * 1.0      28-Sep-2020    	Nihal 		        NA 		   	  Initial Version
 * 2.0      30-Oct-2020     Nihal				NA			  Code documentation
==================================================================================*/

#include "Semaphore.h"

/*
	IN 			: count which is the starting semaphore value, type (0 for binary, 1 for counting semaphore)
	OUT 		: None
	DESCRIPTION	: Sets the count and type of the semaphore
*/
Semaphore::Semaphore(int count, int type) {
    _count = count;
    _type = type;
}

/*
	IN 			: Id of the calling thread
	OUT 		: None
	DESCRIPTION	: Update the semaphore count and notify any sleeping threads
*/
void Semaphore::notify(std::thread::id tid) {
    std::unique_lock<std::mutex> lock(_mtx);
    if((_type==0 && _count == 0) || _type == 1 )
        _count++;

    _cv.notify_one();
}

/*
	IN 			: count which is the starting semaphore value, type (0 for binary, 1 for counting semaphore)
	OUT 		: None
	DESCRIPTION	: If count is zero, the calling thread sleeps until count becomes non zero
*/
void Semaphore::wait(std::thread::id tid) {
    std::unique_lock<std::mutex> lock(_mtx);
    while(_count == 0) {
        _cv.wait(lock);
    }
    _count--;
}

/*
	IN 			: None
	OUT 		: count
	DESCRIPTION	: Returns the semaphore count value
*/
int Semaphore::getCount() {
    return _count;
}
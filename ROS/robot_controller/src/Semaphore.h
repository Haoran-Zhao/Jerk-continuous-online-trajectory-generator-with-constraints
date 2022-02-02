#pragma once

#include <mutex>
#include <condition_variable>
#include <thread>

/**
* Responsible for thread synchornization
*/
class Semaphore {

public:
    /**
    * Constructor
    * Creates Semaphore object
    * @param count Starting semaphore value
    * @param type Type of semaphore : 0 - Binary Semaphore, 1 - Counting Semaphore
    */
    Semaphore(int count = 0, int type = 0); //0 - Binary Semaphore, 1 - Counting Semaphore

    /**
    * Notifies the semaphore
    * @param tid Thread Id
    */
    void notify(std::thread::id tid);

    /**
    * Wait for the semaphore
    * @param tid Thread Id
    */
    void wait(std::thread::id tid);

    /**
    * Get the current sempahore count
    * @returns the sempahore count
    */
    int getCount();

private:

    /**
    * Stores the sempahore mutex
    */
    std::mutex _mtx;

    /**
    * Condition variable that performs locking based on checks
    */
    std::condition_variable _cv;

    /**
    * Stores the current sempahore value
    */
    int _count;

    /**
    * Stores the type of sempahore
    */
    int _type;
};
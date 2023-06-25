#include "TrafficLight.h"
#include "TrafficObject.h"
#include <chrono>
#include <mutex>
#include <thread>

// clang-format off

/* Implementation of class "MessageQueue" */

template <typename T>
T MessageQueue<T>::Receive()
{
    // NOTE: FP.5a : The method receive should use std::unique_lock<std::mutex> and _condition.wait() 
    // to wait for and receive new messages and pull them from the queue using move semantics. 
    // The received object should then be returned by the receive function. 

  std::unique_lock<std::mutex> ul(_mutex);
  _condition.wait(ul, [this]{return !_queue.empty();});

  auto phase = std::move(_queue.back());
  _queue.pop_back();
  return phase;
}

// NOTE: FP.4a : The method send should use the mechanisms std::lock_guard<std::mutex> 
// as well as _condition.notify_one() to add a new message to the queue and afterwards send a notification.
template <typename T>
void MessageQueue<T>::Send(T &&msg)
{
  std::lock_guard<std::mutex> lg(_mutex);
  _queue.emplace_back(std::move(msg));
  _condition.notify_one();
}

/* Implementation of class "TrafficLight" */

TrafficLight::TrafficLight()
{
  _currentPhase = TrafficLightPhase::red;
}


void TrafficLight::waitForGreen()
{
    // NOTE: FP.5b : add the implementation of the method waitForGreen, in which an infinite while-loop 
    // runs and repeatedly calls the receive function on the message queue. 
    // Once it receives TrafficLightPhase::green, the method returns.

  while (_phasesQueue.Receive() != TrafficLightPhase::green)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

TrafficLightPhase TrafficLight::getCurrentPhase() const
{
    return _currentPhase;
}

void TrafficLight::simulate()
{
    // NOTE: FP.2b : Finally, the private method „cycleThroughPhases“ should be started in a thread when the
    // public method „simulate“ is called. To do this, use the thread queue in the base class. 

  threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases, this));
}

// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases()
{
    // NOTE: FP.2a : Implement the function with an infinite loop that measures the time between two loop cycles 
    // and toggles the current phase of the traffic light between red and green and sends an update method 
    // to the message queue using move semantics. The cycle duration should be a random value between 4 and 6 seconds. 
    // Also, the while-loop should use std::this_thread::sleep_for to wait 1ms between two cycles. 

  // Set random generator
  std::random_device seed;
  auto gen = std::mt19937{seed()};
  auto dist = std::uniform_int_distribution<std::int32_t>{4000, 6000};

  int random_time = 6000;
  std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();

  while (true) {

    auto elapsed_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);

    if (elapsed_time.count() > random_time) {

      random_time = dist(gen);
      start = std::chrono::steady_clock::now();

      _currentPhase = _currentPhase == TrafficLightPhase::red ? TrafficLightPhase::green : TrafficLightPhase::red;

      // NOTE: Add a MessageQueue send !
      _phasesQueue.Send(std::move(_currentPhase));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

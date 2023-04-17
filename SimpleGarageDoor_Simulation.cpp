/**
 * @file  GarageDoor.h
 * @brief Class to maintain garage door state machine
 * @note
 *
 *
 * Given SM diagram use OO in C or C++ to implement:
 * 1. Functionality which represents this SM.
 * 2. The functionality shall include following, public methods
 *     a) Init function to execute initial transition.
 *     b) GetCurrentState() function to determine which state the SM is currently in.
 *     c) SetEvent() function to inject events into the SM and triggers transitions.
 * 3. The functionality shall include following, private methods:
 *     a) UpdateSm() - to reflect the injected event
 *     b) SetState() - for setting the new state.
 */
/*-----------------------------------------------------------------------*/
/* Header                                                                */
/*-----------------------------------------------------------------------*/
#include <iostream>
#include <chrono>
#include <thread>
#include <functional>
/*-----------------------------------------------------------------------*/
/* Macro                                                                 */
/*-----------------------------------------------------------------------*/
#define TIMEOUT_TEST      (5)   /* Test value of 5 sec */
#define GARAGE_OP_TIMEOUT (3)   /* Garage timeout value set to 3 sec */

/*-----------------------------------------------------------------------*/
/* DataType                                                              */
/*-----------------------------------------------------------------------*/
/*
 * Different states of Garage Door
 */
enum State{
    CLOSE,
    OPENING,
    OPENED,
    CLOSING,
    FAIL,
    STOPPED_CLOSING,
    STOPPED_OPENING
};

/*
 * Different events to be injected to Garage Door class
 */
enum Event{
    INITIAL_EVENT,
    BUTTON_PRESS,
    MOTION_COMPLETED,
    SENSOR_TRIGGER,
    RESET,
    MOTION_TIMEOUT
};

/* 
 * Garage Door Motion up, down, stop
 */
enum Motion_e{
    STOP,
    DOWN,
    UP
};

/*
 * LED light state
 */
enum Light_e{
    OFF,
    ON
};

/* Variable to keep track of time out in application */
volatile uint8_t timeout = false;

/*
 * Light class to perform LED tasks
 */
class Light{
    private:
        Light_e light_;
	uint8_t stopFlag_;

    public:
	uint8_t sec = 0;
	Light(){
            light_ = OFF;
            stopFlag_ = 1;
	}

        void SetLight(Light_e lt){
	    light_ = lt;
        }

        Light_e GetLight(void){
           return light_;
        } 

        void Blink_Start(void){
	    stopFlag_ =  0;
	    std::thread([&]() {
	    while(sec <= GARAGE_OP_TIMEOUT){
        	if(stopFlag_ == 1){
		    break;
		}
                if(light_ == OFF){
		    light_ = ON;
		}else{
		    light_ = OFF;
		}
                SetLight(light_);
		std::this_thread::sleep_for(std::chrono::seconds(1));
		sec++;
            }  
	    }).detach();
        }

        void Blink_Stop(void){
	    stopFlag_ = 1;
        }
};

/*
 * Motion Detector class to perform tasks
 */
class MotionDetector{
    private:
        Motion_e mt_;
	bool isRunning_;
	double timeout_;
	std::chrono::steady_clock::time_point startTime_;
        std::function<void()> cbFunc;	
    public:
	MotionDetector(){
	    mt_ = STOP;
	    ResetMotionTimer();
	    isRunning_ = false;
	    timeout_ = GARAGE_OP_TIMEOUT;
	}

	bool hasMotionTimerElapsed(double time){
            if (!isRunning_) {
                return false;
	    }

	    std::chrono::steady_clock::time_point curTime = std::chrono::steady_clock::now() ;
	    if(std::chrono::duration_cast<std::chrono::seconds>(curTime - startTime_).count() >= time){
                return true;
	    }
	    return false;
	}

	void ResetMotionTimer() {
            startTime_ = std::chrono::steady_clock::now();
            isRunning_ = true;
        }


	void registerTimeout(std::function<void()> callbackFunction){
	       cbFunc= callbackFunction;
	}

	void StartMotionTimer() {
            startTime_ = std::chrono::steady_clock::now();
            isRunning_ = true;
	    std::thread([&]() {
	    while(isRunning_){
		if(hasMotionTimerElapsed(timeout_) == true){
		    std::cout<<"Start Motion has timedout"<<std::endl;
		    cbFunc();
	            StopMotionTimer();
	            break;
		}
            }
	    }).detach();
	}

	void StopMotionTimer() {
	    isRunning_ = false;
	}

	void SetMotion(Motion_e mt){
	    mt_ = mt;
	}
};

/*
 * Main Garage door class to handle state machine
 */
class GarageDoor{
   private:
       State state_;
       Event event_;
       MotionDetector* md;
       Light* lt;

       /* Go to different states based on the events */
       void UpdateSm(){
	   State nextState = state_;
           if(event_ == MOTION_TIMEOUT){
               if(state_ == CLOSING || state_ == OPENING){
                   nextState = FAIL;
               }
            }

	   if(event_ == BUTTON_PRESS){
	       if(state_ == OPENING){
                   nextState = STOPPED_OPENING;
	       }else if(state_ == STOPPED_OPENING){
	           nextState = OPENING;
	       }else if(state_ == CLOSING){
                   nextState = STOPPED_CLOSING;
	       }else if(state_ == STOPPED_CLOSING){
                   nextState = CLOSING;
               }else if(state_ == CLOSE){
                   nextState = OPENING;
               }else if(state_ == OPENED){
                   nextState = CLOSING;
              }else if(state_ == CLOSING){
                   nextState = STOPPED_CLOSING;
              }else if(state_ == STOPPED_CLOSING){
                   nextState = CLOSING;
	      }
	   }else if(event_ == MOTION_COMPLETED){
	      if(state_ == OPENING){
                   nextState = OPENED;
              }else if(state_ == CLOSING){
		   nextState = CLOSE;
              }
	   }else if(event_ == SENSOR_TRIGGER){
              if(state_ == CLOSING){
                   nextState = OPENING;
              }
	   }else if(event_ == RESET){
	      if(state_ == FAIL){
	         nextState = OPENING;
	      }
	   }

	   if(nextState != state_){
	      exitStateHandling(md, lt);
              SetState(nextState);
	      entryStateHandling(md, lt);
	   }
       }

       void exitStateHandling(MotionDetector* md, Light* lt){
	    if(state_ == OPENING || state_ == CLOSING){
                std::cout<<"Motion Stop"<<std::endl;
	        md->StopMotionTimer();
	        lt->Blink_Stop();
	        md->SetMotion(STOP);
	    }else if(state_ == FAIL){
		lt->SetLight(OFF);
	    }
       }

       void entryStateHandling(MotionDetector* md, Light* lt){
	    if(state_ == OPENING || state_ == CLOSING){
	        md->StartMotionTimer();
	        lt->Blink_Start();
	        if(state_ == OPENING){
	               md->SetMotion(UP); 
		       std::cout<<"Starting Upward Motion"<<std::endl;
                }else{
	               md->SetMotion(DOWN); 
		       std::cout<<"Starting Downward Motion"<<std::endl;
	        }
	    }else if(state_ == FAIL){
		lt->SetLight(ON);
	    }
       }

       void SetState(State st){
           state_ = st;
       }

   public:
       GarageDoor(MotionDetector* m, Light* l){
	   state_ = CLOSE;
           event_ = INITIAL_EVENT;
           md = m;
	   lt = l;
       }

       void SetEvent(Event et){
            event_ = et;
            UpdateSm(); 
       }

       State GetCurrentState(){
            return state_;
       }
};
/*-----------------------------------------------------------------------*/
/* Prototype                                                             */
/*-----------------------------------------------------------------------*/
/*-----------------------------------------------------------------------*/
/* Variable                                                              */
/*-----------------------------------------------------------------------*/
/*-----------------------------------------------------------------------*/
/* Function                                                              */
/*-----------------------------------------------------------------------*/

void TimeoutHandler(){
   timeout = true;
}

int main(){
    std::cout<< "Garage Door Simulation"<<std::endl;
    int sec = 0;

    /* Initailize the objects */
    MotionDetector Md;
    Light lt;
    GarageDoor G(&Md, &lt);

    /* Register timeout callback */
    Md.registerTimeout(TimeoutHandler);
    std::cout<<"Initial State "<<G.GetCurrentState()<<std::endl;
    
    /* Inject events and check the state */
    std::cout<<"Set BUTTON_PRESS in CLOSE State"<<std::endl;
    G.SetEvent(BUTTON_PRESS);
    std::cout<<"After BUTTON_PRESS State "<<G.GetCurrentState()<<std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    std::cout<<"Set BUTTON_PRESS in OPENING State"<<std::endl;
    G.SetEvent(BUTTON_PRESS);
    std::cout<<"After BUTTON_PRESS State "<<G.GetCurrentState()<<std::endl;
    
    std::cout<<"Set BUTTON_PRESS in STOPPED_OPENING State"<<std::endl;
    G.SetEvent(BUTTON_PRESS);
    std::cout<<"After BUTTON_PRESS State "<<G.GetCurrentState()<<std::endl;

    std::cout<<"Simulate timeout in OPENING State"<<std::endl;
    /* Wait for more than the time out */
    while(sec < TIMEOUT_TEST){
	if(timeout == true){	
            G.SetEvent(MOTION_TIMEOUT);
	    break;
	}
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	sec++;
    }
    std::cout<<"After Timeout in OPENING State, Curren State "<<G.GetCurrentState()<<std::endl;


    std::cout<<"Set RESET in FAIL State"<<std::endl;
    G.SetEvent(RESET);
    std::cout<<"After RESET in FAIL State, Current State "<<G.GetCurrentState()<<std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    std::cout<<"Set MOTION_COMPLETED in OPENING State"<<std::endl;
    G.SetEvent(MOTION_COMPLETED);
    std::cout<<"After MOTION_COMPLETED Current State "<<G.GetCurrentState()<<std::endl;


    std::cout<<"Set BUTTON_PRESS in OPENED State"<<std::endl;
    G.SetEvent(BUTTON_PRESS);
    std::cout<<"Button Pressed in OPENED State, Current State "<<G.GetCurrentState()<<std::endl;

    std::cout<<"Set MOTION_COMPLETED in CLOSING State"<<std::endl;
    G.SetEvent(MOTION_COMPLETED);
    std::cout<<"After MOTION_COMPLETED Current State "<<G.GetCurrentState()<<std::endl;

    /* Second loop */
    std::cout<<"Set BUTTON_PRESS in CLOSE State"<<std::endl;
    G.SetEvent(BUTTON_PRESS);
    std::cout<<"After BUTTON_PRESS State "<<G.GetCurrentState()<<std::endl;

    std::cout<<"Set MOTION_COMPLETED in OPENING State"<<std::endl;
    G.SetEvent(MOTION_COMPLETED);
    std::cout<<"After MOTION_COMPLETED Current State "<<G.GetCurrentState()<<std::endl;

    std::cout<<"Set BUTTON_PRESS in OPENED State"<<std::endl;
    G.SetEvent(BUTTON_PRESS);
    std::cout<<"Button Pressed in OPENED State, Current State "<<G.GetCurrentState()<<std::endl;

    std::cout<<"Set SENSOR_TRIGGER in CLOSING State"<<std::endl;
    G.SetEvent(BUTTON_PRESS);
    std::cout<<"Sensor triggered in closing state, Current State "<<G.GetCurrentState()<<std::endl;

    std::cout<<"Set MOTION_COMPLETED in OPENING State"<<std::endl;
    G.SetEvent(MOTION_COMPLETED);
    std::cout<<"After MOTION_COMPLETED Current State "<<G.GetCurrentState()<<std::endl;

    std::cout<<"Set BUTTON_PRESS in OPENED State"<<std::endl;
    G.SetEvent(BUTTON_PRESS);
    std::cout<<"Button Pressed in OPENED State, Current State "<<G.GetCurrentState()<<std::endl;

    std::cout<<"Set BUTTON_PRESS in CLOSING State"<<std::endl;
    G.SetEvent(BUTTON_PRESS);
    std::cout<<"Button Pressed in CLOSING State, Current State "<<G.GetCurrentState()<<std::endl;

    std::cout<<"Set BUTTON_PRESS in STOPPED_CLOSING State"<<std::endl;
    G.SetEvent(BUTTON_PRESS);
    std::cout<<"Button Pressed in STOPPED_CLOSING State, Current State "<<G.GetCurrentState()<<std::endl;

    std::cout<<"Simulate timeout in CLOSING State"<<std::endl;
    sec = 0;
    timeout = false;
    while(sec < TIMEOUT_TEST){
	if(timeout == true){	
            G.SetEvent(MOTION_TIMEOUT);
	    break;
	}
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	sec++;
    }

    std::cout<<"After Timeout in Closing State, Curren State "<<G.GetCurrentState()<<std::endl;


    G.SetEvent(RESET);
    std::cout<<"After RESET in Fail State, Current State "<<G.GetCurrentState()<<std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    return 0;
}
